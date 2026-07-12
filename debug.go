package main

import (
	"fmt"
	"io"
	"time"
)

// DebugRawCommand sends a named command and dumps the raw request/response bytes.
func (cam *XU81Camera) DebugRawCommand(cmdName string, params []byte) error {
	cmd, ok := commands[cmdName]
	if !ok {
		return fmt.Errorf("unknown command: %s", cmdName)
	}

	cam.mu.Lock()
	defer cam.mu.Unlock()
	if err := cam.ensureConnectedLocked(); err != nil {
		return err
	}

	cmdBytes := formatCommandBytes(cmd, params)
	fmt.Printf("Sending %s: %X\n", cmd.name, cmdBytes)

	if _, err := cam.serial.Write(cmdBytes); err != nil {
		cam.markDisconnectedLocked()
		return fmt.Errorf("write error: %w", err)
	}

	// Read response with a longer timeout, restoring the configured one after
	cam.serial.SetReadTimeout(500 * time.Millisecond)
	defer cam.serial.SetReadTimeout(cam.config.ReadTimeout)

	totalRead := 0
	var fullResponse []byte

	for totalRead < 256 { // Max response size
		buf := make([]byte, 256)
		n, err := cam.serial.Read(buf)
		if err != nil && err != io.EOF {
			if totalRead > 0 {
				break // We got some data, that's enough
			}
			return fmt.Errorf("read error: %w", err)
		}

		if n > 0 {
			fullResponse = append(fullResponse, buf[:n]...)
			totalRead += n

			// Check for end marker
			if len(fullResponse) > 0 && fullResponse[len(fullResponse)-1] == 0xEF {
				break
			}
		}

		if err == io.EOF {
			break
		}
	}

	fmt.Printf("Received %d bytes: %X\n", len(fullResponse), fullResponse)

	// Try to interpret the response
	if len(fullResponse) > 0 {
		switch fullResponse[0] {
		case 0x06:
			fmt.Println("Response: ACK")
		case 0x15:
			fmt.Println("Response: NAK")
		case 0xFE:
			fmt.Println("Response: Status/Data response")
			if len(fullResponse) >= 6 {
				errorCode := string(fullResponse[3:5])
				fmt.Printf("Error code: %s\n", errorCode)
				if len(fullResponse) > 6 {
					fmt.Printf("Data portion: %X\n", fullResponse[5:len(fullResponse)-1])
				}
			}
		default:
			fmt.Printf("Unknown response type: 0x%02X\n", fullResponse[0])
		}
	}

	return nil
}

// Add this debug function to check focus mode
func (cam *XU81Camera) DebugFocusControl() error {
	// First check camera status
	status, err := cam.GetStatus()
	if err != nil {
		return fmt.Errorf("failed to get status: %w", err)
	}

	fmt.Printf("Camera Power: %v\n", status.PowerOn)
	fmt.Printf("Manual Focus Mode: %v\n", status.ManualFocus)
	fmt.Printf("Currently Focusing: %v\n", status.Focusing)

	// Try setting manual focus mode
	fmt.Println("Setting manual focus mode...")
	if err := cam.FocusManual(); err != nil {
		return fmt.Errorf("failed to set manual focus: %w", err)
	}

	time.Sleep(500 * time.Millisecond)

	// Check status again
	status, err = cam.GetStatus()
	if err != nil {
		return fmt.Errorf("failed to get status after manual focus: %w", err)
	}
	fmt.Printf("Manual Focus Mode after setting: %v\n", status.ManualFocus)

	// Now try focus far
	fmt.Println("Attempting focus far...")
	if err := cam.FocusFar(); err != nil {
		return fmt.Errorf("focus far failed: %w", err)
	}

	return nil
}
