package main

import (
	"errors"
	"fmt"
	"time"
)

// DebugRawCommand sends a named command with raw parameters (bypassing
// validation) and dumps the request/response bytes.
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

	_ = cam.serial.ResetInputBuffer()

	cmdBytes := formatCommandBytes(cmd, params)
	fmt.Printf("Sending %s: %X\n", cmd.name, cmdBytes)

	if _, err := cam.serial.Write(cmdBytes); err != nil {
		cam.markDisconnectedLocked()
		return fmt.Errorf("write error: %w", err)
	}

	resp, err := cam.readResponse()
	if err != nil {
		if !errors.Is(err, errResponseTimeout) {
			cam.markDisconnectedLocked()
		}
		return fmt.Errorf("read error: %w", err)
	}

	fmt.Printf("Received %d bytes: %X\n", len(resp.Data), resp.Data)
	switch resp.Type {
	case ResponseACK:
		fmt.Println("Response: ACK")
	case ResponseNAK:
		fmt.Println("Response: NAK")
	case ResponseStatus, ResponseError:
		fmt.Println("Response: Status/Data response")
		if resp.ErrorCode != "" {
			fmt.Printf("Error code: %s (%v)\n", resp.ErrorCode, resp.Error)
		}
		if len(resp.Data) > 6 {
			fmt.Printf("Data portion: %X\n", resp.Data[5:len(resp.Data)-1])
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
