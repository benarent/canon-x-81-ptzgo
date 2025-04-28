package main

import (
	"bufio"
	"encoding/hex"
	"errors"
	"fmt"
	"io"
	"log"
	"os"
	"strconv"
	"strings"
	"time"

	"go.bug.st/serial"
)

const (
	headerByte  = 0xFF
	deviceNum   = "3030"
	endMarkByte = 0xEF
)

type XU81Camera struct {
	port   string
	serial serial.Port
}

// OpenPort opens the serial connection to the camera
func (cam *XU81Camera) OpenPort() error {
	mode := &serial.Mode{
		BaudRate: 9600, // As specified in the manual
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(cam.port, mode)
	if err != nil {
		return fmt.Errorf("failed to open serial port: %v", err)
	}

	// The manual suggests 300 ms for answer wait
	port.SetReadTimeout(300 * time.Millisecond)

	cam.serial = port
	return nil
}

// ClosePort closes the serial connection
func (cam *XU81Camera) ClosePort() error {
	if cam.serial != nil {
		return cam.serial.Close()
	}
	return nil
}

// readResponse now understands ACK/NAK and variable-length frames.
func (cam *XU81Camera) readResponse() ([]byte, error) {
	const (
		ack = 0x06
		nak = 0x15
	)

	var buf []byte
	deadline := time.Now().Add(500 * time.Millisecond)

	for time.Now().Before(deadline) {
		tmp := make([]byte, 32)
		n, err := cam.serial.Read(tmp)
		if err != nil && !errors.Is(err, io.EOF) {
			return nil, fmt.Errorf("read error: %w", err)
		}
		if n == 0 {
			continue
		}
		buf = append(buf, tmp[:n]...)

		switch buf[0] {
		case ack:
			return buf[:1], nil // success
		case nak:
			return nil, fmt.Errorf("NAK") // generic error; decode if needed
		case 0xFE:
			if buf[len(buf)-1] == 0xEF && len(buf) >= 6 {
				// long status frame finished – validate error flags
				if string(buf[3:5]) != "00" { // ASCII “00” = 0x30 0x30
					return nil, parseErrorCode(fmt.Sprintf("%02X%02X", buf[3], buf[4]))
				}
				return buf, nil
			}
		}
	}
	return nil, fmt.Errorf("timeout waiting for response")
}

type Command struct {
	name        string
	code        string
	paramLength int
	description string
}

var commands = map[string]Command{
	"pan_right": {
		name:        "Pan Right Start",
		code:        "0153",
		paramLength: 1,
		description: "Start panning right",
	},
	"pan_left": {
		name:        "Pan Left Start",
		code:        "0153",
		paramLength: 1,
		description: "Start panning left",
	},
	"tilt_up": {
		name:        "Tilt Up Start",
		code:        "0153",
		paramLength: 1,
		description: "Start tilting up",
	},
	"tilt_down": {
		name:        "Tilt Down Start",
		code:        "0153",
		paramLength: 1,
		description: "Start tilting down",
	},
	"pan_speed": {
		name:        "Pan Speed Assignment",
		code:        "0150",
		paramLength: 3,
		description: "Set panning speed (5-3000)",
	},
	"tilt_speed": {
		name:        "Tilt Speed Assignment",
		code:        "0151",
		paramLength: 3,
		description: "Set tilting speed (5-3000)",
	},
	"pan_tilt_pos": {
		name:        "Pan/Tilt Position Assignment",
		code:        "0162",
		paramLength: 8,
		description: "Set absolute pan/tilt position",
	},
	"power_on": {
		name:        "Camera Power ON",
		code:        "01A0",
		paramLength: 1,
		description: "Turn on the camera power",
	},
	"power_off": {
		name:        "Camera Power OFF",
		code:        "01A0",
		paramLength: 1,
		description: "Turn off the camera power",
	},
	"pan_stop": {
		name:        "Pan/Tilt Stop",
		code:        "0153",
		paramLength: 1,
		description: "Stop pan/tilt movement",
	},
	"zoom_pos": {
		name:        "Zoom Position",
		code:        "01A3",
		paramLength: 2,
		description: "Set zoom position (00h-EEh)",
	},
	"focus_auto": {
		name:        "Focus Auto",
		code:        "01A1",
		paramLength: 1,
		description: "Enable auto focus",
	},
	"status": {
		name:        "Operation Status",
		code:        "0186",
		paramLength: 0,
		description: "Get camera operation status",
	},
	"washer_control": {
		name:        "Washer Control",
		code:        "017A",
		paramLength: 1,
		description: "Auto-washer (4 wiper cycles, washer on first 2)",
	},
	"washer_out": {
		name:        "Washer Output On/Off",
		code:        "0171",
		paramLength: 2,
		description: "Manually turn washer on or off",
	},

	// New commands for IR Filter, WDR, ND Filter
	"ir_filter": {
		name:        "IR Filter ON/OFF",
		code:        "01BB",
		paramLength: 1,
		description: "Toggle IR filter",
	},
	"wdr": {
		name:        "WDR ON/OFF",
		code:        "01A5",
		paramLength: 1,
		description: "Wide Dynamic Range on/off",
	},
	"nd_filter": {
		name:        "ND Filter ON/OFF",
		code:        "01B6",
		paramLength: 1,
		description: "Neutral Density filter on/off",
	},
}

func formatCommandBytes(cmd Command, params []byte) []byte {
	cmdBytes := []byte{headerByte}
	devNum, _ := hex.DecodeString(deviceNum)
	cmdBytes = append(cmdBytes, devNum...)

	cmdCode, _ := hex.DecodeString(cmd.code)
	cmdBytes = append(cmdBytes, cmdCode...)

	if len(params) > 0 {
		cmdBytes = append(cmdBytes, params...)
	}

	cmdBytes = append(cmdBytes, endMarkByte)
	return cmdBytes
}

// parseErrorCode interprets the camera's ASCII-hex error flags
func parseErrorCode(code string) error {
	if code == "3030" {
		return nil // "00" in ASCII-hex => no error
	}
	b, err := hex.DecodeString(code)
	if err != nil || len(b) != 2 {
		return fmt.Errorf("invalid error code: %s", code)
	}
	var errs []string

	// First byte
	if b[0]&0x80 != 0 {
		errs = append(errs, "Mode Error")
	}
	if b[0]&0x40 != 0 {
		errs = append(errs, "Parameter Error")
	}
	if b[0]&0x20 != 0 {
		errs = append(errs, "Command Error")
	}
	if b[0]&0x10 != 0 {
		errs = append(errs, "Busy")
	}
	// Second byte
	if b[1]&0x01 != 0 {
		errs = append(errs, "System Error")
	}

	if len(errs) == 0 {
		return fmt.Errorf("unknown error code: %s", code)
	}
	return fmt.Errorf("camera errors: %s", strings.Join(errs, ", "))
}

func (cam *XU81Camera) sendCommand(cmd Command, params []byte) ([]byte, error) {
	if cam.serial == nil {
		return nil, fmt.Errorf("serial port not open")
	}

	cmdBytes := formatCommandBytes(cmd, params)

	log.Printf("Sending command: %X\n", cmdBytes)
	if _, err := cam.serial.Write(cmdBytes); err != nil {
		return nil, fmt.Errorf("failed to send command: %v", err)
	}

	resp, err := cam.readResponse()
	if err != nil {
		return nil, fmt.Errorf("command failed: %w", err)
	}
	if len(resp) == 1 && resp[0] == 0x06 {
		return resp, nil // simple ACK, nothing more to print
	}
	return resp, nil
}

// ==========================
//
//	Existing Methods
//
// ==========================
func (cam *XU81Camera) PowerOn() error {
	c := commands["power_on"]
	_, err := cam.sendCommand(c, []byte{0x31}) // param=1 => ASCII '1'
	if err != nil {
		return fmt.Errorf("power on failed: %v", err)
	}
	time.Sleep(15 * time.Second) // Let camera boot
	return nil
}

func (cam *XU81Camera) PowerOff() error {
	c := commands["power_off"]
	_, err := cam.sendCommand(c, []byte{0x30}) // param=0 => ASCII '0'
	return err
}

func (cam *XU81Camera) Stop() error {
	c := commands["pan_stop"]
	_, err := cam.sendCommand(c, []byte{0x30}) // param=0 => ASCII '0'
	return err
}

func (cam *XU81Camera) SetZoomPosition(pos int) error {
	if pos < 0 || pos > 0xEE {
		return fmt.Errorf("zoom position must be 0..238")
	}
	c := commands["zoom_pos"]
	posHex := fmt.Sprintf("%02X", pos)
	params, _ := hex.DecodeString(posHex)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) GetStatus() ([]byte, error) {
	c := commands["status"]
	return cam.sendCommand(c, nil)
}

func (cam *XU81Camera) PanRight() error {
	c := commands["pan_right"]
	_, err := cam.sendCommand(c, []byte{0x31}) // param=1 => ASCII '1'
	return err
}

func (cam *XU81Camera) PanLeft() error {
	c := commands["pan_left"]
	_, err := cam.sendCommand(c, []byte{0x32})
	return err
}

func (cam *XU81Camera) TiltUp() error {
	c := commands["tilt_up"]
	_, err := cam.sendCommand(c, []byte{0x33})
	return err
}

func (cam *XU81Camera) TiltDown() error {
	c := commands["tilt_down"]
	_, err := cam.sendCommand(c, []byte{0x34})
	return err
}

func (cam *XU81Camera) SetPanSpeed(speed int) error {
	if speed < 5 || speed > 3000 {
		return fmt.Errorf("pan speed must be 5..3000")
	}
	c := commands["pan_speed"]
	speedHex := fmt.Sprintf("%03X", speed)
	params, _ := hex.DecodeString(speedHex)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) SetTiltSpeed(speed int) error {
	if speed < 5 || speed > 3000 {
		return fmt.Errorf("tilt speed must be 5..3000")
	}
	c := commands["tilt_speed"]
	speedHex := fmt.Sprintf("%03X", speed)
	params, _ := hex.DecodeString(speedHex)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) SetPanTiltPosition(pan, tilt int) error {
	// Pan: -9000..9000 => offset by 0x8000 => 0x5CD8..0xA328
	// Tilt: -6500..6500 => offset by 0x8000 => 0x669C..0x9964
	if pan < -9000 || pan > 9000 {
		return fmt.Errorf("pan angle must be -9000..9000")
	}
	if tilt < -6500 || tilt > 6500 {
		return fmt.Errorf("tilt angle must be -6500..6500")
	}

	c := commands["pan_tilt_pos"]
	panHex := fmt.Sprintf("%04X", uint16(pan+0x8000))
	tiltHex := fmt.Sprintf("%04X", uint16(tilt+0x8000))

	params, _ := hex.DecodeString(panHex + tiltHex)
	_, err := cam.sendCommand(c, params)
	return err
}

// ==========================
//
//	Washer Methods
//
// ==========================
func (cam *XU81Camera) WasherControl() error {
	// 0x017A param=1 => '1' => auto sequence: 4 cycles, washer on for first 2
	c := commands["washer_control"]
	_, err := cam.sendCommand(c, []byte{0x31})
	return err
}

func (cam *XU81Camera) WasherOutputOn() error {
	// 0x0171 param=B1 => ASCII "B1" => hex "42 31"
	c := commands["washer_out"]
	params, _ := hex.DecodeString("4231")
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) WasherOutputOff() error {
	// 0x0171 param=B0 => ASCII "B0" => hex "42 30"
	c := commands["washer_out"]
	params, _ := hex.DecodeString("4230")
	_, err := cam.sendCommand(c, params)
	return err
}

// ==========================
//  NEW: IR, WDR, ND Filters
// ==========================

// IRFilterOn sets IR filter ON (0x01BB param=1 => '1')
func (cam *XU81Camera) IRFilterOn() error {
	c := commands["ir_filter"]
	_, err := cam.sendCommand(c, []byte{0x31}) // '1'
	return err
}

// IRFilterOff sets IR filter OFF (param=0 => '0')
func (cam *XU81Camera) IRFilterOff() error {
	c := commands["ir_filter"]
	_, err := cam.sendCommand(c, []byte{0x30}) // '0'
	return err
}

// WDROn (0x01A5 param=1 => '1')
func (cam *XU81Camera) WDROn() error {
	c := commands["wdr"]
	_, err := cam.sendCommand(c, []byte{0x31})
	return err
}

// WDROff (0x01A5 param=0 => '0')
func (cam *XU81Camera) WDROff() error {
	c := commands["wdr"]
	_, err := cam.sendCommand(c, []byte{0x30})
	return err
}

// NDFilterOn (0x01B6 param=1 => '1')
func (cam *XU81Camera) NDFilterOn() error {
	c := commands["nd_filter"]
	_, err := cam.sendCommand(c, []byte{0x31})
	return err
}

// NDFilterOff (0x01B6 param=0 => '0')
func (cam *XU81Camera) NDFilterOff() error {
	c := commands["nd_filter"]
	_, err := cam.sendCommand(c, []byte{0x30})
	return err
}

func printHelp() {
	fmt.Println("\nAvailable commands:")
	fmt.Println("on          - Power on camera")
	fmt.Println("off         - Power off camera")
	fmt.Println("stop        - Stop all movement")
	fmt.Println("right       - Start panning right")
	fmt.Println("left        - Start panning left")
	fmt.Println("up          - Start tilting up")
	fmt.Println("down        - Start tilting down")
	fmt.Println("panspeed N  - Set pan speed (5-3000)")
	fmt.Println("tiltspeed N - Set tilt speed (5-3000)")
	fmt.Println("goto X Y    - Move to pan/tilt position (angles)")
	fmt.Println("zoom N      - Set zoom position (0-238)")
	fmt.Println("status      - Get camera status")

	// Washer
	fmt.Println("washer      - Run 4-cycle washer sequence")
	fmt.Println("washeron    - Turn washer output ON manually")
	fmt.Println("washeroff   - Turn washer output OFF manually")

	// IR, WDR, ND
	fmt.Println("ir on/off   - IR Filter ON/OFF")
	fmt.Println("wdr on/off  - WDR ON/OFF")
	fmt.Println("nd on/off   - ND filter ON/OFF")

	fmt.Println("help        - Show this help")
	fmt.Println("quit        - Exit program")
}

func main() {
	portName := "/dev/ttyUSB0"
	if len(os.Args) > 1 {
		portName = os.Args[1]
	}

	camera := &XU81Camera{port: portName}

	if err := camera.OpenPort(); err != nil {
		log.Fatalf("Failed to open serial port: %v", err)
	}
	defer camera.ClosePort()

	reader := bufio.NewReader(os.Stdin)

	fmt.Printf("XU-81 Camera Test Program (Connected to %s)\n", portName)
	fmt.Println("Type 'help' for commands")

	for {
		fmt.Print("\n> ")
		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input)
		parts := strings.Fields(input)
		if len(parts) == 0 {
			continue
		}

		cmd := parts[0]

		switch cmd {
		case "help":
			printHelp()

		case "on":
			err := camera.PowerOn()
			if err != nil {
				log.Printf("Error: %v\n", err)
			} else {
				fmt.Println("Camera powered on")
			}

		case "off":
			err := camera.PowerOff()
			if err != nil {
				log.Printf("Error: %v\n", err)
			} else {
				fmt.Println("Camera powered off")
			}

		case "stop":
			err := camera.Stop()
			if err != nil {
				log.Printf("Error: %v\n", err)
			} else {
				fmt.Println("Movement stopped")
			}

		case "zoom":
			if len(parts) != 2 {
				fmt.Println("Usage: zoom <position>")
				continue
			}
			pos, err := strconv.Atoi(parts[1])
			if err != nil {
				fmt.Println("Invalid zoom position")
				continue
			}
			err = camera.SetZoomPosition(pos)
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		case "status":
			status, err := camera.GetStatus()
			if err != nil {
				log.Printf("Error: %v\n", err)
			} else {
				fmt.Printf("Status: %X\n", status)
			}

		case "right":
			err := camera.PanRight()
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		case "left":
			err := camera.PanLeft()
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		case "up":
			err := camera.TiltUp()
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		case "down":
			err := camera.TiltDown()
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		case "panspeed":
			if len(parts) != 2 {
				fmt.Println("Usage: panspeed <speed>")
				continue
			}
			speed, err := strconv.Atoi(parts[1])
			if err != nil {
				fmt.Println("Invalid speed value")
				continue
			}
			err = camera.SetPanSpeed(speed)
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		case "tiltspeed":
			if len(parts) != 2 {
				fmt.Println("Usage: tiltspeed <speed>")
				continue
			}
			speed, err := strconv.Atoi(parts[1])
			if err != nil {
				fmt.Println("Invalid speed value")
				continue
			}
			err = camera.SetTiltSpeed(speed)
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		case "goto":
			if len(parts) != 3 {
				fmt.Println("Usage: goto <pan> <tilt>")
				continue
			}
			panVal, err := strconv.Atoi(parts[1])
			if err != nil {
				fmt.Println("Invalid pan angle")
				continue
			}
			tiltVal, err := strconv.Atoi(parts[2])
			if err != nil {
				fmt.Println("Invalid tilt angle")
				continue
			}
			err = camera.SetPanTiltPosition(panVal, tiltVal)
			if err != nil {
				log.Printf("Error: %v\n", err)
			}

		// Washer commands:
		case "washer":
			if err := camera.WasherControl(); err != nil {
				log.Printf("Error: %v\n", err)
			} else {
				fmt.Println("Washer auto-sequence started (4 cycles).")
			}

		case "washeron":
			if err := camera.WasherOutputOn(); err != nil {
				log.Printf("Error: %v\n", err)
			} else {
				fmt.Println("Washer output forced ON.")
			}

		case "washeroff":
			if err := camera.WasherOutputOff(); err != nil {
				log.Printf("Error: %v\n", err)
			} else {
				fmt.Println("Washer output turned OFF.")
			}

		// IR / WDR / ND
		case "ir":
			if len(parts) != 2 {
				fmt.Println("Usage: ir <on|off>")
				continue
			}
			switch parts[1] {
			case "on":
				if err := camera.IRFilterOn(); err != nil {
					log.Printf("Error: %v\n", err)
				} else {
					fmt.Println("IR Filter ON.")
				}
			case "off":
				if err := camera.IRFilterOff(); err != nil {
					log.Printf("Error: %v\n", err)
				} else {
					fmt.Println("IR Filter OFF.")
				}
			default:
				fmt.Println("Usage: ir <on|off>")
			}

		case "wdr":
			if len(parts) != 2 {
				fmt.Println("Usage: wdr <on|off>")
				continue
			}
			switch parts[1] {
			case "on":
				if err := camera.WDROn(); err != nil {
					log.Printf("Error: %v\n", err)
				} else {
					fmt.Println("WDR ON.")
				}
			case "off":
				if err := camera.WDROff(); err != nil {
					log.Printf("Error: %v\n", err)
				} else {
					fmt.Println("WDR OFF.")
				}
			default:
				fmt.Println("Usage: wdr <on|off>")
			}

		case "nd":
			if len(parts) != 2 {
				fmt.Println("Usage: nd <on|off>")
				continue
			}
			switch parts[1] {
			case "on":
				if err := camera.NDFilterOn(); err != nil {
					log.Printf("Error: %v\n", err)
				} else {
					fmt.Println("ND Filter ON.")
				}
			case "off":
				if err := camera.NDFilterOff(); err != nil {
					log.Printf("Error: %v\n", err)
				} else {
					fmt.Println("ND Filter OFF.")
				}
			default:
				fmt.Println("Usage: nd <on|off>")
			}

		case "quit":
			return

		default:
			fmt.Println("Unknown command. Type 'help' for available commands")
		}
	}
}
