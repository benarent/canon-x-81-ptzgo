package main

import (
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/peterh/liner"
	"go.bug.st/serial"
)

// Protocol constants
const (
	headerByte         = 0xFF
	responseHeader     = 0xFE
	deviceNum          = "3030"
	endMarkByte        = 0xEF
	ackByte            = 0x06
	nakByte            = 0x15
	defaultReadTimeout = 300 * time.Millisecond
	responseTimeout    = 500 * time.Millisecond
	powerOnBootDelay   = 15 * time.Second
	paramOn            = 0x31 // ASCII '1'
	paramOff           = 0x30 // ASCII '0'
	minSpeed           = 5
	maxSpeed           = 3000
	maxZoom            = 0xEE
	minPanAngle        = -9000
	maxPanAngle        = 9000
	minTiltAngle       = -6500
	maxTiltAngle       = 6500
)

// Camera model types
type CameraModel int

const (
	ModelXU80 CameraModel = iota
	ModelXU81
)

// Digital zoom modes (XU-81 only)
type DigitalZoomMode int

const (
	OpticalInterlocking DigitalZoomMode = iota
	Independent
)

// Shooting modes
type ShootingMode int

const (
	ShootingAuto ShootingMode = iota
	ShootingManual
	ShootingNight
	ShootingShutterPriority
	ShootingIrisPriority
)

// Response types
type ResponseType int

const (
	ResponseACK ResponseType = iota
	ResponseNAK
	ResponseStatus
	ResponseError
)

// Config holds the camera configuration
type Config struct {
	Port             string          `json:"port"`
	BaudRate         int             `json:"baud_rate"`
	ReadTimeout      time.Duration   `json:"read_timeout"`
	PowerOnDelay     time.Duration   `json:"power_on_delay"`
	DefaultPanSpeed  int             `json:"default_pan_speed"`
	DefaultTiltSpeed int             `json:"default_tilt_speed"`
	CameraModel      CameraModel     `json:"camera_model"`
	DigitalZoomMode  DigitalZoomMode `json:"digital_zoom_mode"`
}

// Add this method to your XU81Camera struct (anywhere after the struct definition):
func (cam *XU81Camera) DebugRawCommand(cmdName string, params []byte) error {
	cmd, ok := commands[cmdName]
	if !ok {
		return fmt.Errorf("unknown command: %s", cmdName)
	}

	if err := cam.ensureConnected(); err != nil {
		return err
	}

	cmdBytes := formatCommandBytes(cmd, params)
	fmt.Printf("Sending %s: %X\n", cmd.name, cmdBytes)

	if _, err := cam.serial.Write(cmdBytes); err != nil {
		return fmt.Errorf("write error: %w", err)
	}

	// Read response with timeout
	cam.serial.SetReadTimeout(500 * time.Millisecond)

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

// LoadConfig loads configuration from a file
func LoadConfig(filename string) (*Config, error) {
	cfg := &Config{
		Port:            "/dev/ttyUSB0",
		BaudRate:        9600,
		ReadTimeout:     defaultReadTimeout,
		PowerOnDelay:    powerOnBootDelay,
		CameraModel:     ModelXU81,
		DigitalZoomMode: OpticalInterlocking,
	}

	if filename == "" {
		return cfg, nil
	}

	data, err := os.ReadFile(filename)
	if err != nil {
		return cfg, err
	}

	if err := json.Unmarshal(data, &cfg); err != nil {
		return nil, err
	}

	return cfg, nil
}

// SerialPort interface for better testability
type SerialPort interface {
	Read([]byte) (int, error)
	Write([]byte) (int, error)
	Close() error
	SetReadTimeout(time.Duration) error
}

// Command represents a camera command
type Command struct {
	name        string
	code        string
	paramLength int
	description string
	validator   func(params []byte, model CameraModel) error
}

// Response represents a parsed camera response
type Response struct {
	Type      ResponseType
	Data      []byte
	ErrorCode string
	Error     error
}

// CameraStatus represents the parsed camera status
type CameraStatus struct {
	// Basic status (12 bits)
	PowerOn           bool
	Focusing          bool
	ManualFocus       bool
	PanLimitPosition  bool
	Panning           bool
	TiltLimitPosition bool
	Tilting           bool
	Zooming           bool

	// Extended status (20 bits)
	CameraBusy          bool
	CameraManualMode    bool
	WhiteBalanceNotAuto bool
	PanMotorError       bool
	TiltMotorError      bool

	// Detailed status
	PanPosition   int
	TiltPosition  int
	ZoomPosition  int
	FocusPosition int
	ShootingMode  ShootingMode
	IRFilter      bool
	WDR           bool
	NDFilter      bool
	Raw           []byte
}

// SystemInfo represents system information
type SystemInfo struct {
	ProductName   string
	ROMVersion    string
	SerialNumber  string
	CameraVersion int
}

// ShotMemory represents a shot memory preset
type ShotMemory struct {
	PanPosition   int
	TiltPosition  int
	ZoomPosition  int
	FocusPosition int
}

// Camera interface for better testability
type Camera interface {
	// Power control
	PowerOn() error
	PowerOff() error

	// Movement control
	Stop() error
	PanLeft() error
	PanRight() error
	TiltUp() error
	TiltDown() error

	// Position control
	SetPanTiltPosition(pan, tilt int) error
	GetPanTiltPosition() (pan, tilt int, err error)
	SetZoomPosition(int) error
	GetZoomPosition() (int, error)

	// Camera control
	SetShootingMode(ShootingMode) error
	GetShootingMode() (ShootingMode, error)
	SetWhiteBalance(mode int) error
	GetWhiteBalance() (int, error)

	// Status
	GetStatus() (*CameraStatus, error)
	GetExtendedStatus() (*CameraStatus, error)
	GetSystemInfo() (*SystemInfo, error)

	// Shot memory
	RecordShotMemory(slot int) error
	MoveShotMemory(slot int, duration int) error

	Close() error
}

// XU81Camera represents the camera controller
type XU81Camera struct {
	port        string
	serial      SerialPort
	logger      *log.Logger
	isConnected bool
	mu          sync.Mutex
	config      *Config
}

// NewXU81Camera creates a new camera controller instance
func NewXU81Camera(config *Config, logger *log.Logger) *XU81Camera {
	if logger == nil {
		logger = log.New(os.Stdout, "[XU81] ", log.LstdFlags)
	}
	return &XU81Camera{
		port:   config.Port,
		logger: logger,
		config: config,
	}
}

// Commands map containing all available commands
var commands = map[string]Command{
	// Pan-Tilt Head Control Commands
	"pan_speed": {
		name: "Pan Speed Assignment", code: "0150", paramLength: 3,
		description: "Set panning speed (5-3000)", validator: validateSpeedParam,
	},
	"tilt_speed": {
		name: "Tilt Speed Assignment", code: "0151", paramLength: 3,
		description: "Set tilting speed (5-3000)", validator: validateSpeedParam,
	},
	"pan_speed_request": {
		name: "Pan Speed Request", code: "0152", paramLength: 1,
		description: "Get current pan speed", validator: validateSingleParam,
	},
	"tilt_speed_request": {
		name: "Tilt Speed Request", code: "0152", paramLength: 1,
		description: "Get current tilt speed", validator: validateSingleParam,
	},
	"pan_stop": {
		name: "Pan/Tilt Stop", code: "0153", paramLength: 1,
		description: "Stop pan/tilt movement", validator: validateSingleParam,
	},
	"pan_right": {
		name: "Pan Right Start", code: "0153", paramLength: 1,
		description: "Start panning right", validator: validateSingleParam,
	},
	"pan_left": {
		name: "Pan Left Start", code: "0153", paramLength: 1,
		description: "Start panning left", validator: validateSingleParam,
	},
	"tilt_up": {
		name: "Tilt Up Start", code: "0153", paramLength: 1,
		description: "Start tilting up", validator: validateSingleParam,
	},
	"tilt_down": {
		name: "Tilt Down Start", code: "0153", paramLength: 1,
		description: "Start tilting down", validator: validateSingleParam,
	},
	"initialize_home": {
		name: "Pan-Tilt Head Initialize 1", code: "0158", paramLength: 1,
		description: "Initialize and move to home position", validator: validateSingleParam,
	},
	"initialize_original": {
		name: "Pan-Tilt Head Initialize 2", code: "0158", paramLength: 1,
		description: "Initialize and move to original position", validator: validateSingleParam,
	},
	"pan_tilt_pos": {
		name: "Pan/Tilt Position Assignment", code: "0162", paramLength: 8,
		description: "Set absolute pan/tilt position", validator: validatePositionParam,
	},
	"pan_tilt_request": {
		name: "Pan/Tilt Angle Request", code: "0163", paramLength: 0,
		description: "Get current pan/tilt position", validator: nil,
	},

	"wiper_control": {
		name: "Wiper Control", code: "017A", paramLength: 1,
		description: "Operate wiper (one cycle)", validator: validateSingleParam,
	},
	"washer_control": {
		name: "Washer Control", code: "017A", paramLength: 1,
		description: "Auto-washer (4 wiper cycles, washer on first 2)", validator: validateSingleParam,
	},
	"washer_out": {
		name: "Washer Output On/Off", code: "0171", paramLength: 2,
		description: "Manually turn washer on or off", validator: validateWasherParam,
	},

	// Camera Control Commands
	"camera_version": {
		name: "Camera Version Request", code: "01BE", paramLength: 1,
		description: "Get camera version", validator: validateSingleParam,
	},
	"power_on": {
		name: "Camera Power ON", code: "01A0", paramLength: 1,
		description: "Turn on the camera power", validator: validateSingleParam,
	},
	"power_off": {
		name: "Camera Power OFF", code: "01A0", paramLength: 1,
		description: "Turn off the camera power", validator: validateSingleParam,
	},
	"focus_auto": {
		name: "Focus Auto", code: "01A1", paramLength: 1,
		description: "Enable auto focus", validator: validateSingleParam,
	},
	"focus_manual": {
		name: "Focus Manual", code: "01A1", paramLength: 1,
		description: "Set manual focus mode", validator: validateSingleParam,
	},
	"focus_infinity": {
		name: "Focus Fixed at Infinity", code: "01A1", paramLength: 1,
		description: "Fix focus at infinity", validator: validateSingleParam,
	},
	"focus_near": {
		name: "Focus Near", code: "01A1", paramLength: 1,
		description: "Move focus near", validator: validateSingleParam,
	},
	"focus_far": {
		name: "Focus Far", code: "01A1", paramLength: 1,
		description: "Move focus far", validator: validateSingleParam,
	},
	"focus_oneshot": {
		name: "One-shot AF", code: "01A1", paramLength: 1,
		description: "Perform one-shot autofocus", validator: validateSingleParam,
	},
	"zoom_stop": {
		name: "Zoom Stop", code: "01A2", paramLength: 1,
		description: "Stop zoom operation", validator: validateSingleParam,
	},
	"zoom_wide": {
		name: "Zoom Wide", code: "01A2", paramLength: 1,
		description: "Zoom to wide", validator: validateSingleParam,
	},
	"zoom_tele": {
		name: "Zoom Tele", code: "01A2", paramLength: 1,
		description: "Zoom to telephoto", validator: validateSingleParam,
	},
	"zoom_pos": {
		name: "Zoom Position", code: "01A3", paramLength: 2,
		description: "Set zoom position (00h-EEh)", validator: validateZoomParam,
	},
	"zoom_pos_request": {
		name: "Zoom Position Request", code: "01A4", paramLength: 0,
		description: "Get current zoom position", validator: nil,
	},
	"digital_zoom": {
		name: "Digital Zoom Magnification Assignment", code: "01DA", paramLength: 2,
		description: "Set digital zoom magnification (XU-81 only)", validator: validateDigitalZoomParam,
	},
	"shooting_mode": {
		name: "Shooting Mode Switching", code: "0184", paramLength: 1,
		description: "Set shooting mode", validator: validateShootingModeParam,
	},
	"shutter_speed": {
		name: "Shutter Speed Assignment", code: "01AD", paramLength: 1,
		description: "Set shutter speed", validator: validateShutterParam,
	},
	"gain_assignment": {
		name: "Gain Assignment", code: "01AE", paramLength: 1,
		description: "Set camera gain", validator: validateGainParam,
	},
	"iris_assignment": {
		name: "Iris Assignment", code: "01A6", paramLength: 2,
		description: "Set iris position", validator: validateIrisParam,
	},
	"white_balance": {
		name: "White Balance Control", code: "01A7", paramLength: 1,
		description: "Set white balance mode", validator: validateWhiteBalanceParam,
	},
	"wdr": {
		name: "WDR ON/OFF", code: "01A5", paramLength: 1,
		description: "Wide Dynamic Range on/off", validator: validateSingleParam,
	},
	"nd_filter": {
		name: "ND Filter ON/OFF", code: "01B6", paramLength: 1,
		description: "Neutral Density filter on/off", validator: validateSingleParam,
	},
	"ir_filter": {
		name: "IR Filter ON/OFF", code: "01BB", paramLength: 1,
		description: "Toggle IR filter", validator: validateSingleParam,
	},
	"auto_ir": {
		name: "Auto IR Control", code: "01BA", paramLength: 1,
		description: "Auto IR filter control", validator: validateAutoIRParam,
	},
	"color_bar": {
		name: "Color Bar Control", code: "01B8", paramLength: 1,
		description: "Color bar on/off", validator: validateSingleParam,
	},

	// System Control Commands
	"status": {
		name: "Operation Status", code: "0186", paramLength: 0,
		description: "Get camera operation status", validator: nil,
	},
	"extended_status": {
		name: "Extended Operation Status", code: "0186", paramLength: 1,
		description: "Get extended operation status", validator: validateSingleParam,
	},
	"product_name": {
		name: "Product Name Request", code: "0187", paramLength: 0,
		description: "Get product name", validator: nil,
	},
	"rom_version": {
		name: "ROM Version Request", code: "0188", paramLength: 0,
		description: "Get ROM version", validator: nil,
	},
	"shot_memory_record": {
		name: "Shot Memory Recording", code: "0182", paramLength: 2,
		description: "Record current position to shot memory", validator: validateShotMemoryParam,
	},
	"shot_memory_move": {
		name: "Shot Memory Movement", code: "0183", paramLength: 4,
		description: "Move to shot memory position", validator: validateShotMoveParam,
	},
	"serial_number": {
		name: "Serial Number Request", code: "01E9", paramLength: 1,
		description: "Get serial number", validator: validateSingleParam,
	},
}

// Validation functions
func validateSingleParam(params []byte, model CameraModel) error {
	if len(params) != 1 {
		return errors.New("command requires exactly 1 parameter byte")
	}
	return nil
}

func validateSpeedParam(params []byte, model CameraModel) error {
	if len(params) != 3 {
		return errors.New("speed command requires exactly 3 parameter bytes")
	}
	// Convert ASCII hex to integer for validation
	speedStr := string(params)
	speed, err := strconv.ParseInt(speedStr, 16, 32)
	if err != nil {
		return fmt.Errorf("invalid speed parameter: %w", err)
	}
	if speed < minSpeed || speed > maxSpeed {
		return fmt.Errorf("speed must be between %d and %d", minSpeed, maxSpeed)
	}
	return nil
}

func validatePositionParam(params []byte, model CameraModel) error {
	if len(params) != 8 {
		return errors.New("position command requires exactly 8 parameter bytes")
	}
	return nil
}

func validateZoomParam(params []byte, model CameraModel) error {
	if len(params) != 2 {
		return errors.New("zoom command requires exactly 2 parameter bytes")
	}
	return nil
}

func validateWasherParam(params []byte, model CameraModel) error {
	if len(params) != 2 {
		return errors.New("washer command requires exactly 2 parameter bytes")
	}
	return nil
}

func validateDigitalZoomParam(params []byte, model CameraModel) error {
	if model != ModelXU81 {
		return errors.New("digital zoom only available on XU-81")
	}
	if len(params) != 2 {
		return errors.New("digital zoom command requires exactly 2 parameter bytes")
	}
	return nil
}

func validateShootingModeParam(params []byte, model CameraModel) error {
	if len(params) != 1 {
		return errors.New("shooting mode command requires exactly 1 parameter byte")
	}
	mode := params[0]
	if mode < 0x30 || mode > 0x34 { // ASCII '0' to '4'
		return errors.New("invalid shooting mode parameter")
	}
	return nil
}

func validateShutterParam(params []byte, model CameraModel) error {
	if len(params) != 1 {
		return errors.New("shutter command requires exactly 1 parameter byte")
	}
	return nil
}

func validateGainParam(params []byte, model CameraModel) error {
	if len(params) != 1 {
		return errors.New("gain command requires exactly 1 parameter byte")
	}
	return nil
}

func validateIrisParam(params []byte, model CameraModel) error {
	if len(params) != 2 {
		return errors.New("iris command requires exactly 2 parameter bytes")
	}
	return nil
}

func validateWhiteBalanceParam(params []byte, model CameraModel) error {
	if len(params) != 1 {
		return errors.New("white balance command requires exactly 1 parameter byte")
	}
	return nil
}

func validateAutoIRParam(params []byte, model CameraModel) error {
	if len(params) != 1 {
		return errors.New("auto IR command requires exactly 1 parameter byte")
	}
	return nil
}

func validateShotMemoryParam(params []byte, model CameraModel) error {
	if len(params) != 2 {
		return errors.New("shot memory command requires exactly 2 parameter bytes")
	}
	return nil
}

func validateShotMoveParam(params []byte, model CameraModel) error {
	if len(params) != 4 {
		return errors.New("shot move command requires exactly 4 parameter bytes")
	}
	return nil
}

// Helper functions
func intToASCIIHex(value int, digits int) []byte {
	hexStr := fmt.Sprintf("%0*X", digits, value)
	return []byte(hexStr)
}

func asciiHexToInt(data []byte) (int, error) {
	hexStr := string(data)
	val, err := strconv.ParseInt(hexStr, 16, 32)
	return int(val), err
}

func signedIntToHex(value int) []byte {
	// Convert signed integer to unsigned 16-bit representation
	unsigned := uint16(value + 0x8000)
	hexStr := fmt.Sprintf("%04X", unsigned)
	return []byte(hexStr)
}

func hexToSignedInt(data []byte) int {
	if len(data) != 4 {
		return 0
	}
	hexStr := string(data)
	unsigned, _ := strconv.ParseUint(hexStr, 16, 16)
	return int(unsigned) - 0x8000
}

// ensureConnected ensures the serial port is connected
func (cam *XU81Camera) ensureConnected() error {
	cam.mu.Lock()
	defer cam.mu.Unlock()

	if !cam.isConnected || cam.serial == nil {
		return cam.openPort()
	}
	return nil
}

// openPort opens the serial connection to the camera
func (cam *XU81Camera) openPort() error {
	mode := &serial.Mode{
		BaudRate: cam.config.BaudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(cam.port, mode)
	if err != nil {
		return fmt.Errorf("failed to open serial port: %w", err)
	}

	if err := port.SetReadTimeout(cam.config.ReadTimeout); err != nil {
		port.Close()
		return fmt.Errorf("failed to set read timeout: %w", err)
	}

	cam.serial = port
	cam.isConnected = true
	cam.logger.Printf("Connected to %s", cam.port)
	return nil
}

// Close closes the serial connection
func (cam *XU81Camera) Close() error {
	cam.mu.Lock()
	defer cam.mu.Unlock()

	if cam.serial != nil {
		err := cam.serial.Close()
		cam.isConnected = false
		cam.serial = nil
		return err
	}
	return nil
}

// formatCommandBytes formats a command into bytes for transmission
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

// parseResponse parses the raw response bytes
// Update your parseResponse function:
func (cam *XU81Camera) parseResponse(data []byte) Response {
	if len(data) == 0 {
		return Response{Type: ResponseError, Error: errors.New("empty response")}
	}

	switch data[0] {
	case ackByte:
		return Response{Type: ResponseACK, Data: data}
	case nakByte:
		return Response{Type: ResponseNAK, Error: errors.New("NAK received")}
	case responseHeader:
		if len(data) >= 6 && data[len(data)-1] == endMarkByte {
			// Check error code (bytes 3-4 are ASCII hex error code)
			errorCode := string(data[3:5])
			if errorCode != "00" { // "00" = ASCII "00" = no error
				return Response{
					Type:      ResponseError,
					ErrorCode: errorCode,
					Error:     parseErrorCode(errorCode),
					Data:      data, // Include the full data even on error
				}
			}
			// Success case - include the full response data
			return Response{
				Type: ResponseStatus,
				Data: data, // This was missing!
			}
		}
	}

	return Response{Type: ResponseError, Error: fmt.Errorf("unknown response format: %X", data)}
}

// readResponse reads and parses the camera response
func (cam *XU81Camera) readResponse() (Response, error) {
	var buf []byte
	deadline := time.Now().Add(responseTimeout)

	for time.Now().Before(deadline) {
		tmp := make([]byte, 32)
		n, err := cam.serial.Read(tmp)
		if err != nil && !errors.Is(err, io.EOF) {
			return Response{}, fmt.Errorf("read error: %w", err)
		}
		if n == 0 {
			continue
		}
		buf = append(buf, tmp[:n]...)

		// Check for complete response
		if len(buf) > 0 && (buf[0] == ackByte || buf[0] == nakByte) {
			return cam.parseResponse(buf[:1]), nil
		}
		if len(buf) >= 6 && buf[0] == responseHeader && buf[len(buf)-1] == endMarkByte {
			return cam.parseResponse(buf), nil
		}
	}

	return Response{}, errors.New("timeout waiting for response")
}

// parseErrorCode interprets the camera's ASCII-hex error flags
func parseErrorCode(code string) error {
	if code == "3030" { // ASCII "00"
		return nil
	}

	// Parse ASCII hex error code
	var errs []string

	if len(code) >= 2 {
		// First byte (MSB)
		firstChar := code[0]
		if firstChar >= '8' || (firstChar >= '4' && firstChar <= '7') {
			errs = append(errs, "Mode Error")
		}
		if firstChar == '5' || firstChar == '7' || firstChar == 'D' || firstChar == 'F' {
			errs = append(errs, "Parameter Error")
		}
		if firstChar == '3' || firstChar == '7' || firstChar == 'B' || firstChar == 'F' {
			errs = append(errs, "Command Error")
		}
		if firstChar == '1' || firstChar == '3' || firstChar == '5' || firstChar == '7' ||
			firstChar == '9' || firstChar == 'B' || firstChar == 'D' || firstChar == 'F' {
			errs = append(errs, "Busy")
		}
	}

	if len(code) >= 2 {
		// Second byte (LSB)
		secondChar := code[1]
		if secondChar == '1' || secondChar == '3' || secondChar == '5' || secondChar == '7' ||
			secondChar == '9' || secondChar == 'B' || secondChar == 'D' || secondChar == 'F' {
			errs = append(errs, "System Error")
		}
	}

	if len(errs) == 0 {
		return fmt.Errorf("unknown error code: %s", code)
	}
	return fmt.Errorf("camera errors: %s", strings.Join(errs, ", "))
}

// sendCommand sends a command to the camera and returns the response
func (cam *XU81Camera) sendCommand(cmd Command, params []byte) (Response, error) {
	if err := cam.ensureConnected(); err != nil {
		return Response{}, fmt.Errorf("connection error: %w", err)
	}

	if cmd.validator != nil {
		if err := cmd.validator(params, cam.config.CameraModel); err != nil {
			return Response{}, fmt.Errorf("parameter validation failed: %w", err)
		}
	}

	cmdBytes := formatCommandBytes(cmd, params)

	cam.logger.Printf("Sending command %s: %X", cmd.name, cmdBytes)
	if _, err := cam.serial.Write(cmdBytes); err != nil {
		return Response{}, fmt.Errorf("failed to send command: %w", err)
	}

	resp, err := cam.readResponse()
	if err != nil {
		return Response{}, fmt.Errorf("command %s failed: %w", cmd.name, err)
	}

	return resp, nil
}

// Power control methods
func (cam *XU81Camera) PowerOn() error {
	c := commands["power_on"]
	resp, err := cam.sendCommand(c, []byte{paramOn})
	if err != nil {
		return fmt.Errorf("power on failed: %w", err)
	}
	if resp.Type == ResponseACK {
		cam.logger.Println("Camera powering on, waiting for boot...")
		time.Sleep(cam.config.PowerOnDelay)
	}
	return nil
}

func (cam *XU81Camera) PowerOff() error {
	c := commands["power_off"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

// Movement control methods
func (cam *XU81Camera) Stop() error {
	c := commands["pan_stop"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

func (cam *XU81Camera) PanRight() error {
	c := commands["pan_right"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) PanLeft() error {
	c := commands["pan_left"]
	_, err := cam.sendCommand(c, []byte{0x32}) // ASCII '2'
	return err
}

func (cam *XU81Camera) TiltUp() error {
	c := commands["tilt_up"]
	_, err := cam.sendCommand(c, []byte{0x33}) // ASCII '3'
	return err
}

func (cam *XU81Camera) TiltDown() error {
	c := commands["tilt_down"]
	_, err := cam.sendCommand(c, []byte{0x34}) // ASCII '4'
	return err
}

// Position control methods
func (cam *XU81Camera) SetPanTiltPosition(pan, tilt int) error {
	if pan < minPanAngle || pan > maxPanAngle {
		return fmt.Errorf("pan angle must be between %d and %d", minPanAngle, maxPanAngle)
	}
	if tilt < minTiltAngle || tilt > maxTiltAngle {
		return fmt.Errorf("tilt angle must be between %d and %d", minTiltAngle, maxTiltAngle)
	}

	c := commands["pan_tilt_pos"]
	panHex := signedIntToHex(pan)
	tiltHex := signedIntToHex(tilt)

	params := append(panHex, tiltHex...)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) GetPanTiltPosition() (pan, tilt int, err error) {
	c := commands["pan_tilt_request"]
	resp, err := cam.sendCommand(c, nil)
	if err != nil {
		return 0, 0, err
	}

	if resp.Type != ResponseStatus || len(resp.Data) < 13 {
		return 0, 0, errors.New("invalid response format")
	}

	// Extract position data (bytes 5-12, ASCII hex)
	panData := resp.Data[5:9]
	tiltData := resp.Data[9:13]

	pan = hexToSignedInt(panData)
	tilt = hexToSignedInt(tiltData)

	return pan, tilt, nil
}

func (cam *XU81Camera) SetZoomPosition(pos int) error {
	if pos < 0 || pos > maxZoom {
		return fmt.Errorf("zoom position must be between 0 and %d", maxZoom)
	}
	c := commands["zoom_pos"]
	params := intToASCIIHex(pos, 2)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) GetZoomPosition() (int, error) {
	c := commands["zoom_pos_request"]
	resp, err := cam.sendCommand(c, nil)
	if err != nil {
		return 0, err
	}

	if resp.Type != ResponseStatus || len(resp.Data) < 7 {
		return 0, errors.New("invalid response format")
	}

	// Extract zoom position (bytes 5-6, ASCII hex)
	pos, err := asciiHexToInt(resp.Data[5:7])
	return pos, err
}

// Camera control methods
func (cam *XU81Camera) SetShootingMode(mode ShootingMode) error {
	c := commands["shooting_mode"]
	param := byte(0x30 + int(mode)) // Convert to ASCII
	_, err := cam.sendCommand(c, []byte{param})
	return err
}

func (cam *XU81Camera) GetShootingMode() (ShootingMode, error) {
	// This would require implementing the status request command for shooting mode
	// For now, return a placeholder
	return ShootingAuto, errors.New("not implemented")
}

func (cam *XU81Camera) SetWhiteBalance(mode int) error {
	if mode < 0 || mode > 4 {
		return errors.New("white balance mode must be 0-4")
	}
	c := commands["white_balance"]
	param := byte(0x30 + mode) // Convert to ASCII
	_, err := cam.sendCommand(c, []byte{param})
	return err
}

func (cam *XU81Camera) GetWhiteBalance() (int, error) {
	// This would require implementing the status request command
	return 0, errors.New("not implemented")
}

// Status methods
func (cam *XU81Camera) GetStatus() (*CameraStatus, error) {
	c := commands["status"]
	resp, err := cam.sendCommand(c, nil)
	if err != nil {
		return nil, err
	}

	// The response format is:
	// FE (header) + 3030 (device) + 3030 (error) + 303030 (3-byte status) + EF (end)
	// Total: 9 bytes
	if resp.Type != ResponseStatus || len(resp.Data) < 9 {
		return nil, fmt.Errorf("invalid status response: got %d bytes, data: %X", len(resp.Data), resp.Data)
	}

	// Parse 3-byte status (bytes 5-7, ASCII hex)
	// Note: indices 5,6,7 in the full response (0-based)
	statusData := resp.Data[5:8]
	statusValue, err := asciiHexToInt(statusData)
	if err != nil {
		return nil, fmt.Errorf("failed to parse status: %w", err)
	}

	status := &CameraStatus{
		Raw:               resp.Data,
		Focusing:          (statusValue & 0x001) != 0, // b0
		ManualFocus:       (statusValue & 0x002) != 0, // b1
		PowerOn:           (statusValue & 0x020) == 0, // b5 inverted (Camera Power OFF flag)
		Zooming:           (statusValue & 0x080) != 0, // b7
		PanLimitPosition:  (statusValue & 0x100) != 0, // b8
		Panning:           (statusValue & 0x200) != 0, // b9
		TiltLimitPosition: (statusValue & 0x400) != 0, // b10
		Tilting:           (statusValue & 0x800) != 0, // b11
	}

	return status, nil
}
func (cam *XU81Camera) GetExtendedStatus() (*CameraStatus, error) {
	c := commands["extended_status"]
	resp, err := cam.sendCommand(c, []byte{paramOff})
	if err != nil {
		return nil, err
	}

	if resp.Type != ResponseStatus || len(resp.Data) < 10 {
		return nil, errors.New("invalid extended status response")
	}

	// Parse 5-byte extended status (bytes 5-9, ASCII hex)
	statusData := resp.Data[5:10]
	statusValue, err := asciiHexToInt(statusData)
	if err != nil {
		return nil, fmt.Errorf("failed to parse extended status: %w", err)
	}

	status := &CameraStatus{
		Raw:                 resp.Data,
		Focusing:            (statusValue & 0x00001) != 0, // b0
		ManualFocus:         (statusValue & 0x00002) != 0, // b1
		PowerOn:             (statusValue & 0x00020) == 0, // b5 inverted
		PanLimitPosition:    (statusValue & 0x00100) != 0, // b8
		Panning:             (statusValue & 0x00200) != 0, // b9
		TiltLimitPosition:   (statusValue & 0x00400) != 0, // b10
		Tilting:             (statusValue & 0x00800) != 0, // b11
		Zooming:             (statusValue & 0x00080) != 0, // b7
		WhiteBalanceNotAuto: (statusValue & 0x01000) != 0, // b12
		CameraManualMode:    (statusValue & 0x02000) != 0, // b13
		CameraBusy:          (statusValue & 0x04000) != 0, // b14
		PanMotorError:       (statusValue & 0x10000) != 0, // b16
		TiltMotorError:      (statusValue & 0x20000) != 0, // b17
	}

	return status, nil
}

func (cam *XU81Camera) GetSystemInfo() (*SystemInfo, error) {
	info := &SystemInfo{}

	// Get product name
	if c, ok := commands["product_name"]; ok {
		resp, err := cam.sendCommand(c, nil)
		if err == nil && resp.Type == ResponseStatus && len(resp.Data) > 5 {
			// Extract product name from response
			nameData := resp.Data[5 : len(resp.Data)-1] // Exclude end mark
			info.ProductName = string(nameData)
		}
	}

	// Get ROM version
	if c, ok := commands["rom_version"]; ok {
		resp, err := cam.sendCommand(c, nil)
		if err == nil && resp.Type == ResponseStatus && len(resp.Data) > 5 {
			// Extract ROM version from response
			versionData := resp.Data[5 : len(resp.Data)-1] // Exclude end mark
			info.ROMVersion = string(versionData)
		}
	}

	// Get camera version
	if c, ok := commands["camera_version"]; ok {
		resp, err := cam.sendCommand(c, []byte{paramOff})
		if err == nil && resp.Type == ResponseStatus && len(resp.Data) >= 7 {
			// Extract camera version (2-byte ASCII hex)
			versionData := resp.Data[5:7]
			if version, err := asciiHexToInt(versionData); err == nil {
				info.CameraVersion = version
			}
		}
	}

	return info, nil
}

// Shot memory methods
func (cam *XU81Camera) RecordShotMemory(slot int) error {
	if slot < 0 || slot > 127 {
		return errors.New("shot memory slot must be 0-127")
	}
	c := commands["shot_memory_record"]
	params := intToASCIIHex(slot, 2)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) MoveShotMemory(slot int, duration int) error {
	if slot < 0 || slot > 127 {
		return errors.New("shot memory slot must be 0-127")
	}
	if duration < 0 || duration > 48 {
		return errors.New("duration must be 0-48 seconds")
	}

	c := commands["shot_memory_move"]
	slotParams := intToASCIIHex(slot, 2)
	durationParams := intToASCIIHex(duration, 2)
	params := append(slotParams, durationParams...)
	_, err := cam.sendCommand(c, params)
	return err
}

// Additional camera control methods
func (cam *XU81Camera) InitializeHome() error {
	c := commands["initialize_home"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	if err == nil {
		cam.logger.Println("Camera initializing to home position...")
		time.Sleep(10 * time.Second) // Wait for initialization
	}
	return err
}

func (cam *XU81Camera) InitializeOriginal() error {
	c := commands["initialize_original"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	if err == nil {
		cam.logger.Println("Camera initializing to original position...")
		time.Sleep(10 * time.Second) // Wait for initialization
	}
	return err
}

func (cam *XU81Camera) SetPanSpeed(speed int) error {
	if speed < minSpeed || speed > maxSpeed {
		return fmt.Errorf("speed must be between %d and %d", minSpeed, maxSpeed)
	}
	c := commands["pan_speed"]
	params := intToASCIIHex(speed, 3)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) SetTiltSpeed(speed int) error {
	if speed < minSpeed || speed > maxSpeed {
		return fmt.Errorf("speed must be between %d and %d", minSpeed, maxSpeed)
	}
	c := commands["tilt_speed"]
	params := intToASCIIHex(speed, 3)
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) GetPanSpeed() (int, error) {
	c := commands["pan_speed_request"]
	resp, err := cam.sendCommand(c, []byte{paramOff})
	if err != nil {
		return 0, err
	}

	if resp.Type != ResponseStatus || len(resp.Data) < 8 {
		return 0, errors.New("invalid response format")
	}

	pos, err := asciiHexToInt(resp.Data[5:8])
	return pos, err
}

func (cam *XU81Camera) GetTiltSpeed() (int, error) {
	c := commands["tilt_speed_request"]
	resp, err := cam.sendCommand(c, []byte{paramOn})
	if err != nil {
		return 0, err
	}

	if resp.Type != ResponseStatus || len(resp.Data) < 8 {
		return 0, errors.New("invalid response format")
	}

	speed, err := asciiHexToInt(resp.Data[5:8])
	return speed, err
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

// Filter and enhancement controls
func (cam *XU81Camera) WDROn() error {
	c := commands["wdr"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) WDROff() error {
	c := commands["wdr"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

func (cam *XU81Camera) NDFilterOn() error {
	c := commands["nd_filter"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) NDFilterOff() error {
	c := commands["nd_filter"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

func (cam *XU81Camera) IRFilterOn() error {
	c := commands["ir_filter"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) IRFilterOff() error {
	c := commands["ir_filter"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

func (cam *XU81Camera) ColorBarOn() error {
	c := commands["color_bar"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) ColorBarOff() error {
	c := commands["color_bar"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

// Focus controls
func (cam *XU81Camera) FocusAuto() error {
	c := commands["focus_auto"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

func (cam *XU81Camera) FocusManual() error {
	c := commands["focus_manual"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) FocusInfinity() error {
	c := commands["focus_infinity"]
	_, err := cam.sendCommand(c, []byte{0x42}) // ASCII 'B'
	return err
}

func (cam *XU81Camera) FocusNear() error {
	c := commands["focus_near"]
	_, err := cam.sendCommand(c, []byte{0x32}) // ASCII '2'
	return err
}

func (cam *XU81Camera) FocusFar() error {
	c := commands["focus_far"]
	_, err := cam.sendCommand(c, []byte{0x33}) // ASCII '3'
	return err
}

func (cam *XU81Camera) FocusOneShot() error {
	c := commands["focus_oneshot"]
	_, err := cam.sendCommand(c, []byte{0x41}) // ASCII 'A'
	return err
}

// Zoom controls
func (cam *XU81Camera) ZoomStop() error {
	c := commands["zoom_stop"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

func (cam *XU81Camera) ZoomWide() error {
	c := commands["zoom_wide"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) ZoomTele() error {
	c := commands["zoom_tele"]
	_, err := cam.sendCommand(c, []byte{0x32}) // ASCII '2'
	return err
}

// Washer controls
func (cam *XU81Camera) WasherControl() error {
	c := commands["washer_control"]
	_, err := cam.sendCommand(c, []byte{paramOn})
	return err
}

func (cam *XU81Camera) WasherOutputOn() error {
	c := commands["washer_out"]
	params := []byte{0x42, 0x31} // ASCII "B1"
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) WasherOutputOff() error {
	c := commands["washer_out"]
	params := []byte{0x42, 0x30} // ASCII "B0"
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) WiperControl() error {
	c := commands["wiper_control"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

// CLI command structure
type CLICommand struct {
	Name        string
	Usage       string
	MinArgs     int
	MaxArgs     int
	Handler     func(cam *XU81Camera, args []string) error
	Description string
}

// Enhanced CLI commands
var cliCommands = map[string]CLICommand{
	// Power commands
	"on": {
		Name: "on", Usage: "on", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.PowerOn() },
		Description: "Power on camera",
	},
	"off": {
		Name: "off", Usage: "off", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.PowerOff() },
		Description: "Power off camera",
	},
	"debug": {
		Name: "debug", Usage: "debug <command> [hex_params...]", MinArgs: 1, MaxArgs: 10,
		Handler: func(cam *XU81Camera, args []string) error {
			cmdName := args[0]

			var params []byte
			for i := 1; i < len(args); i++ {
				val, err := strconv.ParseUint(args[i], 16, 8)
				if err != nil {
					return fmt.Errorf("invalid hex parameter: %s", args[i])
				}
				params = append(params, byte(val))
			}

			return cam.DebugRawCommand(cmdName, params)
		},
		Description: "Debug raw command/response",
	},

	// Movement commands
	"stop": {
		Name: "stop", Usage: "stop", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.Stop() },
		Description: "Stop all movement",
	},
	"left": {
		Name: "left", Usage: "left", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.PanLeft() },
		Description: "Start panning left",
	},
	"right": {
		Name: "right", Usage: "right", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.PanRight() },
		Description: "Start panning right",
	},
	"up": {
		Name: "up", Usage: "up", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.TiltUp() },
		Description: "Start tilting up",
	},
	"down": {
		Name: "down", Usage: "down", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.TiltDown() },
		Description: "Start tilting down",
	},

	// Position commands
	"goto": {
		Name: "goto", Usage: "goto <pan> <tilt>", MinArgs: 2, MaxArgs: 2,
		Handler: func(cam *XU81Camera, args []string) error {
			pan, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid pan angle: %w", err)
			}
			tilt, err := strconv.Atoi(args[1])
			if err != nil {
				return fmt.Errorf("invalid tilt angle: %w", err)
			}
			return cam.SetPanTiltPosition(pan, tilt)
		},
		Description: "Move to pan/tilt position",
	},
	"getpos": {
		Name: "getpos", Usage: "getpos", MinArgs: 0, MaxArgs: 0,
		Handler: func(cam *XU81Camera, args []string) error {
			pan, tilt, err := cam.GetPanTiltPosition()
			if err != nil {
				return err
			}
			fmt.Printf("Position: Pan=%d, Tilt=%d\n", pan, tilt)
			return nil
		},
		Description: "Get current pan/tilt position",
	},

	// Zoom commands
	"zoom": {
		Name: "zoom", Usage: "zoom <position>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			pos, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid zoom position: %w", err)
			}
			return cam.SetZoomPosition(pos)
		},
		Description: "Set zoom position (0-238)",
	},
	"zoomwide": {
		Name: "zoomwide", Usage: "zoomwide", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.ZoomWide() },
		Description: "Start zooming to wide",
	},
	"zoomtele": {
		Name: "zoomtele", Usage: "zoomtele", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.ZoomTele() },
		Description: "Start zooming to telephoto",
	},
	"zoomstop": {
		Name: "zoomstop", Usage: "zoomstop", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.ZoomStop() },
		Description: "Stop zoom movement",
	},

	// Speed commands
	"panspeed": {
		Name: "panspeed", Usage: "panspeed <speed>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			speed, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid speed value: %w", err)
			}
			return cam.SetPanSpeed(speed)
		},
		Description: "Set pan speed (5-3000)",
	},
	"tiltspeed": {
		Name: "tiltspeed", Usage: "tiltspeed <speed>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			speed, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid speed value: %w", err)
			}
			return cam.SetTiltSpeed(speed)
		},
		Description: "Set tilt speed (5-3000)",
	},
	"debugfocus": {
		Name: "debugfocus", Usage: "debugfocus", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.DebugFocusControl() },
		Description: "Debug focus control issues",
	},
	// Add this debug command to your cliCommands map:
	"rawcmd": {
		Name: "rawcmd", Usage: "rawcmd <command_name> [params...]", MinArgs: 1, MaxArgs: 10,
		Handler: func(cam *XU81Camera, args []string) error {
			cmdName := args[0]
			cmd, ok := commands[cmdName]
			if !ok {
				return fmt.Errorf("unknown command: %s", cmdName)
			}

			var params []byte
			if len(args) > 1 {
				// Parse params as hex bytes
				for i := 1; i < len(args); i++ {
					val, err := strconv.ParseUint(args[i], 16, 8)
					if err != nil {
						return fmt.Errorf("invalid hex param: %s", args[i])
					}
					params = append(params, byte(val))
				}
			}

			// Send command with extra debugging
			if err := cam.ensureConnected(); err != nil {
				return err
			}

			cmdBytes := formatCommandBytes(cmd, params)
			fmt.Printf("Sending: %X\n", cmdBytes)

			if _, err := cam.serial.Write(cmdBytes); err != nil {
				return fmt.Errorf("write error: %w", err)
			}

			// Read raw response
			buf := make([]byte, 256)
			n, err := cam.serial.Read(buf)
			if err != nil && err != io.EOF {
				return fmt.Errorf("read error: %w", err)
			}

			fmt.Printf("Received %d bytes: %X\n", n, buf[:n])
			fmt.Printf("As string: %q\n", buf[:n])

			// Try to parse it
			resp := cam.parseResponse(buf[:n])
			fmt.Printf("Parsed: Type=%v, Error=%v, ErrorCode=%s\n",
				resp.Type, resp.Error, resp.ErrorCode)

			return nil
		},
		Description: "Send raw command and show response",
	},

	// Focus commands
	"focusauto": {
		Name: "focusauto", Usage: "focusauto", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.FocusAuto() },
		Description: "Enable auto focus",
	},
	"focusmanual": {
		Name: "focusmanual", Usage: "focusmanual", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.FocusManual() },
		Description: "Set manual focus mode",
	},
	"focusnear": {
		Name: "focusnear", Usage: "focusnear", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.FocusNear() },
		Description: "Move focus near",
	},
	"focusfar": {
		Name: "focusfar", Usage: "focusfar", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.FocusFar() },
		Description: "Move focus far",
	},
	"focusinf": {
		Name: "focusinf", Usage: "focusinf", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.FocusInfinity() },
		Description: "Set focus to infinity",
	},
	"focusoneshot": {
		Name: "focusoneshot", Usage: "focusoneshot", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.FocusOneShot() },
		Description: "Perform one-shot autofocus",
	},

	// Filter commands
	"wdr": {
		Name: "wdr", Usage: "wdr <on|off>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			switch args[0] {
			case "on":
				return cam.WDROn()
			case "off":
				return cam.WDROff()
			default:
				return fmt.Errorf("invalid argument: use 'on' or 'off'")
			}
		},
		Description: "WDR ON/OFF",
	},
	"nd": {
		Name: "nd", Usage: "nd <on|off>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			switch args[0] {
			case "on":
				return cam.NDFilterOn()
			case "off":
				return cam.NDFilterOff()
			default:
				return fmt.Errorf("invalid argument: use 'on' or 'off'")
			}
		},
		Description: "ND filter ON/OFF",
	},
	"ir": {
		Name: "ir", Usage: "ir <on|off>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			switch args[0] {
			case "on":
				return cam.IRFilterOn()
			case "off":
				return cam.IRFilterOff()
			default:
				return fmt.Errorf("invalid argument: use 'on' or 'off'")
			}
		},
		Description: "IR Filter ON/OFF",
	},
	"colorbar": {
		Name: "colorbar", Usage: "colorbar <on|off>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			switch args[0] {
			case "on":
				return cam.ColorBarOn()
			case "off":
				return cam.ColorBarOff()
			default:
				return fmt.Errorf("invalid argument: use 'on' or 'off'")
			}
		},
		Description: "Color bar ON/OFF",
	},

	// Status commands
	"status": {
		Name: "status", Usage: "status", MinArgs: 0, MaxArgs: 0,
		Handler: func(cam *XU81Camera, args []string) error {
			status, err := cam.GetStatus()
			if err != nil {
				return err
			}
			fmt.Printf("Status:\n")
			fmt.Printf("  Power: %v\n", status.PowerOn)
			fmt.Printf("  Panning: %v\n", status.Panning)
			fmt.Printf("  Tilting: %v\n", status.Tilting)
			fmt.Printf("  Zooming: %v\n", status.Zooming)
			fmt.Printf("  Focusing: %v\n", status.Focusing)
			fmt.Printf("  Manual Focus: %v\n", status.ManualFocus)
			fmt.Printf("  Raw: %X\n", status.Raw)
			return nil
		},
		Description: "Get camera status",
	},
	"extstatus": {
		Name: "extstatus", Usage: "extstatus", MinArgs: 0, MaxArgs: 0,
		Handler: func(cam *XU81Camera, args []string) error {
			status, err := cam.GetExtendedStatus()
			if err != nil {
				return err
			}
			fmt.Printf("Extended Status:\n")
			fmt.Printf("  Power: %v\n", status.PowerOn)
			fmt.Printf("  Camera Busy: %v\n", status.CameraBusy)
			fmt.Printf("  Camera Manual Mode: %v\n", status.CameraManualMode)
			fmt.Printf("  WB Not Auto: %v\n", status.WhiteBalanceNotAuto)
			fmt.Printf("  Pan Motor Error: %v\n", status.PanMotorError)
			fmt.Printf("  Tilt Motor Error: %v\n", status.TiltMotorError)
			fmt.Printf("  Raw: %X\n", status.Raw)
			return nil
		},
		Description: "Get extended camera status",
	},
	"info": {
		Name: "info", Usage: "info", MinArgs: 0, MaxArgs: 0,
		Handler: func(cam *XU81Camera, args []string) error {
			info, err := cam.GetSystemInfo()
			if err != nil {
				return err
			}
			fmt.Printf("System Information:\n")
			fmt.Printf("  Product: %s\n", info.ProductName)
			fmt.Printf("  ROM Version: %s\n", info.ROMVersion)
			fmt.Printf("  Camera Version: %d\n", info.CameraVersion)
			fmt.Printf("  Serial Number: %s\n", info.SerialNumber)
			return nil
		},
		Description: "Get system information",
	},

	// Shot memory commands
	"record": {
		Name: "record", Usage: "record <slot>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			slot, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid slot number: %w", err)
			}
			return cam.RecordShotMemory(slot)
		},
		Description: "Record current position to shot memory slot (0-127)",
	},
	"recall": {
		Name: "recall", Usage: "recall <slot> [duration]", MinArgs: 1, MaxArgs: 2,
		Handler: func(cam *XU81Camera, args []string) error {
			slot, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid slot number: %w", err)
			}
			duration := 5 // Default duration
			if len(args) > 1 {
				duration, err = strconv.Atoi(args[1])
				if err != nil {
					return fmt.Errorf("invalid duration: %w", err)
				}
			}
			return cam.MoveShotMemory(slot, duration)
		},
		Description: "Move to shot memory position with optional duration",
	},

	// Initialization commands
	"inithome": {
		Name: "inithome", Usage: "inithome", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.InitializeHome() },
		Description: "Initialize camera to home position",
	},
	"initoriginal": {
		Name: "initoriginal", Usage: "initoriginal", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.InitializeOriginal() },
		Description: "Initialize camera to original position",
	},

	// Washer/Wiper commands
	"washer": {
		Name: "washer", Usage: "washer", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.WasherControl() },
		Description: "Run 4-cycle washer sequence",
	},
	"washeron": {
		Name: "washeron", Usage: "washeron", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.WasherOutputOn() },
		Description: "Turn washer output ON manually",
	},
	"washeroff": {
		Name: "washeroff", Usage: "washeroff", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.WasherOutputOff() },
		Description: "Turn washer output OFF manually",
	},
	"wiper": {
		Name: "wiper", Usage: "wiper", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.WiperControl() },
		Description: "Operate wiper (one cycle)",
	},

	// White balance commands
	"wbauto": {
		Name: "wbauto", Usage: "wbauto", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetWhiteBalance(0) },
		Description: "Set white balance to auto",
	},
	"wbpreset": {
		Name: "wbpreset", Usage: "wbpreset", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetWhiteBalance(1) },
		Description: "Set white balance to preset",
	},
	"wbindoor": {
		Name: "wbindoor", Usage: "wbindoor", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetWhiteBalance(2) },
		Description: "Set white balance to indoor (3200K)",
	},
	"wboutdoor": {
		Name: "wboutdoor", Usage: "wboutdoor", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetWhiteBalance(3) },
		Description: "Set white balance to outdoor (5800K)",
	},
	"wbatw": {
		Name: "wbatw", Usage: "wbatw", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetWhiteBalance(4) },
		Description: "Set white balance to ATW",
	},

	// Shooting mode commands
	"showauto": {
		Name: "showauto", Usage: "showauto", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetShootingMode(ShootingAuto) },
		Description: "Set shooting mode to auto",
	},
	"showmanual": {
		Name: "showmanual", Usage: "showmanual", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetShootingMode(ShootingManual) },
		Description: "Set shooting mode to manual",
	},
	"shownight": {
		Name: "shownight", Usage: "shownight", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetShootingMode(ShootingNight) },
		Description: "Set shooting mode to night",
	},
}

func main() {
	// ---- 1. global flags ----
	cfgFile := flag.String("config", "", "Path to JSON config")
	verbose := flag.Bool("v", false, "Verbose logging")
	flag.Parse()

	// ---- 2. config & logger ----
	cfg, err := LoadConfig(*cfgFile)
	if err != nil {
		fmt.Fprintf(os.Stderr, "config error: %v\n", err)
		os.Exit(1)
	}
	logger := log.New(os.Stdout, "[XU81] ", log.LstdFlags)
	if *verbose {
		logger.SetFlags(log.LstdFlags | log.Lshortfile)
	}

	// ---- 3. camera ----
	cam := NewXU81Camera(cfg, logger)
	defer cam.Close()

	// ---- 4. CLI args or interactive ----
	if flag.NArg() > 0 {
		runCommand(cam, flag.Arg(0), flag.Args()[1:])
		return
	}

	// ---------- INTERACTIVE SHELL ----------
	shell := liner.NewLiner()
	defer shell.Close()

	shell.SetCtrlCAborts(true) // ^C cancels current line
	shell.SetCompleter(func(line string) (c []string) {
		for name := range cliCommands {
			if strings.HasPrefix(name, strings.ToLower(line)) {
				c = append(c, name)
			}
		}
		return
	})

	const historyFile = ".mycamera_history"
	if f, err := os.Open(historyFile); err == nil {
		shell.ReadHistory(f)
		f.Close()
	}

	fmt.Println("Interactive mode — type \"help\" for commands, Ctrl-D to quit.")
	for {
		input, err := shell.Prompt("> ")
		if err == liner.ErrPromptAborted || err == io.EOF {
			fmt.Println()
			break // ^C or Ctrl-D
		}
		input = strings.TrimSpace(input)
		if input == "" {
			continue
		}
		shell.AppendHistory(input)

		if input == "help" {
			for _, c := range cliCommands {
				fmt.Printf("  %-10s %s\n", c.Name, c.Description)
			}
			continue
		}
		if input == "quit" || input == "exit" {
			break
		}

		tokens := strings.Fields(input)
		runCommand(cam, tokens[0], tokens[1:])
	}

	if f, err := os.Create(historyFile); err == nil {
		shell.WriteHistory(f)
		f.Close()
	}
}

func runCommand(cam *XU81Camera, name string, args []string) {
	cmd, ok := cliCommands[name]
	if !ok {
		fmt.Printf("unknown command %q\n", name)
		return
	}
	if len(args) < cmd.MinArgs || len(args) > cmd.MaxArgs {
		fmt.Printf("usage: %s\n", cmd.Usage)
		return
	}
	if err := cmd.Handler(cam, args); err != nil {
		fmt.Printf("error: %v\n", err)
	}
}
