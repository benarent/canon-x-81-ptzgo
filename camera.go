package main

import (
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"os"
	"strconv"
	"sync"
	"time"

	"go.bug.st/serial"
)

// Camera model types
type CameraModel int

const (
	ModelXU80 CameraModel = iota
	ModelXU81
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

// Config holds the camera configuration
type Config struct {
	Port         string        `json:"port"`
	BaudRate     int           `json:"baud_rate"`
	ReadTimeout  time.Duration `json:"read_timeout"`
	PowerOnDelay time.Duration `json:"power_on_delay"`
	CameraModel  CameraModel   `json:"camera_model"`
}

// LoadConfig loads configuration from a file
func LoadConfig(filename string) (*Config, error) {
	cfg := &Config{
		Port:         "/dev/ttyUSB0",
		BaudRate:     9600,
		ReadTimeout:  defaultReadTimeout,
		PowerOnDelay: powerOnBootDelay,
		CameraModel:  ModelXU81,
	}

	if filename == "" {
		return cfg, nil
	}

	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, err
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
	ResetInputBuffer() error
}

// Command represents a camera command
type Command struct {
	name        string
	code        string
	paramLength int
	description string
	validator   func(params []byte, model CameraModel) error
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

	Raw []byte `json:"-"`
}

// SystemInfo represents system information
type SystemInfo struct {
	ProductName   string
	ROMVersion    string
	SerialNumber  string
	CameraVersion int
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
	SetWhiteBalance(mode int) error

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

var _ Camera = (*XU81Camera)(nil)

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
		description: "Get current pan speed",
	},
	"tilt_speed_request": {
		name: "Tilt Speed Request", code: "0152", paramLength: 1,
		description: "Get current tilt speed",
	},
	"pan_stop": {
		name: "Pan/Tilt Stop", code: "0153", paramLength: 1,
		description: "Stop pan/tilt movement",
	},
	"pan_right": {
		name: "Pan Right Start", code: "0153", paramLength: 1,
		description: "Start panning right",
	},
	"pan_left": {
		name: "Pan Left Start", code: "0153", paramLength: 1,
		description: "Start panning left",
	},
	"tilt_up": {
		name: "Tilt Up Start", code: "0153", paramLength: 1,
		description: "Start tilting up",
	},
	"tilt_down": {
		name: "Tilt Down Start", code: "0153", paramLength: 1,
		description: "Start tilting down",
	},
	"initialize_home": {
		name: "Pan-Tilt Head Initialize 1", code: "0158", paramLength: 1,
		description: "Initialize and move to home position",
	},
	"initialize_original": {
		name: "Pan-Tilt Head Initialize 2", code: "0158", paramLength: 1,
		description: "Initialize and move to original position",
	},
	"pan_tilt_pos": {
		name: "Pan/Tilt Position Assignment", code: "0162", paramLength: 8,
		description: "Set absolute pan/tilt position",
	},
	"pan_tilt_request": {
		name: "Pan/Tilt Angle Request", code: "0163", paramLength: 0,
		description: "Get current pan/tilt position",
	},

	"wiper_control": {
		name: "Wiper Control", code: "017A", paramLength: 1,
		description: "Operate wiper (one cycle)",
	},
	"washer_control": {
		name: "Washer Control", code: "017A", paramLength: 1,
		description: "Auto-washer (4 wiper cycles, washer on first 2)",
	},
	"washer_out": {
		name: "Washer Output On/Off", code: "0171", paramLength: 2,
		description: "Manually turn washer on or off",
	},

	// Camera Control Commands
	"camera_version": {
		name: "Camera Version Request", code: "01BE", paramLength: 1,
		description: "Get camera version",
	},
	"power_on": {
		name: "Camera Power ON", code: "01A0", paramLength: 1,
		description: "Turn on the camera power",
	},
	"power_off": {
		name: "Camera Power OFF", code: "01A0", paramLength: 1,
		description: "Turn off the camera power",
	},
	"focus_auto": {
		name: "Focus Auto", code: "01A1", paramLength: 1,
		description: "Enable auto focus",
	},
	"focus_manual": {
		name: "Focus Manual", code: "01A1", paramLength: 1,
		description: "Set manual focus mode",
	},
	"focus_infinity": {
		name: "Focus Fixed at Infinity", code: "01A1", paramLength: 1,
		description: "Fix focus at infinity",
	},
	"focus_near": {
		name: "Focus Near", code: "01A1", paramLength: 1,
		description: "Move focus near",
	},
	"focus_far": {
		name: "Focus Far", code: "01A1", paramLength: 1,
		description: "Move focus far",
	},
	"focus_oneshot": {
		name: "One-shot AF", code: "01A1", paramLength: 1,
		description: "Perform one-shot autofocus",
	},
	"zoom_stop": {
		name: "Zoom Stop", code: "01A2", paramLength: 1,
		description: "Stop zoom operation",
	},
	"zoom_wide": {
		name: "Zoom Wide", code: "01A2", paramLength: 1,
		description: "Zoom to wide",
	},
	"zoom_tele": {
		name: "Zoom Tele", code: "01A2", paramLength: 1,
		description: "Zoom to telephoto",
	},
	"zoom_pos": {
		name: "Zoom Position", code: "01A3", paramLength: 2,
		description: "Set zoom position (00h-EEh)",
	},
	"zoom_pos_request": {
		name: "Zoom Position Request", code: "01A4", paramLength: 0,
		description: "Get current zoom position",
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
		description: "Set shutter speed",
	},
	"gain_assignment": {
		name: "Gain Assignment", code: "01AE", paramLength: 1,
		description: "Set camera gain",
	},
	"iris_assignment": {
		name: "Iris Assignment", code: "01A6", paramLength: 2,
		description: "Set iris position",
	},
	"white_balance": {
		name: "White Balance Control", code: "01A7", paramLength: 1,
		description: "Set white balance mode",
	},
	"wdr": {
		name: "WDR ON/OFF", code: "01A5", paramLength: 1,
		description: "Wide Dynamic Range on/off",
	},
	"nd_filter": {
		name: "ND Filter ON/OFF", code: "01B6", paramLength: 1,
		description: "Neutral Density filter on/off",
	},
	"ir_filter": {
		name: "IR Filter ON/OFF", code: "01BB", paramLength: 1,
		description: "Toggle IR filter",
	},
	"auto_ir": {
		name: "Auto IR Control", code: "01BA", paramLength: 1,
		description: "Auto IR filter control", validator: validateAutoIRParam,
	},
	"color_bar": {
		name: "Color Bar Control", code: "01B8", paramLength: 1,
		description: "Color bar on/off",
	},

	// System Control Commands
	"status": {
		name: "Operation Status", code: "0186", paramLength: 0,
		description: "Get camera operation status",
	},
	"extended_status": {
		name: "Extended Operation Status", code: "0186", paramLength: 1,
		description: "Get extended operation status",
	},
	"product_name": {
		name: "Product Name Request", code: "0187", paramLength: 0,
		description: "Get product name",
	},
	"rom_version": {
		name: "ROM Version Request", code: "0188", paramLength: 0,
		description: "Get ROM version",
	},
	"speed_control": {
		name: "Zoom/Focus/Tilt/Pan Speed Control", code: "0181", paramLength: 8,
		description: "Simultaneous velocity control of all axes", validator: validateDriveParam,
	},
	"shot_memory_record": {
		name: "Shot Memory Recording", code: "0182", paramLength: 2,
		description: "Record current position to shot memory",
	},
	"shot_memory_move": {
		name: "Shot Memory Movement", code: "0183", paramLength: 4,
		description: "Move to shot memory position",
	},
	"serial_number": {
		name: "Serial Number Request", code: "01E9", paramLength: 1,
		description: "Get serial number",
	},
}

// Validation functions check parameter semantics; sendCommand already
// enforces the length declared in the command table.
func validateSpeedParam(params []byte, model CameraModel) error {
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

func validateDigitalZoomParam(params []byte, model CameraModel) error {
	if model != ModelXU81 {
		return errors.New("digital zoom only available on XU-81")
	}
	return nil
}

func validateShootingModeParam(params []byte, model CameraModel) error {
	mode := params[0]
	if mode < 0x30 || mode > 0x34 { // ASCII '0' to '4'
		return errors.New("invalid shooting mode parameter")
	}
	return nil
}

func validateAutoIRParam(params []byte, model CameraModel) error {
	if params[0] < 0x30 || params[0] > 0x32 { // ASCII '0'-'2'
		return errors.New("auto IR parameter must be 0 (off), 1 (slow shutter), or 2 (gain)")
	}
	return nil
}

func validateDriveParam(params []byte, model CameraModel) error {
	for i := 0; i < 8; i += 2 {
		v, err := strconv.ParseUint(string(params[i:i+2]), 16, 8)
		if err != nil {
			return fmt.Errorf("invalid speed control parameter %q: %w", params[i:i+2], err)
		}
		if v > 0x7F {
			return errors.New("speed control axis values must be 00h-7Fh")
		}
	}
	return nil
}

// ensureConnectedLocked ensures the serial port is connected.
// Caller must hold cam.mu.
func (cam *XU81Camera) ensureConnectedLocked() error {
	if !cam.isConnected || cam.serial == nil {
		return cam.openPort()
	}
	return nil
}

// markDisconnectedLocked drops the serial connection after an I/O error so
// the next command reopens the port (e.g. after a USB adapter re-enumerates).
// Caller must hold cam.mu.
func (cam *XU81Camera) markDisconnectedLocked() {
	if cam.serial != nil {
		_ = cam.serial.Close()
	}
	cam.serial = nil
	cam.isConnected = false
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

// sendCommand sends a command to the camera and returns the response.
// The whole write/read transaction holds cam.mu — the protocol is
// half-duplex, so commands must never interleave on the wire.
func (cam *XU81Camera) sendCommand(cmd Command, params []byte) (Response, error) {
	if len(params) != cmd.paramLength {
		return Response{}, fmt.Errorf("command %s requires %d parameter byte(s), got %d",
			cmd.name, cmd.paramLength, len(params))
	}
	if cmd.validator != nil {
		if err := cmd.validator(params, cam.config.CameraModel); err != nil {
			return Response{}, fmt.Errorf("parameter validation failed: %w", err)
		}
	}

	cam.mu.Lock()
	defer cam.mu.Unlock()

	if err := cam.ensureConnectedLocked(); err != nil {
		return Response{}, fmt.Errorf("connection error: %w", err)
	}

	// Drop stale bytes from previous timeouts before transmitting — the
	// protocol is strictly command/answer, so the buffer must be empty.
	_ = cam.serial.ResetInputBuffer()

	cmdBytes := formatCommandBytes(cmd, params)

	cam.logger.Printf("Sending command %s: %X", cmd.name, cmdBytes)
	if _, err := cam.serial.Write(cmdBytes); err != nil {
		cam.markDisconnectedLocked()
		return Response{}, fmt.Errorf("failed to send command: %w", err)
	}

	resp, err := cam.readResponse()
	if err != nil {
		// A timeout means the camera didn't answer; the link itself is
		// fine. Anything else is an I/O error — reopen on next command.
		if !errors.Is(err, errResponseTimeout) {
			cam.markDisconnectedLocked()
		}
		return Response{}, fmt.Errorf("command %s failed: %w", cmd.name, err)
	}
	if resp.Error != nil {
		// The camera answered with a NAK or an error code
		return resp, fmt.Errorf("command %s: %w", cmd.name, resp.Error)
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
		return cam.waitPowerOn(cam.config.PowerOnDelay)
	}
	return nil
}

const powerOnPollInterval = time.Second

// waitPowerOn polls the camera until it reports power on, returning as soon
// as it is ready instead of sleeping a fixed boot delay. Errors are expected
// while booting (the camera stays silent or answers Busy), so they only
// count once the deadline has passed.
func (cam *XU81Camera) waitPowerOn(timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		status, err := cam.GetStatus()
		if err == nil && status.PowerOn {
			return nil
		}
		if time.Now().After(deadline) {
			if err != nil {
				return fmt.Errorf("camera not ready after %s: %w", timeout, err)
			}
			return fmt.Errorf("camera not ready after %s", timeout)
		}
		time.Sleep(powerOnPollInterval)
	}
}

func (cam *XU81Camera) PowerOff() error {
	c := commands["power_off"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}

// Movement control methods

// Stop halts motion on all four axes (pan, tilt, zoom, focus) with a single
// speed-control frame. The discrete pan_stop (0153) only halts pan/tilt.
func (cam *XU81Camera) Stop() error {
	return cam.Drive(0, 0, 0, 0)
}

// BestEffortStop halts pan/tilt movement if a connection was ever
// established. Used on exit paths so a dying process doesn't leave
// the head moving.
func (cam *XU81Camera) BestEffortStop() {
	cam.mu.Lock()
	connected := cam.isConnected
	cam.mu.Unlock()
	if !connected {
		return
	}
	if err := cam.Stop(); err != nil {
		cam.logger.Printf("auto-stop failed: %v", err)
	}
}

func (cam *XU81Camera) PanRight() error {
	c := commands["pan_right"]
	_, err := cam.sendCommand(c, []byte{paramPanRight})
	return err
}

func (cam *XU81Camera) PanLeft() error {
	c := commands["pan_left"]
	_, err := cam.sendCommand(c, []byte{paramPanLeft})
	return err
}

func (cam *XU81Camera) TiltUp() error {
	c := commands["tilt_up"]
	_, err := cam.sendCommand(c, []byte{paramTiltUp})
	return err
}

func (cam *XU81Camera) TiltDown() error {
	c := commands["tilt_down"]
	_, err := cam.sendCommand(c, []byte{paramTiltDown})
	return err
}

// Drive operates all four axes at once with signed velocities — the joystick
// command (manual §11.5, 0181h). Speeds are -64..63 where 0 stops the axis
// and magnitude sets speed: +pan = right, +tilt = up, +zoom = tele,
// +focus = near. Drive(0, 0, 0, 0) stops all axes.
func (cam *XU81Camera) Drive(pan, tilt, zoom, focus int) error {
	for _, v := range []int{pan, tilt, zoom, focus} {
		if v < -64 || v > 63 {
			return errors.New("axis speeds must be between -64 and 63")
		}
	}

	// Wire encoding is 0-127 per axis with 63/64 as stop. Pan/tilt count
	// down from 63 (manual: 0 = max right / max top), zoom/focus count up
	// from 64 (127 = max tele side / max near side).
	// Wire order per manual: zoom, focus, tilt, pan.
	params := intToASCIIHex(64+zoom, 2)
	params = append(params, intToASCIIHex(64+focus, 2)...)
	params = append(params, intToASCIIHex(63-tilt, 2)...)
	params = append(params, intToASCIIHex(63-pan, 2)...)

	c := commands["speed_control"]
	_, err := cam.sendCommand(c, params)
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
	pan, err = hexToSignedInt(resp.Data[5:9])
	if err != nil {
		return 0, 0, fmt.Errorf("pan position: %w", err)
	}
	tilt, err = hexToSignedInt(resp.Data[9:13])
	if err != nil {
		return 0, 0, fmt.Errorf("tilt position: %w", err)
	}

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

func (cam *XU81Camera) SetWhiteBalance(mode int) error {
	if mode < 0 || mode > 4 {
		return errors.New("white balance mode must be 0-4")
	}
	c := commands["white_balance"]
	param := byte(0x30 + mode) // Convert to ASCII
	_, err := cam.sendCommand(c, []byte{param})
	return err
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

const waitIdlePollInterval = 250 * time.Millisecond

// WaitIdle polls the extended status until the camera reports no motion or
// busy state, or the timeout elapses. Useful after recall/goto, which return
// as soon as the camera accepts the command (movement is asynchronous).
// A Busy answer or a response timeout counts as "still moving", not a
// failure — the camera may not service status requests mid-move.
func (cam *XU81Camera) WaitIdle(timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		status, err := cam.GetExtendedStatus()
		if err != nil && !IsBusy(err) && !errors.Is(err, errResponseTimeout) {
			return err
		}
		if err == nil && !status.CameraBusy && !status.Panning && !status.Tilting &&
			!status.Zooming && !status.Focusing {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("camera still busy after %s", timeout)
		}
		time.Sleep(waitIdlePollInterval)
	}
}

func (cam *XU81Camera) GetSystemInfo() (*SystemInfo, error) {
	info := &SystemInfo{}

	resp, err := cam.sendCommand(commands["product_name"], nil)
	if err != nil {
		return nil, fmt.Errorf("product name request: %w", err)
	}
	if resp.Type != ResponseStatus || len(resp.Data) < 7 {
		return nil, errors.New("invalid product name response")
	}
	info.ProductName = string(resp.Data[5 : len(resp.Data)-1]) // exclude end mark

	resp, err = cam.sendCommand(commands["rom_version"], nil)
	if err != nil {
		return nil, fmt.Errorf("ROM version request: %w", err)
	}
	if resp.Type != ResponseStatus || len(resp.Data) < 7 {
		return nil, errors.New("invalid ROM version response")
	}
	info.ROMVersion = string(resp.Data[5 : len(resp.Data)-1])

	resp, err = cam.sendCommand(commands["camera_version"], []byte{paramOff})
	if err != nil {
		return nil, fmt.Errorf("camera version request: %w", err)
	}
	if resp.Type != ResponseStatus || len(resp.Data) < 7 {
		return nil, errors.New("invalid camera version response")
	}
	version, err := asciiHexToInt(resp.Data[5:7])
	if err != nil {
		return nil, fmt.Errorf("camera version parse: %w", err)
	}
	info.CameraVersion = version

	serial, err := cam.GetSerialNumber()
	if err != nil {
		return nil, fmt.Errorf("serial number request: %w", err)
	}
	info.SerialNumber = serial

	return info, nil
}

// GetSerialNumber returns the 15-character serial number information string
// (model name, product code, serial number, reserve).
func (cam *XU81Camera) GetSerialNumber() (string, error) {
	c := commands["serial_number"]
	resp, err := cam.sendCommand(c, []byte{paramOff})
	if err != nil {
		return "", err
	}
	if resp.Type != ResponseStatus || len(resp.Data) < 7 {
		return "", errors.New("invalid serial number response")
	}
	return string(resp.Data[5 : len(resp.Data)-1]), nil
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

// MoveShotMemory moves to a stored preset over the given duration.
// Per manual §11.7 valid movement times are 0 (instant) or 2-48 seconds;
// sending duration 0 again while a move is in progress cancels it.
func (cam *XU81Camera) MoveShotMemory(slot int, duration int) error {
	if slot < 0 || slot > 127 {
		return errors.New("shot memory slot must be 0-127")
	}
	if duration != 0 && (duration < 2 || duration > 48) {
		return errors.New("duration must be 0 (instant) or 2-48 seconds")
	}

	c := commands["shot_memory_move"]
	slotParams := intToASCIIHex(slot, 2)
	durationParams := intToASCIIHex(duration, 2)
	params := append(slotParams, durationParams...)
	_, err := cam.sendCommand(c, params)
	return err
}

// Additional camera control methods

// initializeTimeout bounds the wait for a pan-tilt head initialize sweep.
const initializeTimeout = 30 * time.Second

func (cam *XU81Camera) InitializeHome() error {
	c := commands["initialize_home"]
	if _, err := cam.sendCommand(c, []byte{paramOff}); err != nil {
		return err
	}
	cam.logger.Println("Camera initializing to home position...")
	return cam.WaitIdle(initializeTimeout)
}

func (cam *XU81Camera) InitializeOriginal() error {
	c := commands["initialize_original"]
	if _, err := cam.sendCommand(c, []byte{paramOn}); err != nil {
		return err
	}
	cam.logger.Println("Camera initializing to original position...")
	return cam.WaitIdle(initializeTimeout)
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

// Exposure controls
// SetShutterSpeed assigns the shutter speed (0=2s, 1=1/15, 2=1/60, 3=1/100,
// 4=1/1000 at 59.94). Requires Manual or Shutter Priority shooting mode.
func (cam *XU81Camera) SetShutterSpeed(val int) error {
	if val < 0 || val > 4 {
		return errors.New("shutter speed must be 0-4")
	}
	c := commands["shutter_speed"]
	_, err := cam.sendCommand(c, []byte{byte(0x30 + val)})
	return err
}

// SetGain assigns the camera gain (0=0dB, 1=6dB, 2=12dB, 3=18dB, 4=24dB,
// 5=32dB XU-80 / 42dB XU-81). Requires Manual shooting mode.
func (cam *XU81Camera) SetGain(val int) error {
	if val < 0 || val > 5 {
		return errors.New("gain must be 0-5")
	}
	c := commands["gain_assignment"]
	_, err := cam.sendCommand(c, []byte{byte(0x30 + val)})
	return err
}

// SetIris assigns the iris position (0-255; higher is brighter).
// Requires Manual or Iris Priority shooting mode.
func (cam *XU81Camera) SetIris(val int) error {
	if val < 0 || val > 255 {
		return errors.New("iris must be 0-255")
	}
	c := commands["iris_assignment"]
	_, err := cam.sendCommand(c, intToASCIIHex(val, 2))
	return err
}

// SetDigitalZoom assigns the digital zoom magnification in 0.1x steps above
// 1.0x: value = (magnification-1.0)*10, range 0-110 (1.0x-12.0x).
// XU-81 only; requires the Independent digital zoom mode (SW5).
func (cam *XU81Camera) SetDigitalZoom(val int) error {
	if val < 0 || val > 110 {
		return errors.New("digital zoom must be 0-110 ((magnification-1.0)*10)")
	}
	c := commands["digital_zoom"]
	_, err := cam.sendCommand(c, intToASCIIHex(val, 2))
	return err
}

// Auto IR filter control (Night mode only): 0=off, 1=on slow shutter, 2=on gain.
func (cam *XU81Camera) SetAutoIR(mode int) error {
	if mode < 0 || mode > 2 {
		return errors.New("auto IR mode must be 0-2")
	}
	c := commands["auto_ir"]
	_, err := cam.sendCommand(c, []byte{byte(0x30 + mode)})
	return err
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
	_, err := cam.sendCommand(c, []byte{paramFocusAuto})
	return err
}

func (cam *XU81Camera) FocusManual() error {
	c := commands["focus_manual"]
	_, err := cam.sendCommand(c, []byte{paramFocusManual})
	return err
}

func (cam *XU81Camera) FocusInfinity() error {
	c := commands["focus_infinity"]
	_, err := cam.sendCommand(c, []byte{paramFocusInfinity})
	return err
}

func (cam *XU81Camera) FocusNear() error {
	c := commands["focus_near"]
	_, err := cam.sendCommand(c, []byte{paramFocusNear})
	return err
}

func (cam *XU81Camera) FocusFar() error {
	c := commands["focus_far"]
	_, err := cam.sendCommand(c, []byte{paramFocusFar})
	return err
}

func (cam *XU81Camera) FocusOneShot() error {
	c := commands["focus_oneshot"]
	_, err := cam.sendCommand(c, []byte{paramFocusOneShot})
	return err
}

// Zoom controls
func (cam *XU81Camera) ZoomStop() error {
	c := commands["zoom_stop"]
	_, err := cam.sendCommand(c, []byte{paramZoomStop})
	return err
}

func (cam *XU81Camera) ZoomWide() error {
	c := commands["zoom_wide"]
	_, err := cam.sendCommand(c, []byte{paramZoomWide})
	return err
}

func (cam *XU81Camera) ZoomTele() error {
	c := commands["zoom_tele"]
	_, err := cam.sendCommand(c, []byte{paramZoomTele})
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
	params := []byte{paramWasherSelect, paramOn}
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) WasherOutputOff() error {
	c := commands["washer_out"]
	params := []byte{paramWasherSelect, paramOff}
	_, err := cam.sendCommand(c, params)
	return err
}

func (cam *XU81Camera) WiperControl() error {
	c := commands["wiper_control"]
	_, err := cam.sendCommand(c, []byte{paramOff})
	return err
}
