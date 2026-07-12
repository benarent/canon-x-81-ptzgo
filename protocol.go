package main

import (
	"bytes"
	"encoding/hex"
	"errors"
	"fmt"
	"io"
	"strconv"
	"strings"
	"time"
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

	// Pan/tilt drive directions (0153; '0' stops)
	paramPanRight = 0x31 // '1'
	paramPanLeft  = 0x32 // '2'
	paramTiltUp   = 0x33 // '3'
	paramTiltDown = 0x34 // '4'

	// Focus operations (01A1)
	paramFocusAuto     = 0x30 // '0'
	paramFocusManual   = 0x31 // '1'
	paramFocusNear     = 0x32 // '2'
	paramFocusFar      = 0x33 // '3'
	paramFocusOneShot  = 0x41 // 'A'
	paramFocusInfinity = 0x42 // 'B'

	// Zoom operations (01A2)
	paramZoomStop = 0x30 // '0'
	paramZoomWide = 0x31 // '1'
	paramZoomTele = 0x32 // '2'

	// Washer output (0171): 'B' selects the washer, followed by on/off
	paramWasherSelect = 0x42 // 'B'
	minSpeed          = 5
	maxSpeed          = 3000
	maxZoom           = 0xEE
	minPanAngle       = -9000
	maxPanAngle       = 9000
	minTiltAngle      = -6500
	maxTiltAngle      = 6500
)

// Response types
type ResponseType int

const (
	ResponseACK ResponseType = iota
	ResponseNAK
	ResponseStatus
	ResponseError
)

// Response represents a parsed camera response
type Response struct {
	Type      ResponseType
	Data      []byte
	ErrorCode string
	Error     error
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

func hexToSignedInt(data []byte) (int, error) {
	if len(data) != 4 {
		return 0, fmt.Errorf("expected 4 hex digits, got %d", len(data))
	}
	unsigned, err := strconv.ParseUint(string(data), 16, 16)
	if err != nil {
		return 0, fmt.Errorf("invalid hex value %q: %w", data, err)
	}
	return int(unsigned) - 0x8000, nil
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
			return Response{
				Type: ResponseStatus,
				Data: data,
			}
		}
	}

	return Response{Type: ResponseError, Error: fmt.Errorf("unknown response format: %X", data)}
}

// errResponseTimeout is returned when the camera does not answer in time.
// It indicates an absent or busy camera, not a broken serial link.
var errResponseTimeout = errors.New("timeout waiting for response")

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

		// Discard leading garbage until a valid response start byte
		for len(buf) > 0 && buf[0] != ackByte && buf[0] != nakByte && buf[0] != responseHeader {
			buf = buf[1:]
		}
		if len(buf) == 0 {
			continue
		}

		// Check for complete response
		if buf[0] == ackByte || buf[0] == nakByte {
			return cam.parseResponse(buf[:1]), nil
		}
		if buf[0] == responseHeader {
			// Accept the first complete frame; stale trailing bytes are
			// dropped rather than poisoning this and later responses.
			if i := bytes.IndexByte(buf, endMarkByte); i >= 5 {
				return cam.parseResponse(buf[:i+1]), nil
			}
		}
	}

	return Response{}, errResponseTimeout
}

// CameraError is a decoded camera error-flag answer.
// Per Programmer's Manual §5.2: first digit b3=Mode Error, b2=Parameter Error,
// b1=Command Error, b0=Busy; second digit b0=System Error (b1-b3 reserved).
// The camera sets Busy alongside every other flag, so transient failures can
// be detected with IsBusy and retried.
type CameraError struct {
	Code      string // the two ASCII-hex digits as received
	Mode      bool
	Parameter bool
	Command   bool
	Busy      bool
	System    bool
}

func (e *CameraError) Error() string {
	var errs []string
	if e.Mode {
		errs = append(errs, "Mode Error")
	}
	if e.Parameter {
		errs = append(errs, "Parameter Error")
	}
	if e.Command {
		errs = append(errs, "Command Error")
	}
	if e.Busy {
		errs = append(errs, "Busy")
	}
	if e.System {
		errs = append(errs, "System Error")
	}
	return fmt.Sprintf("camera errors: %s", strings.Join(errs, ", "))
}

// IsBusy reports whether err carries a camera answer with the Busy flag set.
func IsBusy(err error) bool {
	if e, ok := errors.AsType[*CameraError](err); ok {
		return e.Busy
	}
	return false
}

// parseErrorCode interprets the camera's two ASCII-hex error-flag digits.
func parseErrorCode(code string) error {
	if len(code) != 2 {
		return fmt.Errorf("malformed error code: %q", code)
	}
	hi, err := strconv.ParseUint(code[:1], 16, 8)
	if err != nil {
		return fmt.Errorf("malformed error code: %q", code)
	}
	lo, err := strconv.ParseUint(code[1:], 16, 8)
	if err != nil {
		return fmt.Errorf("malformed error code: %q", code)
	}

	camErr := &CameraError{
		Code:      code,
		Mode:      hi&0x8 != 0,
		Parameter: hi&0x4 != 0,
		Command:   hi&0x2 != 0,
		Busy:      hi&0x1 != 0,
		System:    lo&0x1 != 0,
	}
	if !camErr.Mode && !camErr.Parameter && !camErr.Command && !camErr.Busy && !camErr.System {
		return fmt.Errorf("unknown error code: %s", code)
	}
	return camErr
}
