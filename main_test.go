package main

import (
	"bytes"
	"errors"
	"fmt"
	"io"
	"log"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// mockSerialPort is a scripted SerialPort: Read serves bytes from readBuf,
// Write records each frame. For multi-command exchanges, each Write loads
// the next queued answer from responseQueue into readBuf (command/answer).
type mockSerialPort struct {
	readBuf       []byte
	responseQueue [][]byte
	writes        [][]byte
	readErr       error
	resets        int
}

func (m *mockSerialPort) Read(p []byte) (int, error) {
	if m.readErr != nil {
		return 0, m.readErr
	}
	if len(m.readBuf) == 0 {
		return 0, nil // a read timeout returns no data and no error
	}
	n := copy(p, m.readBuf)
	m.readBuf = m.readBuf[n:]
	return n, nil
}

func (m *mockSerialPort) Write(p []byte) (int, error) {
	frame := make([]byte, len(p))
	copy(frame, p)
	m.writes = append(m.writes, frame)
	if len(m.responseQueue) > 0 {
		m.readBuf = append(m.readBuf, m.responseQueue[0]...)
		m.responseQueue = m.responseQueue[1:]
	}
	return len(p), nil
}

func (m *mockSerialPort) Close() error                       { return nil }
func (m *mockSerialPort) SetReadTimeout(time.Duration) error { return nil }

func (m *mockSerialPort) ResetInputBuffer() error {
	m.resets++
	return nil
}

func newTestCamera(mock *mockSerialPort) *XU81Camera {
	return &XU81Camera{
		serial:      mock,
		isConnected: true,
		logger:      log.New(io.Discard, "", 0),
		config:      &Config{CameraModel: ModelXU81},
		respTimeout: 20 * time.Millisecond, // keep timeout tests fast
	}
}

// statusResponse builds a camera answer: FE 3030 3030 <payload> EF
func statusResponse(payload string) []byte {
	resp := []byte{responseHeader, 0x30, 0x30, 0x30, 0x30}
	resp = append(resp, []byte(payload)...)
	return append(resp, endMarkByte)
}

func TestFormatCommandBytes(t *testing.T) {
	tests := []struct {
		name   string
		cmd    Command
		params []byte
		want   []byte
	}{
		{
			name:   "command with param",
			cmd:    commands["power_on"],
			params: []byte{paramOn},
			want:   []byte{0xFF, 0x30, 0x30, 0x01, 0xA0, 0x31, 0xEF},
		},
		{
			name: "request without params",
			cmd:  commands["status"],
			want: []byte{0xFF, 0x30, 0x30, 0x01, 0x86, 0xEF},
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if got := formatCommandBytes(tt.cmd, tt.params); !bytes.Equal(got, tt.want) {
				t.Errorf("got %X, want %X", got, tt.want)
			}
		})
	}
}

func TestParseResponse(t *testing.T) {
	cam := newTestCamera(&mockSerialPort{})

	tests := []struct {
		name     string
		data     []byte
		wantType ResponseType
		wantErr  bool
	}{
		{"status ok", statusResponse("A00"), ResponseStatus, false},
		{"no-status answer", statusResponse(""), ResponseStatus, false},
		{"empty", nil, ResponseError, true},
		{"unknown header", []byte{0x42, 0x30, 0x30, 0x30, 0x30, 0xEF}, ResponseError, true},
		{"truncated status", []byte{responseHeader, 0x30, 0x30}, ResponseError, true},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			resp := cam.parseResponse(tt.data)
			if resp.Type != tt.wantType {
				t.Errorf("type = %v, want %v", resp.Type, tt.wantType)
			}
			if (resp.Error != nil) != tt.wantErr {
				t.Errorf("error = %v, wantErr %v", resp.Error, tt.wantErr)
			}
		})
	}

	t.Run("error code response", func(t *testing.T) {
		// Parameter Error answer per manual: error code chars "50"
		resp := cam.parseResponse([]byte{responseHeader, 0x30, 0x30, '5', '0', endMarkByte})
		if resp.Type != ResponseError {
			t.Fatalf("type = %v, want ResponseError", resp.Type)
		}
		if resp.ErrorCode != "50" {
			t.Errorf("error code = %q, want %q", resp.ErrorCode, "50")
		}
		if resp.Error == nil || !strings.Contains(resp.Error.Error(), "Parameter Error") {
			t.Errorf("error = %v, want Parameter Error", resp.Error)
		}
	})
}

func TestParseErrorCode(t *testing.T) {
	tests := []struct {
		code      string
		wantFlags []string
		wantErr   string
	}{
		// Examples from Programmer's Manual §5.2: Busy is set alongside any error
		{code: "50", wantFlags: []string{"Parameter Error", "Busy"}},
		{code: "90", wantFlags: []string{"Mode Error", "Busy"}},
		{code: "30", wantFlags: []string{"Command Error", "Busy"}},
		{code: "01", wantFlags: []string{"System Error"}},
		{code: "F1", wantFlags: []string{"Mode Error", "Parameter Error", "Command Error", "Busy", "System Error"}},
		{code: "00", wantErr: "unknown error code"},
		{code: "ZZ", wantErr: "malformed error code"},
		{code: "5", wantErr: "malformed error code"},
	}
	for _, tt := range tests {
		t.Run(tt.code, func(t *testing.T) {
			err := parseErrorCode(tt.code)
			if err == nil {
				t.Fatal("expected an error, got nil")
			}
			if tt.wantErr != "" {
				if !strings.Contains(err.Error(), tt.wantErr) {
					t.Errorf("error = %v, want substring %q", err, tt.wantErr)
				}
				if _, ok := errors.AsType[*CameraError](err); ok {
					t.Error("malformed/unknown codes should not decode to *CameraError")
				}
				return
			}
			for _, flag := range tt.wantFlags {
				if !strings.Contains(err.Error(), flag) {
					t.Errorf("error = %v, missing flag %q", err, flag)
				}
			}
		})
	}

	t.Run("structured flags", func(t *testing.T) {
		camErr, ok := errors.AsType[*CameraError](parseErrorCode("50"))
		if !ok {
			t.Fatal("expected *CameraError")
		}
		want := &CameraError{Code: "50", Parameter: true, Busy: true}
		if *camErr != *want {
			t.Errorf("got %+v, want %+v", camErr, want)
		}
	})
}

func TestIsBusy(t *testing.T) {
	wrapped := fmt.Errorf("command Pan Speed Assignment: %w", &CameraError{Code: "10", Busy: true})
	if !IsBusy(wrapped) {
		t.Error("wrapped Busy CameraError should report busy")
	}
	if IsBusy(&CameraError{Code: "01", System: true}) {
		t.Error("System-only CameraError should not report busy")
	}
	if IsBusy(errors.New("plain error")) {
		t.Error("plain error should not report busy")
	}
	if IsBusy(nil) {
		t.Error("nil should not report busy")
	}
}

func TestSendCommandCameraError(t *testing.T) {
	t.Run("error code answer", func(t *testing.T) {
		// Parameter Error answer: error code chars "50" (Parameter + Busy)
		mock := &mockSerialPort{readBuf: []byte{responseHeader, 0x30, 0x30, '5', '0', endMarkByte}}
		cam := newTestCamera(mock)
		err := cam.Stop()
		if err == nil {
			t.Fatal("expected error on error-code answer")
		}
		camErr, ok := errors.AsType[*CameraError](err)
		if !ok {
			t.Fatalf("expected *CameraError in chain, got %v", err)
		}
		if !camErr.Parameter || !camErr.Busy {
			t.Errorf("got %+v, want Parameter and Busy set", camErr)
		}
		if !IsBusy(err) {
			t.Errorf("IsBusy = false, want true: %v", err)
		}
	})
}

func TestSignedHexRoundTrip(t *testing.T) {
	tests := []struct {
		value int
		hex   string
	}{
		{0, "8000"},
		{9000, "A328"},
		{-9000, "5CD8"},
		{6500, "9964"},
		{-6500, "669C"},
	}
	for _, tt := range tests {
		if got := string(signedIntToHex(tt.value)); got != tt.hex {
			t.Errorf("signedIntToHex(%d) = %s, want %s", tt.value, got, tt.hex)
		}
		got, err := hexToSignedInt([]byte(tt.hex))
		if err != nil {
			t.Fatalf("hexToSignedInt(%s): %v", tt.hex, err)
		}
		if got != tt.value {
			t.Errorf("hexToSignedInt(%s) = %d, want %d", tt.hex, got, tt.value)
		}
	}

	if _, err := hexToSignedInt([]byte("12")); err == nil {
		t.Error("expected error for wrong length")
	}
	if _, err := hexToSignedInt([]byte("GGGG")); err == nil {
		t.Error("expected error for invalid hex")
	}
}

func TestASCIIHexHelpers(t *testing.T) {
	if got := string(intToASCIIHex(238, 2)); got != "EE" {
		t.Errorf("intToASCIIHex(238, 2) = %s, want EE", got)
	}
	if got := string(intToASCIIHex(3000, 3)); got != "BB8" {
		t.Errorf("intToASCIIHex(3000, 3) = %s, want BB8", got)
	}
	got, err := asciiHexToInt([]byte("BB8"))
	if err != nil || got != 3000 {
		t.Errorf("asciiHexToInt(BB8) = %d, %v, want 3000", got, err)
	}
}

func TestValidators(t *testing.T) {
	t.Run("speed", func(t *testing.T) {
		if err := validateSpeedParam([]byte("005"), ModelXU81); err != nil {
			t.Errorf("speed 5 should be valid: %v", err)
		}
		if err := validateSpeedParam([]byte("BB8"), ModelXU81); err != nil {
			t.Errorf("speed 3000 should be valid: %v", err)
		}
		if err := validateSpeedParam([]byte("004"), ModelXU81); err == nil {
			t.Error("speed 4 should be rejected")
		}
		if err := validateSpeedParam([]byte("BB9"), ModelXU81); err == nil {
			t.Error("speed 3001 should be rejected")
		}
	})

	t.Run("digital zoom model gating", func(t *testing.T) {
		if err := validateDigitalZoomParam([]byte("6E"), ModelXU81); err != nil {
			t.Errorf("XU-81 should accept digital zoom: %v", err)
		}
		if err := validateDigitalZoomParam([]byte("6E"), ModelXU80); err == nil {
			t.Error("XU-80 should reject digital zoom")
		}
	})

	t.Run("shooting mode", func(t *testing.T) {
		for _, p := range []byte{'0', '1', '2', '3', '4'} {
			if err := validateShootingModeParam([]byte{p}, ModelXU81); err != nil {
				t.Errorf("mode %c should be valid: %v", p, err)
			}
		}
		if err := validateShootingModeParam([]byte{'5'}, ModelXU81); err == nil {
			t.Error("mode 5 should be rejected")
		}
	})

	t.Run("auto IR", func(t *testing.T) {
		for _, p := range []byte{'0', '1', '2'} {
			if err := validateAutoIRParam([]byte{p}, ModelXU81); err != nil {
				t.Errorf("auto IR %c should be valid: %v", p, err)
			}
		}
		if err := validateAutoIRParam([]byte{'3'}, ModelXU81); err == nil {
			t.Error("auto IR 3 should be rejected")
		}
	})
}

func TestSendCommandFraming(t *testing.T) {
	mock := &mockSerialPort{readBuf: statusResponse("")}
	cam := newTestCamera(mock)

	if err := cam.Stop(); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	// Stop drives all axes to zero: zoom "40", focus "40", tilt "3F", pan "3F"
	want := []byte{0xFF, 0x30, 0x30, 0x01, 0x81, '4', '0', '4', '0', '3', 'F', '3', 'F', 0xEF}
	if len(mock.writes) != 1 || !bytes.Equal(mock.writes[0], want) {
		t.Errorf("wrote %X, want %X", mock.writes, want)
	}
}

func TestSendCommandParamLength(t *testing.T) {
	// Length comes from the command table, enforced before any validator.
	mock := &mockSerialPort{}
	cam := newTestCamera(mock)

	if _, err := cam.sendCommand(commands["pan_speed"], []byte("05")); err == nil {
		t.Error("2 bytes for a 3-byte command should be rejected")
	}
	if _, err := cam.sendCommand(commands["status"], []byte{paramOff}); err == nil {
		t.Error("params on a 0-param command should be rejected")
	}
	if len(mock.writes) != 0 {
		t.Errorf("nothing should be written on length failure, got %X", mock.writes)
	}
}

func TestSendCommandValidationFailure(t *testing.T) {
	mock := &mockSerialPort{}
	cam := newTestCamera(mock)

	_, err := cam.sendCommand(commands["pan_speed"], []byte("004"))
	if err == nil {
		t.Fatal("expected validation error")
	}
	if len(mock.writes) != 0 {
		t.Errorf("nothing should be written on validation failure, got %X", mock.writes)
	}
}

func TestGetPanTiltPosition(t *testing.T) {
	mock := &mockSerialPort{readBuf: statusResponse("A3285CD8")}
	cam := newTestCamera(mock)

	pan, tilt, err := cam.GetPanTiltPosition()
	if err != nil {
		t.Fatalf("GetPanTiltPosition: %v", err)
	}
	if pan != 9000 || tilt != -9000 {
		t.Errorf("got pan=%d tilt=%d, want pan=9000 tilt=-9000", pan, tilt)
	}
}

func TestGetStatus(t *testing.T) {
	// Status bits 0xA00: Panning (b9) and Tilting (b11); power-off flag (b5) clear
	mock := &mockSerialPort{readBuf: statusResponse("A00")}
	cam := newTestCamera(mock)

	status, err := cam.GetStatus()
	if err != nil {
		t.Fatalf("GetStatus: %v", err)
	}
	if !status.PowerOn || !status.Panning || !status.Tilting {
		t.Errorf("got %+v, want PowerOn, Panning, and Tilting set", status)
	}
	if status.Zooming || status.Focusing {
		t.Errorf("got %+v, want Zooming and Focusing clear", status)
	}
}

func TestGetExtendedStatus(t *testing.T) {
	// 0x12200: Panning (b9), CameraManualMode (b13), PanMotorError (b16);
	// power-off flag (b5) clear
	mock := &mockSerialPort{readBuf: statusResponse("12200")}
	cam := newTestCamera(mock)

	status, err := cam.GetExtendedStatus()
	if err != nil {
		t.Fatalf("GetExtendedStatus: %v", err)
	}
	if !status.PowerOn || !status.Panning || !status.CameraManualMode || !status.PanMotorError {
		t.Errorf("got %+v, want PowerOn, Panning, CameraManualMode, PanMotorError set", status)
	}
	if status.TiltMotorError || status.Zooming || status.CameraBusy {
		t.Errorf("got %+v, want TiltMotorError, Zooming, CameraBusy clear", status)
	}
}

func TestGetSerialNumber(t *testing.T) {
	mock := &mockSerialPort{readBuf: statusResponse("XU-80W2500000--")}
	cam := newTestCamera(mock)

	serial, err := cam.GetSerialNumber()
	if err != nil {
		t.Fatalf("GetSerialNumber: %v", err)
	}
	if serial != "XU-80W2500000--" {
		t.Errorf("got %q, want %q", serial, "XU-80W2500000--")
	}
}

func TestSendCommandMarksDisconnected(t *testing.T) {
	t.Run("read error disconnects", func(t *testing.T) {
		mock := &mockSerialPort{readErr: errors.New("device unplugged")}
		cam := newTestCamera(mock)
		if err := cam.Stop(); err == nil {
			t.Fatal("expected error on read failure")
		}
		cam.mu.Lock()
		connected := cam.isConnected
		cam.mu.Unlock()
		if connected {
			t.Error("camera should be marked disconnected after a read error")
		}
	})

	t.Run("timeout keeps connection", func(t *testing.T) {
		mock := &mockSerialPort{} // no answer at all
		cam := newTestCamera(mock)
		if err := cam.Stop(); err == nil {
			t.Fatal("expected timeout error")
		}
		cam.mu.Lock()
		connected := cam.isConnected
		cam.mu.Unlock()
		if !connected {
			t.Error("a response timeout should not drop the serial connection")
		}
	})
}

func TestReadResponseTrailingGarbage(t *testing.T) {
	frame := statusResponse("A00")
	mock := &mockSerialPort{readBuf: append(append([]byte{}, frame...), 0x12, 0x34)}
	cam := newTestCamera(mock)

	resp, err := cam.readResponse()
	if err != nil {
		t.Fatalf("readResponse: %v", err)
	}
	if resp.Type != ResponseStatus {
		t.Errorf("type = %v, want ResponseStatus", resp.Type)
	}
	if !bytes.Equal(resp.Data, frame) {
		t.Errorf("data = %X, want first frame %X", resp.Data, frame)
	}
}

func TestMoveShotMemoryDuration(t *testing.T) {
	// Manual §11.7: valid movement times are 0 or 2-48 seconds
	for _, d := range []int{1, -1, 49} {
		cam := newTestCamera(&mockSerialPort{})
		if err := cam.MoveShotMemory(1, d); err == nil {
			t.Errorf("duration %d should be rejected", d)
		}
	}
	for _, d := range []int{0, 2, 48} {
		mock := &mockSerialPort{readBuf: statusResponse("")}
		cam := newTestCamera(mock)
		if err := cam.MoveShotMemory(1, d); err != nil {
			t.Errorf("duration %d should be accepted: %v", d, err)
		}
	}
}

func TestDriveFraming(t *testing.T) {
	mock := &mockSerialPort{readBuf: statusResponse("")}
	cam := newTestCamera(mock)

	// Max-speed pan right, everything else stopped.
	// Wire order zoom, focus, tilt, pan: 64→"40", 64→"40", 63→"3F", 0→"00".
	if err := cam.Drive(63, 0, 0, 0); err != nil {
		t.Fatalf("Drive: %v", err)
	}
	want := []byte{0xFF, 0x30, 0x30, 0x01, 0x81, '4', '0', '4', '0', '3', 'F', '0', '0', 0xEF}
	if len(mock.writes) != 1 || !bytes.Equal(mock.writes[0], want) {
		t.Errorf("wrote %X, want %X", mock.writes, want)
	}

	if err := cam.Drive(64, 0, 0, 0); err == nil {
		t.Error("speed 64 should be rejected")
	}
	if err := cam.Drive(0, 0, -65, 0); err == nil {
		t.Error("speed -65 should be rejected")
	}
}

func TestDriveNegativeFraming(t *testing.T) {
	// Pan/tilt count down from 63, zoom/focus count up from 64, so negative
	// speeds must invert differently per axis pair.
	tests := []struct {
		name                   string
		pan, tilt, zoom, focus int
		params                 string // wire order: zoom, focus, tilt, pan
	}{
		{"all max negative", -64, -64, -64, -64, "00007F7F"},
		{"mixed signs", 10, -10, 5, -5, "453B4935"},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			mock := &mockSerialPort{readBuf: statusResponse("")}
			cam := newTestCamera(mock)
			if err := cam.Drive(tt.pan, tt.tilt, tt.zoom, tt.focus); err != nil {
				t.Fatalf("Drive: %v", err)
			}
			want := append([]byte{0xFF, 0x30, 0x30, 0x01, 0x81}, []byte(tt.params)...)
			want = append(want, 0xEF)
			if len(mock.writes) != 1 || !bytes.Equal(mock.writes[0], want) {
				t.Errorf("wrote %X, want %X", mock.writes, want)
			}
		})
	}
}

func TestRunCommandArgCount(t *testing.T) {
	mock := &mockSerialPort{}
	cam := newTestCamera(mock)

	if err := runCommand(cam, "bogus", nil); err == nil || !strings.Contains(err.Error(), "unknown command") {
		t.Errorf("unknown command should be rejected, got %v", err)
	}
	if err := runCommand(cam, "goto", []string{"1"}); err == nil || !strings.Contains(err.Error(), "usage:") {
		t.Errorf("too few args should show usage, got %v", err)
	}
	if err := runCommand(cam, "zoom", []string{"1", "2"}); err == nil || !strings.Contains(err.Error(), "usage:") {
		t.Errorf("too many args should show usage, got %v", err)
	}
	if len(mock.writes) != 0 {
		t.Errorf("nothing should be written on arg-count failure, got %X", mock.writes)
	}
}

func TestLoadConfigDurations(t *testing.T) {
	writeConfig := func(t *testing.T, content string) string {
		t.Helper()
		path := filepath.Join(t.TempDir(), "config.json")
		if err := os.WriteFile(path, []byte(content), 0o644); err != nil {
			t.Fatal(err)
		}
		return path
	}

	t.Run("duration strings and raw nanoseconds", func(t *testing.T) {
		cfg, err := LoadConfig(writeConfig(t,
			`{"port":"/dev/x","read_timeout":"250ms","power_on_delay":20000000000}`))
		if err != nil {
			t.Fatalf("LoadConfig: %v", err)
		}
		if got := time.Duration(cfg.ReadTimeout); got != 250*time.Millisecond {
			t.Errorf("ReadTimeout = %s, want 250ms", got)
		}
		if got := time.Duration(cfg.PowerOnDelay); got != 20*time.Second {
			t.Errorf("PowerOnDelay = %s, want 20s", got)
		}
	})

	t.Run("invalid duration string", func(t *testing.T) {
		if _, err := LoadConfig(writeConfig(t, `{"read_timeout":"soon"}`)); err == nil {
			t.Error("expected error for invalid duration string")
		}
	})

	t.Run("zero read timeout falls back to default", func(t *testing.T) {
		cfg, err := LoadConfig(writeConfig(t, `{"read_timeout":"0s"}`))
		if err != nil {
			t.Fatalf("LoadConfig: %v", err)
		}
		if got := time.Duration(cfg.ReadTimeout); got != defaultReadTimeout {
			t.Errorf("ReadTimeout = %s, want default %s", got, defaultReadTimeout)
		}
	})

	t.Run("missing file", func(t *testing.T) {
		if _, err := LoadConfig("/nonexistent/config.json"); err == nil || !strings.Contains(err.Error(), "reading config") {
			t.Errorf("expected wrapped read error, got %v", err)
		}
	})

	t.Run("zero baud rate and negative power-on delay fall back to defaults", func(t *testing.T) {
		cfg, err := LoadConfig(writeConfig(t, `{"baud_rate":0,"power_on_delay":"-5s"}`))
		if err != nil {
			t.Fatalf("LoadConfig: %v", err)
		}
		if cfg.BaudRate != 9600 {
			t.Errorf("BaudRate = %d, want 9600", cfg.BaudRate)
		}
		if got := time.Duration(cfg.PowerOnDelay); got != powerOnBootDelay {
			t.Errorf("PowerOnDelay = %s, want default %s", got, powerOnBootDelay)
		}
	})
}

func TestSetZoomPositionModelGating(t *testing.T) {
	t.Run("digital range rejected on XU-80", func(t *testing.T) {
		mock := &mockSerialPort{}
		cam := newTestCamera(mock)
		cam.config.CameraModel = ModelXU80
		if err := cam.SetZoomPosition(200); err == nil {
			t.Error("zoom 200 should be rejected on XU-80")
		}
		if len(mock.writes) != 0 {
			t.Errorf("nothing should be written on rejection, got %X", mock.writes)
		}
		mock.readBuf = statusResponse("")
		if err := cam.SetZoomPosition(128); err != nil {
			t.Errorf("optical zoom 128 should be accepted on XU-80: %v", err)
		}
	})

	t.Run("digital range accepted on XU-81", func(t *testing.T) {
		mock := &mockSerialPort{readBuf: statusResponse("")}
		cam := newTestCamera(mock)
		if err := cam.SetZoomPosition(200); err != nil {
			t.Errorf("zoom 200 should be accepted on XU-81: %v", err)
		}
	})
}

func TestPowerOnWaitsForBoot(t *testing.T) {
	// Per manual §9.3 the power-off flag clears as soon as Camera ON is
	// accepted; readiness is the extended-status camera-busy flag clearing.
	// Queue: Camera ON answer, then an extended status showing power on and
	// not busy — PowerOn must poll extended status before returning.
	mock := &mockSerialPort{responseQueue: [][]byte{
		statusResponse(""),      // Camera ON accepted
		statusResponse("00000"), // power on (b5 clear), not busy (b14 clear)
	}}
	cam := newTestCamera(mock)
	cam.config.PowerOnDelay = Duration(2 * time.Second)

	if err := cam.PowerOn(); err != nil {
		t.Fatalf("PowerOn: %v", err)
	}
	if len(mock.writes) != 2 {
		t.Fatalf("expected Camera ON + 1 status poll, got %d writes: %X", len(mock.writes), mock.writes)
	}
	wantPoll := []byte{0xFF, 0x30, 0x30, 0x01, 0x86, 0x30, 0xEF}
	if !bytes.Equal(mock.writes[1], wantPoll) {
		t.Errorf("second write = %X, want extended status request %X", mock.writes[1], wantPoll)
	}
}

func TestWaitIdle(t *testing.T) {
	busy := statusResponse("00200") // Panning (b9)
	idle := statusResponse("00000")
	mock := &mockSerialPort{responseQueue: [][]byte{busy, idle}}
	cam := newTestCamera(mock)

	if err := cam.WaitIdle(2 * time.Second); err != nil {
		t.Fatalf("WaitIdle: %v", err)
	}
	if len(mock.writes) != 2 {
		t.Errorf("expected 2 status polls, got %d", len(mock.writes))
	}
}

func TestReadResponseSkipsGarbage(t *testing.T) {
	frame := statusResponse("A00")
	mock := &mockSerialPort{readBuf: append([]byte{0x00, 0x99}, frame...)}
	cam := newTestCamera(mock)

	resp, err := cam.readResponse()
	if err != nil {
		t.Fatalf("readResponse: %v", err)
	}
	if resp.Type != ResponseStatus {
		t.Errorf("type = %v, want ResponseStatus", resp.Type)
	}
	if !bytes.Equal(resp.Data, frame) {
		t.Errorf("data = %X, want %X", resp.Data, frame)
	}
}

func TestReadResponseGarbageOnly(t *testing.T) {
	// Noise with no valid start byte must time out, not panic, once the
	// garbage-discard loop has stripped the buffer empty.
	mock := &mockSerialPort{readBuf: []byte{0x00, 0x99}}
	cam := newTestCamera(mock)

	_, err := cam.readResponse()
	if err == nil || !strings.Contains(err.Error(), "timeout") {
		t.Errorf("expected timeout error, got %v", err)
	}
}

func TestReadResponseTimeout(t *testing.T) {
	cam := newTestCamera(&mockSerialPort{})

	_, err := cam.readResponse()
	if err == nil || !strings.Contains(err.Error(), "timeout") {
		t.Errorf("expected timeout error, got %v", err)
	}
}
