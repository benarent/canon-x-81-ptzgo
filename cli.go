package main

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/peterh/liner"
)

// CLI command structure
type CLICommand struct {
	Name        string
	Usage       string
	MinArgs     int
	MaxArgs     int
	Handler     func(cam *XU81Camera, args []string) error
	Description string
}

// jsonOutput switches status/position commands to JSON (set by the -json flag)
var jsonOutput bool

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

	"drive": {
		Name: "drive", Usage: "drive <pan> <tilt> [zoom [focus]]", MinArgs: 2, MaxArgs: 4,
		Handler: func(cam *XU81Camera, args []string) error {
			speeds := make([]int, 4)
			for i, arg := range args {
				v, err := strconv.Atoi(arg)
				if err != nil {
					return fmt.Errorf("invalid speed %q: %w", arg, err)
				}
				speeds[i] = v
			}
			return cam.Drive(speeds[0], speeds[1], speeds[2], speeds[3])
		},
		Description: "Velocity-drive all axes (-64..63 each, 0=stop): +pan=right +tilt=up +zoom=tele +focus=near",
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
			if jsonOutput {
				return json.NewEncoder(os.Stdout).Encode(map[string]int{"pan": pan, "tilt": tilt})
			}
			fmt.Printf("Position: Pan=%d, Tilt=%d\n", pan, tilt)
			return nil
		},
		Description: "Get current pan/tilt position",
	},
	"getzoom": {
		Name: "getzoom", Usage: "getzoom", MinArgs: 0, MaxArgs: 0,
		Handler: func(cam *XU81Camera, args []string) error {
			pos, err := cam.GetZoomPosition()
			if err != nil {
				return err
			}
			if jsonOutput {
				return json.NewEncoder(os.Stdout).Encode(map[string]int{"zoom": pos})
			}
			fmt.Printf("Zoom position: %d\n", pos)
			return nil
		},
		Description: "Get current zoom position",
	},
	"getspeed": {
		Name: "getspeed", Usage: "getspeed", MinArgs: 0, MaxArgs: 0,
		Handler: func(cam *XU81Camera, args []string) error {
			panSpeed, err := cam.GetPanSpeed()
			if err != nil {
				return err
			}
			tiltSpeed, err := cam.GetTiltSpeed()
			if err != nil {
				return err
			}
			if jsonOutput {
				return json.NewEncoder(os.Stdout).Encode(map[string]int{"pan_speed": panSpeed, "tilt_speed": tiltSpeed})
			}
			fmt.Printf("Speed: Pan=%d, Tilt=%d\n", panSpeed, tiltSpeed)
			return nil
		},
		Description: "Get current pan/tilt speeds",
	},
	"wait": {
		Name: "wait", Usage: "wait [seconds]", MinArgs: 0, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			timeout := 30 * time.Second
			if len(args) > 0 {
				secs, err := strconv.Atoi(args[0])
				if err != nil || secs <= 0 {
					return fmt.Errorf("invalid timeout: %s", args[0])
				}
				timeout = time.Duration(secs) * time.Second
			}
			return cam.WaitIdle(timeout)
		},
		Description: "Wait until camera motion and busy flags clear (default timeout 30s)",
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
	// Exposure commands
	"shutter": {
		Name: "shutter", Usage: "shutter <0-4>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			val, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid shutter value: %w", err)
			}
			return cam.SetShutterSpeed(val)
		},
		Description: "Set shutter speed: 0=2s 1=1/15 2=1/60 3=1/100 4=1/1000 (manual/shutter-priority mode)",
	},
	"gain": {
		Name: "gain", Usage: "gain <0-5>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			val, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid gain value: %w", err)
			}
			return cam.SetGain(val)
		},
		Description: "Set gain: 0=0dB 1=6dB 2=12dB 3=18dB 4=24dB 5=max (manual mode)",
	},
	"iris": {
		Name: "iris", Usage: "iris <0-255>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			val, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid iris value: %w", err)
			}
			return cam.SetIris(val)
		},
		Description: "Set iris position 0-255, higher=brighter (manual/iris-priority mode)",
	},
	"digitalzoom": {
		Name: "digitalzoom", Usage: "digitalzoom <0-110>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			val, err := strconv.Atoi(args[0])
			if err != nil {
				return fmt.Errorf("invalid digital zoom value: %w", err)
			}
			return cam.SetDigitalZoom(val)
		},
		Description: "Set digital zoom, (magnification-1.0)*10: 0=1.0x 110=12.0x (XU-81 independent mode)",
	},
	"autoir": {
		Name: "autoir", Usage: "autoir <off|slow|gain>", MinArgs: 1, MaxArgs: 1,
		Handler: func(cam *XU81Camera, args []string) error {
			switch args[0] {
			case "off":
				return cam.SetAutoIR(0)
			case "slow":
				return cam.SetAutoIR(1)
			case "gain":
				return cam.SetAutoIR(2)
			default:
				return fmt.Errorf("invalid argument: use 'off', 'slow', or 'gain'")
			}
		},
		Description: "Auto IR filter: off, slow (slow shutter), gain (night mode only)",
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
			if jsonOutput {
				return json.NewEncoder(os.Stdout).Encode(status)
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
			if jsonOutput {
				return json.NewEncoder(os.Stdout).Encode(status)
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
			if jsonOutput {
				return json.NewEncoder(os.Stdout).Encode(info)
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
	"showshutter": {
		Name: "showshutter", Usage: "showshutter", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetShootingMode(ShootingShutterPriority) },
		Description: "Set shooting mode to shutter priority",
	},
	"showiris": {
		Name: "showiris", Usage: "showiris", MinArgs: 0, MaxArgs: 0,
		Handler:     func(cam *XU81Camera, args []string) error { return cam.SetShootingMode(ShootingIrisPriority) },
		Description: "Set shooting mode to iris priority",
	},
}

func runCommand(cam *XU81Camera, name string, args []string) error {
	cmd, ok := cliCommands[name]
	if !ok {
		return fmt.Errorf("unknown command %q", name)
	}
	if len(args) < cmd.MinArgs || len(args) > cmd.MaxArgs {
		return fmt.Errorf("usage: %s", cmd.Usage)
	}
	return cmd.Handler(cam, args)
}

// runInteractive runs the shell loop until quit/Ctrl-D, stopping the head
// and saving history on the way out.
func runInteractive(cam *XU81Camera) int {
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

	historyFile := ".ptz-control_history"
	if home, err := os.UserHomeDir(); err == nil {
		historyFile = filepath.Join(home, historyFile)
	}
	if f, err := os.Open(historyFile); err == nil {
		shell.ReadHistory(f)
		f.Close()
	}

	saveHistory := func() {
		if f, err := os.Create(historyFile); err == nil {
			shell.WriteHistory(f)
			f.Close()
		}
	}
	registerCleanup(func() {
		// Restore the terminal from raw mode before touching the filesystem.
		shell.Close()
		saveHistory()
	})

	fmt.Println("Interactive mode — type \"help\" for commands, Ctrl-D to quit.")
	for {
		input, err := shell.Prompt("> ")
		if err == liner.ErrPromptAborted {
			fmt.Println()
			continue // ^C abandons the current line
		}
		if err != nil {
			fmt.Println()
			break // Ctrl-D (io.EOF) or a terminal error quits
		}
		input = strings.TrimSpace(input)
		if input == "" {
			continue
		}
		shell.AppendHistory(input)

		if input == "help" {
			names := make([]string, 0, len(cliCommands))
			for name := range cliCommands {
				names = append(names, name)
			}
			sort.Strings(names)
			for _, name := range names {
				fmt.Printf("  %-12s %s\n", name, cliCommands[name].Description)
			}
			continue
		}
		if input == "quit" || input == "exit" {
			break
		}

		tokens := strings.Fields(input)
		if err := runCommand(cam, tokens[0], tokens[1:]); err != nil {
			fmt.Printf("error: %v\n", err)
		}
	}

	cam.BestEffortStop()
	saveHistory()
	return 0
}
