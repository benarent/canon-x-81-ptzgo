Canon XU-80/XU-81 PTZ Camera Controller
=======================================

A Go-based command-line interface for controlling Canon XU-80 and XU-81 PTZ (Pan-Tilt-Zoom) cameras via RS-232C/RS-422 serial communication.

Features
--------

-   **Full PTZ Control**: Pan, tilt, and zoom with variable speeds
-   **Focus Management**: Auto/manual focus modes with near/far control
-   **Preset Positions**: Store and recall up to 128 camera positions
-   **Camera Settings**: White balance, shooting modes, filters, and more
-   **Interactive Shell**: Command-line interface with auto-completion and history
-   **Cross-Platform**: Works on Linux, macOS, and Windows

Hardware Requirements
---------------------

-   Canon XU-80 or XU-81 PTZ camera
-   RS-232C or RS-422 serial connection
-   USB-to-serial adapter (if needed)

Installation
------------

bash

```
git clone https://github.com/yourusername/canon-ptz-controller
cd canon-ptz-controller
go build -o ptz-control
```

Configuration
-------------

Create a `config.json` file (optional):

json

```
{
  "port": "/dev/ttyUSB0",
  "baud_rate": 9600,
  "read_timeout": 300000000,
  "power_on_delay": 15000000000,
  "default_pan_speed": 2000,
  "default_tilt_speed": 2000,
  "camera_model": 1,
  "digital_zoom_mode": 0
}
```

-   `camera_model`: 0 for XU-80, 1 for XU-81
-   `digital_zoom_mode`: 0 for Optical Interlocking, 1 for Independent (XU-81 only)

Usage
-----

### Interactive Mode

bash

```
./ptz-control
```

This starts an interactive shell with command auto-completion and history.

### Command-Line Mode

bash

```
./ptz-control <command> [arguments]
```

Example:

bash

```
./ptz-control on              # Power on camera
./ptz-control goto 1000 500    # Move to position
```

### Configuration File

bash

```
./ptz-control -config myconfig.json
```

Command Reference
-----------------

### Power Control

| Command | Description |
| --- | --- |
| `on` | Power on camera (15s boot time) |
| `off` | Power off camera |

### Movement Control

| Command | Description |
| --- | --- |
| `stop` | Stop all movement |
| `left` | Start panning left |
| `right` | Start panning right |
| `up` | Start tilting up |
| `down` | Start tilting down |
| `goto <pan> <tilt>` | Move to absolute position |
| `getpos` | Get current pan/tilt position |

**Position Ranges:**

-   Pan: -9000 to +9000 (±180°)
-   Tilt: -6500 to +6500 (±130°)

### Speed Control

| Command | Description |
| --- | --- |
| `panspeed <speed>` | Set pan speed (5-3000) |
| `tiltspeed <speed>` | Set tilt speed (5-3000) |

**Speed Reference:**

-   5 = 0.1°/sec (slowest)
-   2000 = 40°/sec (standard max)
-   3000 = 60°/sec (high-speed mode)

### Zoom Control

| Command | Description |
| --- | --- |
| `zoomstop` | Stop zoom movement |
| `zoomwide` | Zoom to wide angle |
| `zoomtele` | Zoom to telephoto |
| `zoom <position>` | Set zoom position (0-238) |

**Zoom Positions:**

-   0-128: Optical zoom (1x-20x)
-   129-238: Digital zoom (20x-240x, XU-81 only)

### Focus Control

| Command | Description |
| --- | --- |
| `focusauto` | Enable autofocus |
| `focusmanual` | Enable manual focus |
| `focusnear` | Move focus near (manual mode) |
| `focusfar` | Move focus far (manual mode) |
| `focusinf` | Set focus to infinity |
| `focusoneshot` | One-shot autofocus (~7s) |

**Important:** `focusnear` and `focusfar` require manual focus mode to be active first.

### Preset Management

| Command | Description |
| --- | --- |
| `record <slot>` | Save position to slot (0-127) |
| `recall <slot> [time]` | Move to preset (optional time 0-48s) |

### White Balance

| Command | Description |
| --- | --- |
| `wbauto` | Auto white balance |
| `wbpreset` | Preset white balance |
| `wbindoor` | Indoor setting (3200K) |
| `wboutdoor` | Outdoor setting (5800K) |
| `wbatw` | Auto tracking white balance |

### Shooting Modes

| Command | Description |
| --- | --- |
| `showauto` | Auto mode |
| `showmanual` | Manual mode |
| `shownight` | Night mode |

### Filters & Effects

| Command | Description |
| --- | --- |
| `wdr on/off` | Wide Dynamic Range |
| `nd on/off` | Neutral Density filter |
| `ir on/off` | Infrared filter |
| `colorbar on/off` | Color bar test pattern |

### System Commands

| Command | Description |
| --- | --- |
| `status` | Show camera status |
| `extstatus` | Extended status info |
| `info` | System information |
| `inithome` | Initialize to home position |
| `initoriginal` | Initialize to last position |

### Maintenance (Outdoor Models)

| Command | Description |
| --- | --- |
| `wiper` | Run wiper once |
| `washer` | Auto wash cycle (4 wipes) |
| `washeron` | Manual washer on |
| `washeroff` | Manual washer off |

Examples
--------

### Basic Operation

bash

```
# Power on and check status
> on
> status

# Move camera
> right          # Start panning
> stop           # Stop movement
> goto 4500 1000 # Go to specific position

# Zoom operations
> zoom 64        # Mid-zoom position
> zoomtele       # Zoom in
> zoomstop       # Stop zooming
```

### Focus Control

bash

```
# Manual focus control
> focusmanual    # Enable manual mode first
> focusfar       # Adjust focus
> focusnear
> focusauto      # Return to autofocus
```

### Using Presets

bash

```
# Save current position
> record 1

# Move somewhere else
> goto -4500 -2000

# Return to saved position
> recall 1       # Fast return
> recall 1 10    # Smooth 10-second transition
```

### Debugging

bash

```
# Debug command communication
> debug status           # Show raw bytes
> debug focus_manual 31  # Send specific hex parameters
```

Technical Details
-----------------

### Communication Protocol

-   **Interface**: RS-232C or RS-422
-   **Baud Rate**: 9600 bps
-   **Data Format**: 8 bits, no parity, 1 stop bit
-   **Protocol**: Half-duplex with command/response

### Command Format

```
[Header][Device][Command][Parameters][End Mark]
FF      3030    XXXX     [params]    EF
```

### Response Format

```
[Header][Device][Error][Status/Data][End Mark]
FE      3030    XXXX   [data]       EF
```

### Status Bits

The camera status command returns a bit field:

-   Bit 0: Focusing
-   Bit 1: Manual Focus Mode
-   Bit 5: Camera Power OFF (inverted)
-   Bit 7: Zooming
-   Bit 8: Pan Limit Reached
-   Bit 9: Panning
-   Bit 10: Tilt Limit Reached
-   Bit 11: Tilting

Troubleshooting
---------------

### Camera Not Responding

1.  Check serial port permissions: `ls -l /dev/ttyUSB*`
2.  Verify correct port in config or use `-config`
3.  Ensure camera is powered and connected
4.  Try a lower baud rate if using long cables

### Focus Commands Not Working

Focus near/far require manual focus mode:

bash

```
> focusmanual   # Enable manual mode first
> focusfar      # Now this will work
```

### Permission Denied on Linux

bash

```
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Finding Serial Port

-   Linux: `/dev/ttyUSB0` or `/dev/ttyS0`
-   macOS: `/dev/tty.usbserial-*`
-   Windows: `COM3`, `COM4`, etc.

Limitations
-----------

-   Pan/Tilt durability: 300,000 cycles (avoid continuous operation)
-   Digital zoom only available on XU-81 model
-   Some features require outdoor option (wiper, washer, ND filter)
-   Commands cannot be queued; wait for completion

Contributing
------------

Pull requests welcome! Please:

1.  Follow Go coding standards
2.  Add tests for new features
3.  Update documentation
4.  Test with actual hardware if possible

License
-------

MIT 

Acknowledgments
---------------

Based on Canon XU-80/XU-81 Programmer's Manual Ver. 02.02

RJ-45

- 2 - Organge Ground - 2
- 3 - green-white stripe = RXD+
- 4 - Blue - TXD- 
- 5 - Blue/White - TXD+
- 6 - Green - RXD-
- 7 - Brown/White - GND 