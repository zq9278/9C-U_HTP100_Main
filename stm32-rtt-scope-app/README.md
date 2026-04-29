# STM32 RTT Scope Desktop

Standalone Electron desktop app for realtime STM32 variables over SEGGER RTT.

## Development

```powershell
cd stm32-rtt-scope-app
npm.cmd install
npm.cmd run dev
```

The app starts `JLinkRTTClient.exe` without command-line arguments. This matches SEGGER J-Link RTT Client versions that reject `-Device`, `-If`, and `-RTTChannel`.

## Firmware Protocol

Use one RTT output stream first:

```c
SEGGER_RTT_printf(0, "heater start\n");
SEGGER_RTT_printf(0, "$VAR,temp=38.5,pressure=42,pwm=70,state=HEATING\n");
```

Only frames beginning with `$VAR` are parsed as variables. All other RTT output, including ordinary `key=value` text, stays in the RTT Log panel.

The UI supports:

- Accent color selection
- Dark/light background switching
- Custom font size, background color, panel color, and text color
- Draggable left/right panel widths
- Draggable PID/RTT Log split
- Draggable chart card reordering
- Per-chart button and corner-drag resizing
- Deleting stale variables from the variable list and saved config

PID command format sent to RTT stdin:

```text
GET,pid_heat
SET,pid_heat,kp=1.20,ki=0.080,kd=0.010
SAVE,pid_heat,kp=1.20,ki=0.080,kd=0.010
```

## Build

```powershell
npm.cmd run build
npm.cmd run dist
```

The portable Windows executable is generated in `release/`.
