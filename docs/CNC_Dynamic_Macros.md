# Runtime-editable CNC macros

This fork's probing engine is implemented in `klippy/extras/cnc_probe.py`. For editable CNC wrapper macros, this project uses the third-party [3DCoded/DynamicMacros](https://github.com/3DCoded/DynamicMacros) project rather than modifying Klipper's standard macro loader. DynamicMacros is an optional external add-on; it is not bundled into this repository.

## What can change without restarting

- Edit `cnc_dynamic_macros.cfg` while Klipper is running, then issue `DYNAMIC_MACRO` to reload the file. This reloads macros without restarting Klipper.
- Change macro defaults live with `SET_DYNAMIC_VARIABLE`.
- The CNC Python object's `cnc_probe.last_result` remains in memory when only dynamic macros are reloaded. It is not cleared by editing/reloading the macro file.
- A Klipper restart still resets in-memory probe results. Persisting a measurement across a restart would require a separate save-to-disk feature.

**Important:** do not edit a macro while it is executing a probing/motion sequence. Edit/reload only when the machine is idle and the tool is clear.

## Setup

1. Install DynamicMacros using its official instructions: <https://docs.3dcoded.xyz/DynamicMacros/tutorial>. It is a third-party add-on and requires an initial Klipper restart to install/register the extension.
2. Copy `config/cnc_dynamic_macros.cfg` from this repository into the same directory as your `printer.cfg` (usually `~/printer_data/config/`).
3. Add this section to `printer.cfg`:

```ini
[dynamicmacros]
configs: cnc_dynamic_macros.cfg
```

4. Add/enable `[cnc_probe]` in the main Klipper configuration, and ensure the configured `[probe]` is suitable for safe side probing.
5. Restart Klipper once after initial installation/configuration. Later macro text edits do not require a restart.

If your DynamicMacros installation already has a `[dynamicmacros]` section, add `cnc_dynamic_macros.cfg` to its existing `configs:` list rather than adding a second section.

## Use

Measure without applying rotation first:

```gcode
G69
CNC_ROTATION SIDE=Y_MIN X_START=20 X_END=220 OUTSIDE=10 Z=5 SAFE_Z=20 STEP=20 PROBE_DEPTH=5 F=60 APPLY=0
```

Inspect the console's measured points, edge angle, G68 rotation and fit RMS. Apply only after verifying the result:

```gcode
G68 R=<verified-angle> X=<pivot-x> Y=<pivot-y>
```

For X-side edges, use `SIDE=X_MIN` or `SIDE=X_MAX`; the wrapper uses `Y_START` and `Y_END`. To override any defaults for one run, pass parameters to `CNC_ROTATION`.

## Change parameters live (no restart)

`SET_DYNAMIC_VARIABLE` changes a dynamic macro variable in memory. Examples:

```gcode
SET_DYNAMIC_VARIABLE MACRO=CNC_ROTATION VARIABLE=step VALUE=10.0
SET_DYNAMIC_VARIABLE MACRO=CNC_ROTATION VARIABLE=feed VALUE=30.0
SET_DYNAMIC_VARIABLE MACRO=CNC_ROTATION VARIABLE=probe_depth VALUE=3.0
SET_DYNAMIC_VARIABLE MACRO=CNC_ROTATION VARIABLE=apply VALUE=0
```

Change the default edge and sample range:

```gcode
SET_DYNAMIC_VARIABLE MACRO=CNC_ROTATION VARIABLE=side VALUE='"Y_MIN"'
SET_DYNAMIC_VARIABLE MACRO=CNC_ROTATION VARIABLE=x_start VALUE=20.0
SET_DYNAMIC_VARIABLE MACRO=CNC_ROTATION VARIABLE=x_end VALUE=220.0
```

These runtime variable edits last only until Klipper restarts or the macro is reloaded from its file. For persistent defaults, edit `cnc_dynamic_macros.cfg` and reload it with:

```gcode
DYNAMIC_MACRO
```

This reload reads all configured dynamic macro files; check the DynamicMacros documentation for the exact behavior of the installed version. Macro source edits do not themselves preserve probe measurements across a Klipper restart.

## Safety

- The wrapper defaults to `APPLY=0`: it measures without automatically applying G68.
- Test with the tool clear and use a low feed. Set `OUTSIDE`, `Z`, `SAFE_Z` and `PROBE_DEPTH` for the actual machine and probe.
- `CNC_FIND_ROTATION` still requires `G69` before measurement.
- DynamicMacros is third-party software. Review its source and compatibility with your Klipper/Kalico version before installing.
- This configuration is a convenience wrapper, not evidence that probing geometry has been validated on a real CNC machine.
