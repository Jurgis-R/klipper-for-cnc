# CNC workpiece edge probing

This fork adds optional G-code probing helpers in `klippy/extras/cnc_probe.py`.
They require a configured Klipper `[probe]` whose endstop is suitable for
side contact probing, plus this section:

```ini
[cnc_probe]
```

Do not enable this on a machine unless the configured probe/tool can safely
contact the sides of the workpiece. Start with low probing feed and generous
clearance, and verify travel with the tool clear of the part.

## G38.2

```gcode
G38.2 X<target> [Y<target>] [Z<target>] F<mm/min>
```

Moves toward the target and stops at probe contact. It errors if no contact
occurs. At contact, the current G-code position is synchronized to the
measured trigger position. With G68 active, XY targets are transformed into
machine coordinates and the trigger position is transformed back to the
active work coordinate system. G28 remains a machine-coordinate operation.

## CNC_FIND_WORKPIECE

```gcode
CNC_FIND_WORKPIECE X_MIN=<outside-left> X_MAX=<outside-right> \
    Y_MIN=<outside-near> Y_MAX=<outside-far> Z=<side-probe-height> \
    SAFE_Z=<clearance-height> F=<mm/min>
```

The supplied XY rectangle must fully surround the workpiece, with X_MIN and
Y_MIN outside the part and X_MAX and Y_MAX outside the opposite sides. SAFE_Z
must be high enough to travel around the part without collision and must be
greater than Z. Z is the height at which the probe contacts the vertical sides.
The command probes all four sides, retracts vertically at each contact, routes
around the outside at SAFE_Z, and reports the four axis-aligned corners, XY
centre and dimensions. It compensates the configured probe XY offset.

All XY values and reported bounds use the active G-code work frame. When G68
is active, the search paths are rotated with it and the result is reported in
that rotated frame. This command does **not** discover a part's rotation angle
by itself: set G68 to the desired angle first if the workpiece is not aligned
to the unrotated work axes. The search rectangle must surround the complete
workpiece in the active work frame.

The command does not automatically change G68, G92, tool offsets, or machine
limits. It reports measured geometry only. Test the probing path and offset
sign on a non-cutting setup before relying on the result.

## Automatic angle detection from multiple edge touches

Use `CNC_FIND_ROTATION` to probe multiple points along one approximately straight
edge. It fits a least-squares line, reports every measured point, the fitted edge
angle and fit RMS error, and can apply G68 automatically.

```gcode
G69
CNC_FIND_ROTATION SIDE=Y_MIN X_START=20 X_END=220 OUTSIDE=10 Z=5 SAFE_Z=20 STEP=20 PROBE_DEPTH=5 F=60 APPLY=1
```

This example samples an edge at X=20, 40, ... 220 mm and probes from Y=10 mm
toward +Y for at most 5 mm at each station. It therefore needs the selected
Y_MIN edge to be just inside that small probe-depth window. For `SIDE=Y_MAX`,
the probe moves toward -Y; for `SIDE=X_MIN`, it moves toward +X; for
`SIDE=X_MAX`, it moves toward -X. For X sides, use `Y_START` and `Y_END`
instead of X_START/X_END. Use at least three samples; STEP is the spacing in mm.
The routine is limited to 100 samples per run.

If `APPLY=1` (the default), it automatically issues `G68 R=<calculated angle>`.
You can specify `CENTER_X=<x> CENTER_Y=<y>` to choose the G68 rotation centre;
if omitted, the sample points' mean position is used. The fitted angle for a
Y-side is the edge angle relative to machine X; for an X-side the command
subtracts 90 degrees so that edge aligns with work Y.

The command requires G68 to be inactive at the start (`G69` first), because
the fit is performed in the unrotated machine XY frame. The resulting G68 is
then enabled automatically if APPLY=1. The reported `fit RMS` is the residual
distance from the fitted line; a large value suggests a damaged, curved, dirty,
or incorrectly sampled edge. Inspect the reported points and RMS before cutting.

The latest measurement is exposed by the Klipper object status for
`cnc_probe.last_result` (including the points, angle, centre and fit RMS), so
Moonraker clients can retrieve it without parsing console text. The console
also prints the same values. A side touch is only safe if the probe/tool is
designed for it and the supplied OUTSIDE/PROBE_DEPTH values bound the search;
a failed sample may still move the full PROBE_DEPTH without triggering.


## Runtime-editable wrapper macros

For adjustable CNC wrappers that can be edited and reloaded without restarting Klipper, see [CNC_Dynamic_Macros.md](CNC_Dynamic_Macros.md) and [config/cnc_dynamic_macros.cfg](../config/cnc_dynamic_macros.cfg). These use the optional third-party DynamicMacros add-on. The in-memory probing result survives a macro-only reload, but not a Klipper restart.
