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
