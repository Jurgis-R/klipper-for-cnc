# CNC edge probing and workpiece measurement helpers.
#
# Enable with a [cnc_probe] section and an existing [probe] section.
# All XY inputs and reported measurements use the active G-code work frame,
# including the G68 XY rotation in gcode_move.
import math

class CNCProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.last_result = None
        self.gcode.register_command('G38.2', self.cmd_G38_2,
                                    desc="Probe toward a target; error if no contact occurs.")
        self.gcode.register_command('CNC_FIND_WORKPIECE',
                                    self.cmd_CNC_FIND_WORKPIECE,
                                    desc="Probe four workpiece edges and report corners, centre and size.")

    def _objects(self, gcmd):
        gmove = self.printer.lookup_object('gcode_move')
        probe = self.printer.lookup_object('probe', None)
        if probe is None or not hasattr(probe, 'mcu_probe'):
            raise gcmd.error("CNC probing requires a configured [probe] section")
        if not self.printer.lookup_object('toolhead').get_kinematics():
            raise gcmd.error("CNC probing requires configured kinematics")
        if 'X' not in gmove.axis_map or 'Y' not in gmove.axis_map or 'Z' not in gmove.axis_map:
            raise gcmd.error("CNC probing requires configured X, Y and Z axes")
        return gmove, probe

    def _logical_to_physical(self, gmove, position):
        return gmove._get_move_position(position)

    def _physical_to_logical(self, gmove, position):
        result = list(position)
        rot = gmove.g68_rotation
        if rot is None:
            return result
        xi, yi = gmove.axis_map['X'], gmove.axis_map['Y']
        # Invert the same rotation + translation used for ordinary G0/G1 moves.
        dx = position[xi] - rot['center_x'] - rot['offset_x']
        dy = position[yi] - rot['center_y'] - rot['offset_y']
        c, sn = math.cos(rot['angle']), math.sin(rot['angle'])
        result[xi] = rot['center_x'] + c * dx + sn * dy
        result[yi] = rot['center_y'] - sn * dx + c * dy
        return result

    def _probe_target(self, gcmd, target, speed, require_trigger=True, axes=None):
        gmove, probe = self._objects(gcmd)
        physical_target = self._logical_to_physical(gmove, target)
        homing = self.printer.lookup_object('homing')
        probe_axes = [a.lower() for a in (axes or [])]
        # This uses Klipper's endstop-aware drip move and trigger-position
        # reconstruction; it does not approximate the contact point from the
        # commanded endpoint.
        trigpos = homing.probing_move(
            probe.mcu_probe, physical_target, speed,
            check_triggered=require_trigger, triggered=True,
            probe_axes=probe_axes)
        gmove.last_position = self._physical_to_logical(gmove, trigpos)
        return list(gmove.last_position)

    def cmd_G38_2(self, gcmd):
        gmove, probe = self._objects(gcmd)
        if not gmove.is_printer_ready:
            raise gcmd.error("G38.2 requires the printer to be ready")
        params = gcmd.get_command_parameters()
        axes = [a for a in ('X', 'Y', 'Z') if a in params]
        if not axes:
            raise gcmd.error("G38.2 requires at least one of X, Y or Z")
        target = list(gmove.last_position)
        try:
            for axis in axes:
                idx = gmove.axis_map[axis]
                value = float(params[axis])
                if not math.isfinite(value):
                    raise ValueError()
                if gmove.absolute_coord:
                    target[idx] = value + gmove.base_position[idx]
                else:
                    target[idx] += value
            if 'F' in params:
                feed = float(params['F'])
                if not math.isfinite(feed) or feed <= 0.:
                    raise ValueError()
                gmove.speed = feed * gmove.speed_factor
            speed = gmove.speed
            if speed <= 0.:
                raise ValueError()
        except (ValueError, TypeError):
            raise gcmd.error("G38.2 requires finite coordinates and a positive feedrate")
        self._probe_target(gcmd, target, speed, True, axes)
        gcmd.respond_info("G38.2: contact at X%.3f Y%.3f Z%.3f" % tuple(
            gmove.last_position[gmove.axis_map[a]] - gmove.base_position[gmove.axis_map[a]]
            for a in ('X', 'Y', 'Z')))

    def _move_work(self, gcmd, x, y, z, feed_mm_min):
        gmove, probe = self._objects(gcmd)
        old_absolute = gmove.absolute_coord
        try:
            gmove.absolute_coord = True
            params = {
                'X': str(x), 'Y': str(y), 'Z': str(z),
                'F': str(feed_mm_min)
            }
            move_cmd = self.gcode.create_gcode_command('G1', 'G1', params)
            gmove.cmd_G1(move_cmd)
        finally:
            gmove.absolute_coord = old_absolute

    def _retract_safe(self, gcmd, safe_z, feed_mm_min):
        gmove, probe = self._objects(gcmd)
        xi, yi, zi = (gmove.axis_map[a] for a in ('X', 'Y', 'Z'))
        x = gmove.last_position[xi] - gmove.base_position[xi]
        y = gmove.last_position[yi] - gmove.base_position[yi]
        self._move_work(gcmd, x, y, safe_z, feed_mm_min)

    def _probe_work(self, gcmd, axis, target_value, feed_mm_min):
        gmove, probe = self._objects(gcmd)
        idx = gmove.axis_map[axis]
        target = list(gmove.last_position)
        target[idx] = target_value + gmove.base_position[idx]
        return self._probe_target(gcmd, target, feed_mm_min / 60.0, True, [axis])

    def cmd_CNC_FIND_WORKPIECE(self, gcmd):
        gmove, probe = self._objects(gcmd)
        if not gmove.is_printer_ready:
            raise gcmd.error("CNC_FIND_WORKPIECE requires the printer to be ready")
        try:
            x_min = gcmd.get_float('X_MIN')
            x_max = gcmd.get_float('X_MAX')
            y_min = gcmd.get_float('Y_MIN')
            y_max = gcmd.get_float('Y_MAX')
            probe_z = gcmd.get_float('Z')
            safe_z = gcmd.get_float('SAFE_Z')
            feed = gcmd.get_float('F', 120.0, above=0.)
            if not all(math.isfinite(v) for v in
                       (x_min, x_max, y_min, y_max, probe_z, safe_z, feed)):
                raise ValueError()
            if x_min >= x_max or y_min >= y_max or safe_z <= probe_z:
                raise ValueError()
        except (ValueError, TypeError):
            raise gcmd.error("Use X_MIN<X_MAX, Y_MIN<Y_MAX, SAFE_Z>Z, and positive F")
        if gmove.g68_rotation is not None:
            # Probe directions are expressed in the active work frame; targets
            # are transformed to machine XY and contact positions transformed back.
            gcmd.respond_info("CNC_FIND_WORKPIECE: G68 rotation is active; probing in rotated work coordinates")
        xmid, ymid = (x_min + x_max) * 0.5, (y_min + y_max) * 0.5

        # Lift vertically first, then position at the outside search corner.
        # Travel between sides is routed around the outside at SAFE_Z, never
        # across the part at the side-probing height.
        xi, yi = gmove.axis_map['X'], gmove.axis_map['Y']
        current_x = gmove.last_position[xi] - gmove.base_position[xi]
        current_y = gmove.last_position[yi] - gmove.base_position[yi]
        self._move_work(gcmd, current_x, current_y, safe_z, feed)
        self._move_work(gcmd, x_min, y_min, safe_z, feed)
        self._move_work(gcmd, x_min, ymid, safe_z, feed)
        self._move_work(gcmd, x_min, ymid, probe_z, feed)
        hit_left = self._probe_work(gcmd, 'X', x_max, feed)
        # Retract vertically at the measured contact point, then route around
        # the lower side to the opposite edge.
        self._retract_safe(gcmd, safe_z, feed)
        self._move_work(gcmd, x_min, y_min, safe_z, feed)
        self._move_work(gcmd, x_max, y_min, safe_z, feed)
        self._move_work(gcmd, x_max, ymid, safe_z, feed)
        self._move_work(gcmd, x_max, ymid, probe_z, feed)
        hit_right = self._probe_work(gcmd, 'X', x_min, feed)
        # Retract vertically at the contact point before routing around the
        # right side to the far Y edge.
        self._retract_safe(gcmd, safe_z, feed)
        self._move_work(gcmd, x_max, y_max, safe_z, feed)
        self._move_work(gcmd, xmid, y_max, safe_z, feed)
        self._move_work(gcmd, xmid, y_max, probe_z, feed)
        hit_far_y = self._probe_work(gcmd, 'Y', y_min, feed)
        # Retract vertically at the contact point before routing around the
        # left side to the near Y edge.
        self._retract_safe(gcmd, safe_z, feed)
        self._move_work(gcmd, x_min, y_max, safe_z, feed)
        self._move_work(gcmd, x_min, y_min, safe_z, feed)
        self._move_work(gcmd, xmid, y_min, safe_z, feed)
        self._move_work(gcmd, xmid, y_min, probe_z, feed)
        hit_near_y = self._probe_work(gcmd, 'Y', y_max, feed)
        self._retract_safe(gcmd, safe_z, feed)

        # Convert trigger positions to work coordinates and compensate the
        # configured probe XY offset, rotated back into the active work frame.
        offsets = probe.get_offsets()
        ox, oy = offsets[0], offsets[1]
        rot = gmove.g68_rotation
        if rot is not None:
            c, sn = math.cos(rot['angle']), math.sin(rot['angle'])
            ox, oy = c * ox + sn * oy, -sn * ox + c * oy
        def work_xy(hit):
            return (hit[gmove.axis_map['X']] - gmove.base_position[gmove.axis_map['X']] + ox,
                    hit[gmove.axis_map['Y']] - gmove.base_position[gmove.axis_map['Y']] + oy)
        left_x = work_xy(hit_left)[0]
        right_x = work_xy(hit_right)[0]
        far_y = work_xy(hit_far_y)[1]
        near_y = work_xy(hit_near_y)[1]
        xmin, xmax = sorted((left_x, right_x))
        ymin, ymax = sorted((near_y, far_y))
        if xmax - xmin <= 0. or ymax - ymin <= 0.:
            raise gcmd.error("CNC_FIND_WORKPIECE returned invalid dimensions; check probe offsets and search bounds")
        cx, cy = (xmin + xmax) * 0.5, (ymin + ymax) * 0.5
        self.last_result = {
            'x_min': xmin, 'x_max': xmax, 'y_min': ymin, 'y_max': ymax,
            'center_x': cx, 'center_y': cy,
            'size_x': xmax - xmin, 'size_y': ymax - ymin,
        }
        gcmd.respond_info(
            "WORKPIECE bounds: X %.3f..%.3f Y %.3f..%.3f\n"
            "Size: X %.3f Y %.3f mm; centre: X %.3f Y %.3f\n"
            "Corners (active work coordinates): LL(%.3f, %.3f) LR(%.3f, %.3f) "
            "UR(%.3f, %.3f) UL(%.3f, %.3f)" %
            (xmin, xmax, ymin, ymax, xmax - xmin, ymax - ymin, cx, cy,
             xmin, ymin, xmax, ymin, xmax, ymax, xmin, ymax))

    def get_status(self, eventtime):
        return {'last_result': self.last_result}

def load_config(config):
    return CNCProbe(config)
