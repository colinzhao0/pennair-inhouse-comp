from typing import Optional, Tuple
import math
from .Mode import Mode
from rclpy.node import Node
from uav import UAV


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class CenterInImage(Mode):
    """Image-based visual servoing that centers detected hoop in the image.

    This mode expects the UAV to provide a `get_latest_vision_detections`
    callable that returns detections with pixel error (ex,ey) and radius.
    """

    def __init__(self, node: Node, uav: UAV,
                 kx: float = 0.002, ky: float = 0.002, kz: float = 0.5, kyaw: float = 0.01,
                 vmax_xy: float = 1.0, vmax_z: float = 0.5, vmax_yaw_dps: float = 30.0,
                 hoop_center_tol_px: float = 8.0, hoop_radius_target_px: float = 80.0,
                 timeout_s: float = 12.0):
        super().__init__(node, uav)
        self.kx = kx
        self.ky = ky
        self.kz = kz
        self.kyaw = kyaw
        self.vmax_xy = vmax_xy
        self.vmax_z = vmax_z
        self.vmax_yaw = math.radians(vmax_yaw_dps)
        self.hoop_center_tol_px = hoop_center_tol_px
        self.hoop_radius_target_px = hoop_radius_target_px
        self.timeout_s = timeout_s
        self.start_time = None

    def on_enter(self) -> None:
        self.node.get_logger().info("CenterInImage: starting visual servoing")
        self.start_time = self.node.get_clock().now().nanoseconds / 1e9 if hasattr(self.node, 'get_clock') else None

    def on_update(self, time_delta: float) -> None:
        fn = getattr(self.uav, 'get_latest_vision_detections', None)
        if not fn:
            return
        try:
            dets = fn() or []
        except Exception:
            dets = []
        if not dets:
            return
        d = dets[0]
        ex = d.get('ex', 0.0)
        ey = d.get('ey', 0.0)
        radius = d.get('radius', d.get('r', 0.0))

        vx = clamp(-self.kx * ex, -self.vmax_xy, self.vmax_xy)
        vy = clamp(self.ky * ey, -self.vmax_xy, self.vmax_xy)
        cur_z = getattr(getattr(self.uav, 'local_position', None), 'z', 0.0)
        z_target = cur_z
        vz = clamp(self.kz * (z_target - cur_z), -self.vmax_z, self.vmax_z)
        yaw_rate = clamp(self.kyaw * ex, -self.vmax_yaw, self.vmax_yaw)

        try:
            self.uav.publish_velocity_setpoint((vx, vy, vz, yaw_rate))
        except Exception:
            try:
                self.uav.set_velocity((vx, vy, vz))
            except Exception:
                pass

    def check_status(self) -> str:
        fn = getattr(self.uav, 'get_latest_vision_detections', None)
        dets = []
        if fn:
            try:
                dets = fn() or []
            except Exception:
                dets = []
        if not dets:
            return "continue"
        d = dets[0]
        ex = abs(d.get('ex', 0.0))
        ey = abs(d.get('ey', 0.0))
        radius = d.get('radius', d.get('r', 0.0))
        if ex <= self.hoop_center_tol_px and ey <= self.hoop_center_tol_px and abs(radius - self.hoop_radius_target_px) <= (0.15 * self.hoop_radius_target_px):
            self.node.get_logger().info("CenterInImage: centered and at traverse distance")
            return "complete"
        if self.start_time is not None:
            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - self.start_time > self.timeout_s:
                self.node.get_logger().info("CenterInImage: timeout - reobserve")
                return "complete"
        return "continue"
