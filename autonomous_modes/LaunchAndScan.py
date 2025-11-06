from typing import List, Any
import math
from time import time
from .Mode import Mode
from rclpy.node import Node
from uav import UAV



class LaunchAndScan(Mode):
    """Arm, takeoff and perform a yaw scan collecting hoop detections.

    This implementation mirrors the mock behaviour but lives in the
    `uav.autonomous_modes` package so it can be used in mission runs and
    detected by the launch helper.
    """

    def __init__(self, node: Node, uav: UAV,
                 takeoff_alt_m: float = 3.0,
                 spin_max_deg: float = 360.0,
                 spin_rate_dps: float = 45.0,
                 min_hoops_to_stop: int = 3,
                 scan_step_deg: float = 10.0):
        super().__init__(node, uav)
        self.takeoff_alt_m = float(takeoff_alt_m)
        self.spin_max_deg = float(spin_max_deg)
        self.spin_rate_dps = float(spin_rate_dps)
        self.min_hoops_to_stop = int(min_hoops_to_stop)
        self.scan_step_deg = float(scan_step_deg)

        self.collected = []
        self.start_yaw = None
        self.scanned_deg = 0.0
        self.last_time = None
        self.home_pose = None
        self.started = False
        self.vision_node = None

    def on_enter(self) -> None:
        self.node.get_logger().info("LaunchAndScan: arming and preparing to takeoff")
        try:
            self.uav.publish_offboard_control_heartbeat_signal()
        except Exception:
            pass

        try:
            self.uav.arm()
        except Exception:
            self.node.get_logger().warning("uav.arm() not available - continuing")

        try:
            self.uav.takeoff(self.takeoff_alt_m)
        except Exception:
            try:
                self.uav.publish_position_setpoint((0.0, 0.0, -abs(self.takeoff_alt_m)))
            except Exception:
                self.node.get_logger().warning("No takeoff method - ensure vehicle is at altitude")

        self.last_time = time()
        self.scanned_deg = 0.0
        self.collected = []
        # save home pose if available
        pos = getattr(self.uav, 'local_position', None)
        if pos:
            try:
                self.home_pose = (pos.x, pos.y, pos.z)
                self.node.get_logger().info(f"Saved home pose from uav.local_position: {self.home_pose}")
            except Exception:
                self.home_pose = None
        else:
            self.home_pose = None

        # Note: vision processing is expected to be provided externally
        # (e.g., CameraNode + a vision pipeline or a uav.vision_nodes.* node).
        # We do not instantiate a mock vision node here by default. The
        # mode will attempt to use `self.uav.get_latest_vision_detections()`
        # if the UAV object provides it, or rely on any vision service
        # clients created by the ModeManager.
        self.vision_node = None

    def on_update(self, time_delta: float) -> None:
        # spin by commanding yaw rate
        yaw_rate_rad = math.radians(self.spin_rate_dps)
        try:
            self.uav.publish_velocity_setpoint((0.0, 0.0, 0.0, yaw_rate_rad))
        except Exception:
            try:
                self.uav.set_yaw_rate(yaw_rate_rad)
            except Exception:
                pass

        # collect detections from mock vision node if available, else fall back to uav accessor
        detections: List[Any] = []
        if self.vision_node is not None:
            try:
                dets = self.vision_node.get_detections() or []
                for d in dets:
                    if isinstance(d, (list, tuple)) and len(d) == 3:
                        det_dict = {'position': (float(d[0]), float(d[1]), float(d[2]))}
                        if det_dict not in self.collected:
                            self.collected.append(det_dict)
            except Exception:
                pass
        else:
            try:
                fn = getattr(self.uav, 'get_latest_vision_detections', None)
                if fn:
                    dets = fn() or []
                    for d in dets:
                        if isinstance(d, dict):
                            if d not in self.collected:
                                self.collected.append(d)
            except Exception:
                pass

        try:
            setattr(self.uav, 'hoops_discovered', self.collected)
        except Exception:
            pass

        now = time()
        dt = now - (self.last_time or now)
        self.last_time = now
        self.scanned_deg += abs(self.spin_rate_dps) * dt

    def check_status(self) -> str:
        if len(self.collected) >= self.min_hoops_to_stop:
            self.node.get_logger().info(f"Found {len(self.collected)} hoops - finishing scan")
            if not getattr(self.uav, 'home_pose', None) and self.home_pose:
                try:
                    self.uav.home_pose = self.home_pose
                except Exception:
                    pass
            if self.vision_node is not None:
                try:
                    self.vision_node.destroy_node()
                except Exception:
                    pass
                self.vision_node = None
            return "complete"
        if self.scanned_deg >= self.spin_max_deg:
            self.node.get_logger().info("Scan exhausted - finishing")
            if self.vision_node is not None:
                try:
                    self.vision_node.destroy_node()
                except Exception:
                    pass
                self.vision_node = None
            return "complete"
        return "continue"

    def on_exit(self) -> None:
        if self.vision_node is not None:
            try:
                self.vision_node.destroy_node()
            except Exception:
                pass
            self.vision_node = None
