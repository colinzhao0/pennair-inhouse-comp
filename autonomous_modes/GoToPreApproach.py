from typing import Optional
import math
from .Mode import Mode
from rclpy.node import Node
from uav import UAV


class GoToPreApproach(Mode):
    """Fly to the next pre-approach waypoint from planned_route.

    While en route it will attempt to yaw the vehicle to face the hoop
    based on the stored hoop bearing (if present) so the camera points
    toward the hoop as it approaches.
    """

    def __init__(self, node: Node, uav: UAV, cruise_speed_mps: float = 2.0, waypoint_tol_m: float = 0.8):
        super().__init__(node, uav)
        self.cruise_speed_mps = float(cruise_speed_mps)
        self.waypoint_tol_m = float(waypoint_tol_m)
        self.target = None
        self.target_hoop = None

    def on_enter(self) -> None:
        route = getattr(self.uav, 'planned_route', []) or []
        if not route:
            self.node.get_logger().info("No planned route - completing GoToPreApproach")
            self.target = None
            return
        item = route[0]
        self.target = tuple(item['pre_approach'])
        self.target_hoop = item.get('hoop')
        self.node.get_logger().info(f"GoToPreApproach: target={self.target}")

    def on_update(self, time_delta: float) -> None:
        if not self.target:
            return
        # publish position setpoint toward target
        try:
            # yaw toward hoop if bearing available
            if isinstance(self.target_hoop, dict) and 'bearing' in self.target_hoop:
                bearing = self.target_hoop['bearing']
                try:
                    # publish a position setpoint and yaw (if method exists)
                    self.uav.publish_position_setpoint(self.target, yaw=bearing)
                except Exception:
                    self.uav.publish_position_setpoint(self.target)
            else:
                self.uav.publish_position_setpoint(self.target)
        except Exception:
            pass

    def check_status(self) -> str:
        if not self.target:
            return "complete"
        pos = getattr(self.uav, 'local_position', None)
        if not pos:
            return "continue"
        cur = (pos.x, pos.y, pos.z)
        if math.dist(cur, self.target) <= self.waypoint_tol_m:
            self.node.get_logger().info("Reached pre-approach waypoint")
            return "complete"
        return "continue"
