import math
from .Mode import Mode
from rclpy.node import Node
from uav import UAV


class CommitTraverse(Mode):
    """Fly a short segment through the hoop along its normal.

    This creates a small straight-line segment through the
    hoop and commands position setpoints until the end point is reached.
    """

    def __init__(self, node: Node, uav: UAV, traverse_dist: float = 4.0, traverse_speed: float = 2.0):
        super().__init__(node, uav)
        self.traverse_dist = float(traverse_dist)
        self.traverse_speed = float(traverse_speed)
        self.start = None
        self.end = None

    def on_enter(self) -> None:
        route = getattr(self.uav, 'planned_route', []) or []
        if not route:
            self.node.get_logger().info("CommitTraverse: no target - completing")
            return
        hoop_pos = tuple(route[0]['hoop_pos'])
        hoop = route[0]['hoop']
        normal = (0.0, 1.0, 0.0)
        if isinstance(hoop, dict) and 'bearing' in hoop:
            b = hoop['bearing']
            normal = (math.cos(b), math.sin(b), 0.0)

        hx, hy, hz = hoop_pos
        self.start = (hx - normal[0] * 1.0, hy - normal[1] * 1.0, hz)
        self.end = (hx + normal[0] * self.traverse_dist, hy + normal[1] * self.traverse_dist, hz)
        self.node.get_logger().info(f"CommitTraverse: start={self.start} end={self.end}")

    def on_update(self, time_delta: float) -> None:
        if not self.end:
            return
        try:
            self.uav.publish_position_setpoint(self.end)
        except Exception:
            pass

    def check_status(self) -> str:
        if not self.end:
            return "complete"
        pos = getattr(self.uav, 'local_position', None)
        if not pos:
            return "continue"
        cur = (pos.x, pos.y, pos.z)
        if math.dist(cur, self.end) <= 1.0:
            try:
                completed = getattr(self.uav, 'hoops_traversed', [])
                route = getattr(self.uav, 'planned_route', []) or []
                if route:
                    completed.append(route.pop(0))
                    self.uav.hoops_traversed = completed
                    self.uav.planned_route = route
            except Exception:
                pass
            return "complete"
        return "continue"
