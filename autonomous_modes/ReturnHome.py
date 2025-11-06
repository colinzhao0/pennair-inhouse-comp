from .Mode import Mode
from rclpy.node import Node
from uav import UAV


class ReturnHome(Mode):
    """Return to saved home pose and hold.

    This mode simply publishes the saved home pose as a position
    setpoint until we are close enough.
    """

    def __init__(self, node: Node, uav: UAV, tol: float = 1.0):
        super().__init__(node, uav)
        self.tol = float(tol)
        self.home = None

    def on_enter(self) -> None:
        self.home = getattr(self.uav, 'home_pose', None) or getattr(self.uav, 'local_position', None)
        self.node.get_logger().info(f"ReturnHome: home={self.home}")

    def on_update(self, time_delta: float) -> None:
        if not self.home:
            return
        target = (
            self.home[0] if isinstance(self.home, (list, tuple)) else getattr(self.home, 'x', 0.0),
            self.home[1] if isinstance(self.home, (list, tuple)) else getattr(self.home, 'y', 0.0),
            self.home[2] if isinstance(self.home, (list, tuple)) else getattr(self.home, 'z', 2.0),
        )
        try:
            self.uav.publish_position_setpoint(target)
        except Exception:
            pass

    def check_status(self) -> str:
        pos = getattr(self.uav, 'local_position', None)
        if not pos or not self.home:
            return "continue"
        cur = (pos.x, pos.y, pos.z)
        hx = self.home[0] if isinstance(self.home, (list, tuple)) else getattr(self.home, 'x', 0.0)
        hy = self.home[1] if isinstance(self.home, (list, tuple)) else getattr(self.home, 'y', 0.0)
        hz = self.home[2] if isinstance(self.home, (list, tuple)) else getattr(self.home, 'z', 2.0)
        dist = ((cur[0]-hx)**2 + (cur[1]-hy)**2 + (cur[2]-hz)**2) ** 0.5
        if dist <= self.tol:
            self.node.get_logger().info("ReturnHome: reached home")
            return "complete"
        return "continue"
