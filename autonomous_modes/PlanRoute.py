from typing import List, Optional, Tuple
import math
from .Mode import Mode
from rclpy.node import Node
from uav import UAV


class PlanRoute(Mode):
    """Simple nearest-neighbor planner that creates pre-approach waypoints.

    Expects discovered hoops to be available on `self.uav.hoops_discovered` as
    a list of dicts with a 'position' key containing (x,y,z) and an optional
    'normal' or 'bearing'. The planner writes a list `self.uav.planned_route`
    containing dicts with 'hoop' and 'pre_approach' keys.
    """

    def __init__(self, node: Node, uav: UAV, pre_approach_dist: float = 3.5, max_targets: int = 4,
                 points: Optional[List[Tuple[float, float, float]]] = None):
        super().__init__(node, uav)
        self.pre_approach_dist = float(pre_approach_dist)
        self.max_targets = int(max_targets)
        self.route = []
        self.points = points

    def on_enter(self) -> None:
        self.node.get_logger().info("PlanRoute: computing simple nearest-neighbor route")
        hoops = []
        if self.points:
            for p in self.points:
                try:
                    x, y, z = float(p[0]), float(p[1]), float(p[2])
                    hoops.append({'position': (x, y, z)})
                except Exception:
                    continue
        else:
            hoops = getattr(self.uav, 'hoops_discovered', []) or []
        if not hoops:
            self.node.get_logger().info("No hoops discovered - skipping route planning")
            self.route = []
            try:
                self.uav.planned_route = []
            except Exception:
                pass
            return

        centers = []
        for h in hoops:
            if isinstance(h, dict) and 'position' in h:
                pos = h.get('position')
                try:
                    centers.append({'hoop': h, 'pos': tuple((float(pos[0]), float(pos[1]), float(pos[2])))})
                except Exception:
                    continue
            elif isinstance(h, (list, tuple)) and len(h) >= 3:
                try:
                    centers.append({'hoop': {'position': tuple((float(h[0]), float(h[1]), float(h[2])) )}, 'pos': tuple((float(h[0]), float(h[1]), float(h[2])))})
                except Exception:
                    continue

        cur = getattr(self.uav, 'local_position', None)
        if cur:
            try:
                cur_pos = (float(cur.x), float(cur.y), float(cur.z))
            except Exception:
                cur_pos = (0.0, 0.0, 0.0)
        else:
            cur_pos = (0.0, 0.0, 0.0)

        remaining = centers.copy()
        order = []
        while remaining and len(order) < self.max_targets:
            remaining.sort(key=lambda c: math.dist(cur_pos, c['pos']))
            chosen = remaining.pop(0)
            order.append(chosen)
            cur_pos = chosen['pos']

        planned = []
        for item in order:
            hx, hy, hz = item['pos']
            normal = None
            hoop = item['hoop']
            if isinstance(hoop, dict) and 'bearing' in hoop:
                b = hoop['bearing']
                normal = (math.cos(b), math.sin(b), 0.0)
            else:
                sx, sy, _ = getattr(self.uav, 'home_pose', (0.0, 0.0, 0.0))
                vx, vy = hx - sx, hy - sy
                d = math.hypot(vx, vy) or 1.0
                normal = (vx / d, vy / d, 0.0)

            pre_x = hx - normal[0] * self.pre_approach_dist
            pre_y = hy - normal[1] * self.pre_approach_dist
            pre_z = hz

            planned.append({'hoop': hoop, 'pre_approach': (pre_x, pre_y, pre_z), 'hoop_pos': item['pos']})

        self.route = planned
        try:
            self.uav.planned_route = planned
        except Exception:
            pass

    def on_update(self, time_delta: float) -> None:
        return

    def check_status(self) -> str:
        return "complete"
