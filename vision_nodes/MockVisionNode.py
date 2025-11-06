"""
MockVisionNode

This module provides a lightweight mock vision node that subclasses
the project's VisionNode. It demonstrates how to obtain the latest
camera frame (via the VisionNode helper) and run a processing function
that returns a list of (x, y, z) coordinates. Currently the processing
function is a stub that returns a hard-coded mock list; replace it with
real detection logic later.

Usage:
    node = MockVisionNode(display=False, use_service=False)
    detections = node.get_detections()

`get_detections()` will try to fetch the latest Image message (either
from topic subscription or via service) and convert it to an OpenCV
frame before calling `process_frame(frame)` which returns a list of
3-tuples (x,y,z).
"""

from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from .VisionNode import VisionNode
from uav_interfaces.srv import CameraData
import numpy as np


class MockVisionNode(VisionNode):
    def __init__(self, display: bool = False, use_service: bool = False):
        # VisionNode expects a custom_service type as first argument; pass CameraData
        super().__init__(custom_service=CameraData, display=display, use_service=use_service)
        self.get_logger().info("MockVisionNode initialized")

    def process_frame(self, frame: np.ndarray) -> List[Tuple[float, float, float]]:
        """Process an OpenCV BGR frame and return a list of (x,y,z) coords.

        This is a stub implementation. Replace with real detection logic.
        The coordinates should be returned in the map/local frame expected
        by the mission (x, y, z).
        """
        # TODO: implement hoop detection and range estimation. For now, return a mock list.
        # Example: return two fake hoop positions in front of the vehicle
        return [ (1.0, 2.0, -1.0), (3.5, 0.5, -1.2) ]

    def get_detections(self) -> List[Tuple[float, float, float]]:
        """Fetch the latest camera frame, convert it, and return detections.

        This method will:
          - If `use_service` is True, call the camera service to retrieve the image.
          - Otherwise use the last-subscribed Image message on `/camera`.
          - Convert the Image message to an OpenCV frame using
            `convert_image_msg_to_frame` and then call `process_frame`.
        """
        # Try service path first if configured
        image_msg = None
        if self.use_service:
            try:
                img, caminfo = self.request_data(cam_image=True, cam_info=False)
                image_msg = img
            except Exception as e:
                self.get_logger().warning(f"Camera service call failed: {e}")

        # If no image_msg from service, fall back to last-subscribed image
        if image_msg is None:
            image_msg = getattr(self, 'image', None)

        if image_msg is None:
            self.get_logger().debug("No image available to process")
            return []

        try:
            frame = self.convert_image_msg_to_frame(image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image to frame: {e}")
            return []

        # Optionally display the frame
        if self.display:
            try:
                self.display_frame(frame, self.node_name())
            except Exception:
                pass

        # Run processing function (stub for now)
        detections = self.process_frame(frame)

        # Convert/validate detections to expected shape
        good = []
        for d in detections:
            if isinstance(d, (list, tuple)) and len(d) == 3:
                try:
                    x, y, z = float(d[0]), float(d[1]), float(d[2])
                    good.append((x, y, z))
                except Exception:
                    continue

        return good


def main(args=None):
    rclpy.init(args=args)
    node = MockVisionNode(display=False, use_service=False)

    try:
        # simple loop that prints detections periodically
        import time
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            dets = node.get_detections()
            if dets:
                node.get_logger().info(f"Mock detections: {dets}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
