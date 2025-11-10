"""
Simple Online and Realtime Tracking (SORT) algorithm

Tracks multiple objects with the ability to identify that each object
are unique. Unique IDs of each object persist through time and movement.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import numpy as np
from neato_golf_donkey_kong.helpers import numpy_to_multiarray, multiarray_to_numpy
import cv2


class Sort(Node):
    """SORT node, which inherits from the rclpy Node class."""

    def __init__(self):
        """Initializes the node. No inputs."""
        super().__init__("sort_node")

        # Create a publisher that publishes to topic "/objects"
        self.publisher = self.create_publisher(Float32MultiArray, "/objects", 10)

        # Create a subscriber to "/bbox"
        self.bbox_sub = self.create_subscription(
            Float32MultiArray, "/bbox", self.bbox_msg, 10
        )

        # Initialize linear filter variable
        self.tracks = []

    def bbox_msg(self, msg):
        """
        Detect unique objects from bbox information
        """
        # Convert Float32Array object into Numpy arr
        arr = multiarray_to_numpy(msg)

        # Parse bbox data into detections
        detections = []
        for bbox in arr:
            detections.append((bbox[0], bbox[1], bbox[2], bbox[2]))

        # Initialize tracks if empty
        if len(self.tracks) == 0 and len(detections) != 0:
            self.tracks.append(
                {
                    "id": 1,
                    "state": np.array(
                        [detections[0][0], detections[0][1], detections[0][2], 1, 0, 0]
                    ),
                    "missed_frames": 0,
                }
            )

        # Make predictions of existing tracks
        for track in self.tracks:
            center = np.array([track["state"][0], track["state"][1]])
            velocity = np.array([track["state"][4], track["state"][5]])
            track["predicted"] = self.make_prediction(center, velocity)

        # Use overlap to assign detections to existing tracks
        assigned_detections = set()
        for track in self.tracks:
            best_overlap = 0
            best_det = None
            for i, det in enumerate(detections):
                if i in assigned_detections:
                    continue
                cx, cy, w, h = det
                w_p = w * 1.6
                overlap = self.bbox_overlap(
                    np.array(
                        [
                            track["predicted"][0] - w_p / 2,
                            track["predicted"][1] - w_p / 2,
                            track["predicted"][0] + w_p / 2,
                            track["predicted"][1] + w_p / 2,
                        ]
                    ),
                    np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]),
                )
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_det = (i, det)

            # Update if match found
            if best_det and best_overlap > 0:
                i, (cx, cy, w, h) = best_det
                assigned_detections.add(i)
                new_vel_x = cx - track["state"][0]
                new_vel_y = cy - track["state"][1]
                track["state"] = np.array([cx, cy, w, 1, new_vel_x, new_vel_y])
                track["missed_frames"] = 0
            else:
                # No matches were found
                track["missed_frames"] += 1

        # Remove tracked objects with no detections
        self.tracks = [t for t in self.tracks if t["missed_frames"] < 10]

        # Add new tracks for unmatched detections
        for i, det in enumerate(detections):
            if i not in assigned_detections:
                cx, cy, w, h = det
                new_track = {
                    "id": len(self.tracks) + 1,
                    "state": np.array([cx, cy, w, 1, 0, 0]),
                    "missed_frames": 0,
                }
                self.tracks.append(new_track)

        tracks_arr = np.array(
            [
                [track["id"], track["state"][0], track["state"][1], track["state"][2]]
                for track in self.tracks
            ]
        )

        # convert numpy array to Float32MultiArray
        multi_array_msg = numpy_to_multiarray(tracks_arr)

        self.publisher.publish(multi_array_msg)

    # ================ HELPER FUNCTIONS ================

    def make_prediction(self, center, velocity):
        """
        Prediction of contour in the next iteration

        Returns: (x, y)
        """
        return center + velocity

    def bbox_overlap(self, boxA, boxB):
        """
        Calculate overlap area (in pixels) between two bounding boxes.

        Each box is defined as a tuple: (x_min, y_min, x_max, y_max)
        """
        # Unpack coordinates
        xA1, yA1, xA2, yA2 = boxA
        xB1, yB1, xB2, yB2 = boxB

        # Compute intersection rectangle
        x_left = max(xA1, xB1)
        y_top = max(yA1, yB1)
        x_right = min(xA2, xB2)
        y_bottom = min(yA2, yB2)

        # Check for no overlap
        if x_right <= x_left or y_bottom <= y_top:
            return 0  # No overlap

        # Compute overlapping area
        overlap_area = (x_right - x_left) * (y_bottom - y_top)
        return overlap_area


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)  # Initialize communication with ROS
    node = Sort()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
