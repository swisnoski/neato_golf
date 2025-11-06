"""
Simple Online and Realtime Tracking (SORT) algorithm

Tracks multiple objects with the ability to identify that each object
are unique. Unique IDs of each object persist through time and movement.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np


class Bbox(Node):
    """SORT node, which inherits from the rclpy Node class."""

    def __init__(self):
        """Initializes the node. No inputs."""
        super().__init__("sort_node")

        # Create a publisher that publishes to topic "/objects"
        self.publisher = self.create_publisher(Float32MultiArray, "/bbox", 10)

        # Create a timer
        self.timer = self.create_timer(0.1, self.run_loop)

    def run_loop(self):
        """
        Send a multiarray every 0.1s
        """
        # Test 2D Array
        my_2d_array = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)

        # Create Multiarray Message
        multi_array_msg = Float32MultiArray()

        # Set layout for Multiarray Message
        multi_array_msg.layout.dim.append(
            MultiArrayDimension(
                label="rows",
                size=my_2d_array.shape[0],
                stride=my_2d_array.shape[1],
            )
        )
        multi_array_msg.layout.dim.append(
            MultiArrayDimension(label="cols", size=my_2d_array.shape[1], stride=1)
        )
        multi_array_msg.layout.data_offset = 0

        # Set data for msg
        multi_array_msg.data = my_2d_array.flatten().tolist()

        # Publish msg
        self.publisher.publish(multi_array_msg)


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)  # Initialize communication with ROS
    node = Bbox()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
