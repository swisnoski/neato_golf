import rclpy
from threading import Thread
from rclpy.node import Node
import time
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class NeatoTracker(Node):
    """ The BallTracker is NOT a Python object that encompasses a ROS node 
        that can NOT process images from the camera and NOT search for a ball within.
        The node will NOT issue motor commands to move forward while NOT keeping
        the ball in the center of the camera's field of view. """

    def __init__(self):
        """ Initialize the neator tracker """
        super().__init__('neato_tracker')
        self.camera = cv2.VideoCapture(0)
        self.cv_image = None                        # the latest image from the camera
        self.binary_image = None
        self.filled_image = None
      
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()


    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        
        cv2.namedWindow('video_window')
        # cv2.namedWindow('binary_window')
        cv2.namedWindow("filtered_neato_window")

        if self.camera.isOpened(): # try to get the first frame
            rval, self.cv_image = self.camera.read()
        else:
            rval = False
            print("camera cannot be read")

        while rval:
            self.run_loop()
            time.sleep(0.1)


    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.cv_image is None:
            self.find_neato()
            cv2.imshow('video_window', self.cv_image)
            # cv2.imshow('binary_window', self.binary_image)
            cv2.imshow('filtered_neato_window', self.filled_image)

            _, self.cv_image = self.camera.read() 
            cv2.waitKey(5)


    def drive(self, linear, angular):
        """
        Publishes a Twist message with the specified linear and angular velocities.

        Args:
            linear (float): Linear velocity in m/s.
            angular (float): Angular velocity in radians/s.
        """    
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


    def find_neato(self):

        self.binary_image = cv2.inRange(self.cv_image, (0, 0, 0), (120, 110, 100))

        filled_neato_frame = self.binary_image.copy()
        h, w = self.cv_image.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(filled_neato_frame, mask, (0,0), 255)
        inv_filled_neato_frame = cv2.bitwise_not(filled_neato_frame)

        self.filled_image = self.binary_image | inv_filled_neato_frame


def main(args=None):
    rclpy.init()
    n = NeatoTracker()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()