import rclpy
from threading import Thread
from rclpy.node import Node
import time
import math 
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class NeatoTracker(Node):
    """The BallTracker is NOT a Python object that encompasses a ROS node
    that can NOT process images from the camera and NOT search for a ball within.
    The node will NOT issue motor commands to move forward while NOT keeping
    the ball in the center of the camera's field of view."""

    def __init__(self):
        """Initialize the neator tracker"""
        super().__init__("neato_tracker")
        self.camera = cv2.VideoCapture(0)
        self.cv_image = None  # the latest image from the camera
        self.binary_image = None
        self.filled_image = None

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""

        cv2.namedWindow("video_window")
        # cv2.namedWindow('binary_window')
        cv2.namedWindow("filtered_neato_window")

        if self.camera.isOpened():  # try to get the first frame
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
            self.find_contour()
            cv2.imshow("video_window", self.cv_image)
            # cv2.imshow('binary_window', self.binary_image)
            cv2.imshow("filtered_neato_window", self.filled_image)

            _, self.cv_image = self.camera.read()
            cv2.waitKey(5)


    def go_to_point(self, desired_x, desired_y):
        """
        Moves the robot to a desired 2D point using current odometry information.

        The robot first rotates in place to face the target, then drives straight.

        Args:
            desired_x (float): Target X position.
            desired_y (float): Target Y position.
        """
        current_x, current_y, current_angle = self.position
        
        # print("Current x is: ", current_x)
        # print("Current y is: ", current_y)
        # print("Des x is: ", desired_x)
        # print("Des y is: ", desired_y)

        # First, we calculate the desired orientation to drive in 
        # odom gives us quanternion, not yaw (z direction)
        x = desired_x - current_x
        y = desired_y - current_y

        desired_angle = math.atan2(y, x)
        print(current_angle, desired_angle)

        current_angle = current_angle % (2 * math.pi)
        desired_angle = desired_angle % (2 * math.pi)
        
        rotation_needed = (desired_angle - current_angle) % (2*math.pi)

        angular_vel = 0.5
        lin_velocity = 0.3

        # then we can perform our actual rotation
        if rotation_needed < math.pi: 
            self.drive(linear=0.0, angular=angular_vel)
            time.sleep((rotation_needed / angular_vel))

        else: 
            rotation_needed = (2* math.pi - rotation_needed)
            self.drive(linear=0.0, angular=-angular_vel)
            time.sleep((rotation_needed / angular_vel))
        self.drive(linear=0.0, angular=0.0)

        # calculate needed distance and drive forward 
        distance = math.sqrt((x)**2 + (y)**2)
        self.drive(linear=lin_velocity, angular=0.0)
        time.sleep((distance / lin_velocity))

        # set speed to zero 
        self.drive(linear=0.0, angular=0.0)


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
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(filled_neato_frame, mask, (0, 0), 255)
        inv_filled_neato_frame = cv2.bitwise_not(filled_neato_frame)

        self.filled_image = self.binary_image | inv_filled_neato_frame


    def find_contour(self):
        # find contours in the thresholded image
        cnts, _ = cv2.findContours(
            self.filled_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        # Source - https://stackoverflow.com/questions/42798659/how-to-remove-small-connected-objects-using-opencv
        # Posted by Neeraj Madan
        # Retrieved 2025-11-05, License - CC BY-SA 4.0
        cnts = [c for c in cnts if cv2.contourArea(c) > 10000]

        mask = np.zeros_like(self.filled_image)

        # Fill only the large contours
        for contour in cnts:
            area = cv2.contourArea(contour)
            if area > 10000:
                cv2.fillPoly(mask, [contour], 255)

        # Apply the mask to keep only large regions
        self.filled_image = cv2.bitwise_and(self.filled_image, mask)


        # loop over the contours
        if cnts is not None:
            for c in cnts:
                # compute the center of the contour
                # Source - https://stackoverflow.com/questions/35247211/zerodivisionerror-python
                # Posted by handle
                # Retrieved 2025-11-05, License - CC BY-SA 4.0
                cx = 0
                cy = 0
                for p in c:
                    cx += p[0][0]
                    cy += p[0][1]
                cx = int(cx/len(c))
                cy = int(cy/len(c))

                # draw the contour and center of the shape on the image
                cv2.drawContours(self.filled_image, [c], -1, (0, 255, 0), 2)
                cv2.circle(self.filled_image, (cx, cy), 7, (0, 255, 255), -1)
                cv2.putText(
                    self.filled_image,
                    "center",
                    (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )


def main(args=None):
    rclpy.init()
    n = NeatoTracker()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
