"""Main module for tracking and driving the Neato, contains a ROS node"""

from threading import Thread
import time
import math
from rclpy.node import Node
import rclpy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from neato_golf_donkey_kong.helpers import numpy_to_multiarray


class NeatoTracker(Node):
    """The NeatoTracker is a Python object that encompasses a ROS node
    that can process images from the camera and search for a *golf* ball within,
    and a Neato, and a target.
    The node will issue motor commands to move while keeping the everything in the
    camera's field of view."""

    def __init__(self):
        """Initialize the neator tracker"""
        super().__init__("neato_tracker")
        self.neato_position = [0, 0, 0]  # only uses x and y
        self.pixels_to_cm = 0  # scaling factor depending on distance to ground

        self.camera = cv2.VideoCapture(0)
        self.cv_image = None  # the latest image from the camera
        self.binary_image = None  # binary image of the cv_image
        self.filled_image = None  # filtered to only include the Neato
        self.edge_image = None  # Canny processed image
        self.balls = []  # Potential ball coordinates
        self.target = []  # Potential target coordinates
        self.planned = False  # If the path planning has happened
        self.lines = []  # List of endpoints of lines
        self.path = []  # List of waypoints of the path

        # Create Publishers
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.bbox_pub = self.create_publisher(Float32MultiArray, "bbox", 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()

        driver_thread = Thread(target=self.driver_wrapper)
        driver_thread.start()

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""

        cv2.namedWindow("video_window")
        cv2.namedWindow("binary_window")
        cv2.namedWindow("filtered_neato_window")

        if self.camera.isOpened():  # try to get the first frame
            rval, self.cv_image = self.camera.read()
        else:
            rval = False
            print("camera cannot be read")

        # index = 1
        while rval:
            self.find_neato()
            self.find_ball()
            self.find_target()
            self.find_contour()
            self.find_line()
            self.find_heading()
            self.run_loop()
            print(self.balls)
            self.plan_path()
            # if self.balls and self.planned:
            #     if index % 50 == 0:
            #         print(self.balls)
            #         print("GO GO GO")
            #         # self.go_to_pixel_coord(self.balls[0][0], self.balls[0][1])
            #         self.go_to_pixel_coord(self.path[0][0], self.path[0][1])
            #         self.find_neato()
            #         self.find_ball()
            #         self.find_target()
            #         self.find_contour()
            #         self.find_line()
            #         self.find_heading()
            #         self.run_loop()
            #         self.go_to_pixel_coord(self.path[1][0], self.path[1][1])
            #         # self.go_to_pixel_coord(222, 200)
            # index += 1
            # print(index)
            time.sleep(0.1)

    def driver_wrapper(self):
        """This function takes care of sending the driving commands to the Neato.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""
        index = 1
        while True:
            if self.balls and self.planned:  # Ball deteted and path planned
                if index % 100 == 0:
                    print(self.balls)
                    print("GO GO GO")
                    self.go_to_pixel_coord(self.path[0][0], self.path[0][1])
                    self.go_to_pixel_coord(self.path[1][0], self.path[1][1])
            index += 1
            time.sleep(0.1)
            print("Index: ", index)

    def run_loop(self):
        """This function takes care of doing all the cv2 related tasks,
        including showing the images and reading the new frame from the camera."""
        # NOTE: only do cv2.imshow and cv2.waitKey in this function
        if not self.cv_image is None:
            cv2.imshow("video_window", self.cv_image)
            cv2.imshow("canny", self.edge_image)

            cv2.imshow("binary_window", self.binary_image)
            cv2.imshow("filtered_neato_window", self.filled_image)

            _, self.cv_image = self.camera.read()
            cv2.waitKey(5)

    def go_to_pixel_coord(self, desired_pixel_x, desired_pixel_y):
        """
        Moves the robot to a desired 2D point using current odometry information.

        The robot first rotates in place to face the target, then drives straight.

        Args:
            desired_x (float): Target X position in PIXELS
            desired_y (float): Target Y position in PIXELS
        """
        ANGULAR_VEL = 0.3
        LIN_VELOCITY = 0.05

        current_pixel_x, current_pixel_y, current_angle = self.neato_position

        # print("Current x is: ", current_x)
        # print("Current y is: ", current_y)
        # print("Des x is: ", desired_x)
        # print("Des y is: ", desired_y)

        # calculate x and y from PIXELS to METERS
        x = (desired_pixel_x - current_pixel_x) * (1 / self.pixels_to_cm) * 0.01

        # make y negative since down is positive
        y = -(desired_pixel_y - current_pixel_y) * (1 / self.pixels_to_cm) * 0.01

        desired_angle = math.atan2(y, x)
        print(current_angle, desired_angle)

        # Calculate current and desired angle
        current_angle = current_angle % (2 * math.pi)
        desired_angle = desired_angle % (2 * math.pi)

        rotation_needed = (desired_angle - current_angle) % (2 * math.pi)

        # then we can perform our actual rotation
        if rotation_needed < math.pi:
            self.drive(linear=0.0, angular=ANGULAR_VEL)
            time.sleep((rotation_needed / ANGULAR_VEL))

        else:
            rotation_needed = 2 * math.pi - rotation_needed
            self.drive(linear=0.0, angular=-ANGULAR_VEL)
            time.sleep((rotation_needed / ANGULAR_VEL))
        self.drive(linear=0.0, angular=0.0)

        # calculate needed distance and drive forward
        distance = math.sqrt((x) ** 2 + (y) ** 2) - 0.15
        print(distance)
        self.drive(linear=LIN_VELOCITY, angular=0.0)
        time.sleep((distance / LIN_VELOCITY))

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
        """
        Filters binary image to just Neato, then fills it

        This function takes in the raw frame from the camera and does color filtering
        to get its binary image. It then fills every pixel outside of the Neato boundary,
        inverses the binary, and now we have a binary image with just the Neato filled.
        """

        # Color filter
        self.binary_image = cv2.inRange(self.cv_image, (0, 0, 0), (120, 110, 100))

        # Image processing magic
        filled_neato_frame = self.binary_image.copy()
        h, w = self.cv_image.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(filled_neato_frame, mask, (0, 0), 255)
        inv_filled_neato_frame = cv2.bitwise_not(filled_neato_frame)

        # Invert filled image so only the Neato is filled
        self.filled_image = self.binary_image | inv_filled_neato_frame

    def find_ball(self):
        """
        Using camera data, find bounding box of ball(s).

        Takes in raw image data from the camera, then uses color filters to identify
        the golf ball(s) in the frame. Appending all of their pixel coordinates and
        bounding boxes into a list for further processing.
        """
        # Mask specific range of white
        lower_white = np.array([220, 220, 220])  # Adjust these values
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(self.cv_image, lower_white, upper_white)

        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Get center, width, and height from contours
        bbox = []
        self.balls = []
        for contour in contours:
            if len(contour) < 4:
                # Invalid detection
                continue

            cx, cy, w, h = self.bbox_chw(contour)

            bbox.append([cx, cy, w, h])
            self.balls.append([cx, cy])

            cv2.circle(self.cv_image, (int(cx), int(cy)), 7, (0, 255, 255), -1)

        # Convert list to Float32MultiArray msg and publish to /bbox
        bbox_arr = np.array(bbox)
        multiarr_msg = numpy_to_multiarray(bbox_arr)
        self.bbox_pub.publish(multiarr_msg)

    def find_target(self):
        """
        Using camera data, find bounding box of the target.

        Takes in raw image data from the camera, then uses color filters to identify
        the target in the frame. Storing its coordinates and bounding box for further
        processing.
        """
        # Convert to HSV
        lower_white = np.array([20, 100, 175])  # Adjust these values as needed
        upper_white = np.array([50, 200, 240])
        frame_hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # Get mask
        mask = cv2.inRange(frame_hsv, lower_white, upper_white)

        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through individual contours
        detections = []
        for contour in contours:
            if len(contour) < 4:
                # Invalid detection
                continue

            cx, cy, w, h = self.bbox_chw(contour)

            detections.append((cx, cy, w, h))

        if detections:
            print(f"Detections: {detections}")
            self.target = [detections[0]]

    def find_contour(self):
        """
        Find contours in the thresholded image.

        Takes in the thresholded (filled) image, find the contour of shapes
        in the image, and only fills shapes above a certain size, effectively
        filtering out the Neato. Then identifies the moments of the image and
        locates the center of the Neato.
        """
        AREA_CONST = 2000
        cnts, _ = cv2.findContours(
            self.filled_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        # Source - https://stackoverflow.com/questions/42798659/how-to-remove-small-connected-objects-using-opencv
        # Posted by Neeraj Madan
        # Retrieved 2025-11-05, License - CC BY-SA 4.0
        cnts = [c for c in cnts if cv2.contourArea(c) > AREA_CONST]
        # print(len(cnts))

        mask = np.zeros_like(self.filled_image)

        # Fill only the large contours
        for contour in cnts:
            area = cv2.contourArea(contour)
            if area > AREA_CONST:
                cv2.fillPoly(mask, [contour], 255)

        if len(cnts) == 1:
            self.pixels_to_cm = math.sqrt(cv2.contourArea(cnts[0]) / 1000)
            print(f"Scaling ratio: {self.pixels_to_cm}")
            print(f"Neato position: {self.neato_position}")

        # Apply the mask to keep only large regions
        self.filled_image = cv2.bitwise_and(self.filled_image, mask)

        # loop over the contours
        if cnts is not None:
            for c in cnts:
                # compute the center of the contour
                # Source - https://stackoverflow.com/questions/35247211/zerodivisionerror-python
                # Posted by handle
                # Retrieved 2025-11-05, License - CC BY-SA 4.0
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx = 0
                    cy = 0
                    for p in c:
                        cx += p[0][0]
                        cy += p[0][1]
                    cx = int(cx / len(c))
                    cy = int(cy / len(c))
                self.neato_position = [cx, cy, self.neato_position[2]]

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

    def bbox_chw(self, pts):
        """
        Takes in points and returns the bounding box of the rectangle it represents.

        Returns:

        (cx, cy, w, h)
            cx, cy : center coordinates
            w, h   : width and height
        """
        x, y, w, h = cv2.boundingRect(pts)
        cx = x + w / 2.0
        cy = y + h / 2.0

        return cx, cy, w, h

    def find_line(self):
        """
        Finds straight lines in a binary image.

        This function takes in a binary image, uses Canny to perform edge detection,
        then uses the HoughLinesP method to directly obtain end points of lines, with
        the threshold set so that only the straight edge of the Neato would be detected.
        """
        # Apply HoughLinesP method to
        # to directly obtain line end points
        lines_list = []
        self.edge_image = cv2.Canny(self.filled_image, 000, 50)
        lines = cv2.HoughLinesP(
            self.edge_image,  # Input edge image
            1,  # Distance resolution in pixels
            np.pi / 180,  # Angle resolution in radians
            threshold=90,  # Min number of votes for valid line
            minLineLength=4,  # Min allowed length of line
            maxLineGap=10,  # Max allowed gap between line for joining them
        )

        # Iterate over points
        if lines is not None:  # If there are lines
            print(f"Length: {len(lines)}")
            for points in lines:
                # Extracted points nested in the list
                x1, y1, x2, y2 = points[0]
                # Draw the lines joing the points
                # On the original image
                cv2.line(self.cv_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
                # Maintain a simples lookup list for points
                lines_list.append([(x1, y1), (x2, y2)])
        else:
            print("No line detected")
        self.lines = lines

    def find_heading(self):
        """
        Finds heading of a Neato based on the endpoints of the straight edge.

        This function takes in one pair of endpoints representing a line segment
        of the straight edge of the Neato, then finds its slope and its inverse,
        calculates the true heading of the Neato using arctan2.=
        """
        if self.lines is None:
            print("No line to find heading for")
            return

        if len(self.lines) == 1:
            x1, y1, x2, y2 = self.lines[0][0]
        else:
            print("More than one line, unable to find heading")
            return

        cx = self.neato_position[0]
        cy = self.neato_position[1]

        slope_neato = (y2 - y1) / (x2 - x1)
        slope_heading = -(1 / slope_neato)
        b_heading = cy - slope_heading * cx  # y = mx + b
        b_neato = y1 - slope_neato * x1
        # Find the intersecting point of two lines
        x_intersect = (b_heading - b_neato) / (slope_neato - slope_heading)
        y_intersect = slope_neato * x_intersect + b_neato
        # Flips y because down is positive
        angle = np.arctan2(cy - y_intersect, x_intersect - cx)
        print(f"ANGLE: {angle}")
        self.neato_position[2] = angle

    def plan_path(self):
        """
        Plans path given coordinates of the golf ball and target.

        Takes in the coordinates of the golf ball and target, then finds the
        target coordinates for the Neato to allow for ample space to turn and
        capture the golf ball.
        """
        if not self.balls or not self.target:
            print("No balls or no target, path not planned")
            return
        x1, y1 = self.balls[0][0], self.balls[0][1]
        x2, y2 = self.target[0][0], self.target[0][1]
        print(f"x1: {x1}, x2: {x2}, y1: {y1}, y2: {y2}")

        slope = (y2 - y1) / (x2 - x1)
        b = y1 - slope * x1

        # Calculate unit vector in direction of the line
        length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        unit_x = (x2 - x1) / length

        shift_x = 25 * unit_x  # Calculate shift of 10 units along the line

        # Shifts the coordinate depending on Neato's relative location to the ball
        if x2 > x1:
            x_dest = x1 - shift_x
        else:
            x_dest = x1 + shift_x
        y_dest = slope * x_dest + b

        self.path = [[x_dest, y_dest], [x2, y2]]
        self.planned = True
        print("PATH PLANNED")


def main(args=None):
    rclpy.init()
    n = NeatoTracker()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
