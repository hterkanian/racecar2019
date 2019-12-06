#!/usr/bin/env python
"""
Author: Harry Terkanian
December 5, 2019

Input:      ROS message from Zed Camera with image

Output(s):  Publishes steering angle to steer racecar toward object identified 
            in image (at arbitrary speed set by self.drive_speed);

Uses CvBridge to convert mesage image from Zed camera
Uses OpenCV2 to process converted image to locate object(s) in desired 
color range.

NOTE: Adjust self.color_range for the desired HSV color range of object(s) 
    being searched for.  

NOTE: current color_range is for yellow.

Requires: ROS Kinetic, Python 2.7, OpenCV 2, cv_bridge
"""


import rospy
import roslib
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np      #cv2 requires numpy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped


class ObjectFollowerNode:


    def __init__(self):
        #===============Class initialization===================================

        #===============State Variables========================================
        self.color_range    = [np.array([0, 150, 150]), 
                                    np.array([40, 255, 255])]
        self.drive_speed    = 0.50  # arbitrary drive speed
        self.Kp             = 5.0   # proportional steering gain
        self.bridge = CvBridge()    # converts CV2 images to ROS and back
                
        #===============begin ROS topic subscriptions & publications===========           
        #===============publisher for Ackermann drive /navigation topic========
        self.pub_cmd = rospy.Publisher(
                "/ackermann_cmd_mux/input/navigation",
                AckermannDriveStamped, queue_size = 10
                )

        #===============subscribe to /zed/left/image_rect_color================
        rospy.Subscriber("/zed/left/image_rect_color",
                Image,
                self.camera_callback,
                self.pub_cmd
                )
        #===============end ROS subscriptions and publications=================
        #===============end __init__() method==================================


    #===================class methods==========================================
    def camera_callback(self, msg, publisher):
        """
        Callback method for camera messages.  
        identifies object(s) in image with color in specified color range; 
        calculates relationship of horizontal location of object center
        to horizontal image center, normalized to range (-1: +1).
        Steering command is issued to turn toward object.

        Image message format:
            msg.header
            msg.height       # uint32 image height (rows)
            msg.width        # uint32 image width (columns)
            msg.encoding     # string encoding of pixels
                           Color values:
                                RGB8 = "rgb8";
                                RGBA8 = "rgba8";
                                BGR8 = "bgr8";
                                BGRA8 = "bgra8";
                                MONO8="mono8";
                                MONO16="mono16";
            msg.is_bigendian # is data big_endian
            msg.step         # full row length in bytes
            msg.data         # [] matrix data, size is (step * rows)
        """

        # publisher object
        pub_cmd =  publisher

        image_msg = msg

        # extract image from message
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 
                    image_msg.encoding) 
        except CvBridgeError as e:
            print('CvBridge error: ', e)    # fatal, quit

        # Convert image to HSV format

        #TODO add arguments to cvtColor to convert to hsv encoding
        hsv_image =  cv2.cvtColor(?????)

        # find any object(s) in image with color in color range
        #TODO add arguments to produce image with only desired color
        image_threshold = cv2.inRange(hsv_image,????? )

        # get rid of spurious pixels by averaging over a block
        #TODO 5 x 75 block is arbitrary, feel free to change
        cv2.blur(image_threshold, (5, 75))

        # find the contours
        #TODO add arguments to find contours in image_threshold
        img_mod, contours, hierarchy = cv2.findContours(image_threshold,
                cv2.RETR_CCOMP,
                cv2.CHAIN_APPROX_SIMPLE)

        # create a copy to draw contours on
        labeled_image = image_thresh.copy()
        #TODO add arguments to draw contours
        cv2.drawContours(labeled_image, contours, ????)

        # identify the index of the largest contour using cv2.contourArea
        # return the index of the largest_contour or None if no contours found
            largest_contour = ??????
                                       
        if largest_contour == None:
            # no contours found in image; nothing to do
            return
 
        # calculate the error:
        # find bounding rectangle of object with largest contour using 
        # cv2.boundingRect function; 
        # calculate the horizontal center of that rectangle;
        # calculate how far the rectangle center is from the image center; and
        # normalize the result to range -1:+1 so the error is dimensionless
        rect = cv2.boundingRect(contours[largest_contour])

        #TODO find horizontal center of rectangle, calculate error
        error = 

        # call controller to issue drive command
        self.controller(error, pub_cmd)


    def controller(self, error, pub_cmd):
        """
        Applies prorpotional steering gain (self.Kp)
        Caps steering angle at +/- self.steering_saturation
        Issues drive message with arbitrary drive speed and calculated 
        steering angle.
        """

        cmd_msg = AckermannDriveStamped()
        cmd_msg.drive.speed = self.drive_speed

        # apply proportional controller gain (self.Kp)
        raw_steering_angle = error * self.Kp 

        # cap steering angle at self.steering_saturation
        if abs(raw_steering_angle) > self.steering_saturation:
            steering_angle = math.copysign(self.steering_saturation, 
                    raw_steering_angle)

        # insert steering angle and issue drive command
        cmd_msg.drive.steering_angle = steering_angle
        pub_cmd.publish(cmd_msg)


#===============execution begins here==========================================
if __name__ == "__main__":
    rospy.init_node("our_object_follower", anonymous = True)
    node = ObjectFollowerNode()
    rospy.spin()
