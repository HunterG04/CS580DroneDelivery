#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from intro_to_robotics.image_converter import ToOpenCV, depthToOpenCV

def process_image(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([0, 10, 10])
    upper_bound = np.array([10,255,255])
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    M = cv2.moments(mask)
    location = None
    magnitude = 0
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        magnitude = M['m00']
        location = (cx-320, cy-240) #scale so that 0,0 is center of screen
        #draw a circle image where we detected the midpoint of the object
        cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

    cv2.imshow("processing result", image)
    cv2.imshow("image_mask", mask)

    #waitKey() is necessary for making all the cv2.imshow() commands work
    cv2.waitKey(1)
    return location, magnitude


class Node:
    def __init__(self):
        #register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)

        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def image_callback(self, ros_image):
        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(ros_image))

        #run our vision processing algorithm to pick out the object
        #returns the location (x,y) of the object on the screen, and the
        #magnitude of the discovered object. Magnitude can be used to estimate
        #distance
        location, magnitude = process_image(cv_image)

        #log the processing results
        rospy.logdebug("image location: {}\tmagnitude: {}".format(location, magnitude))

        ###########
        # Insert turtlebot controlling logic here!
        ###########
        cmd = Twist()


        #publish command to the turtlebot
        self.movement_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("lab2_example")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()
