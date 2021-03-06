#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from intro_to_robotics.image_converter import ToOpenCV, depthToOpenCV
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Transform
import copy
import math
import sys
import urllib2
import json
import colorsys

r = 0.035
l = 0.1

x = 0.0
y = 0.0
yaw = 0.0

init = True
return_flag = False
is_home = True
last_direction = 0
count = 1
j = {}

# this function does our image processing
# returns the location and "size" of the detected object
def process_image(image):
    # convert color space from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    global is_home, j
    if is_home:
        response = urllib2.urlopen("http://andrewlewis.pythonanywhere.com/color/").read()
        j = json.loads(response)
        print "Inside of http request"

    #hsv = colorsys.rgb_to_hsv(j["r"], j["g"], j["b"])

    color = np.uint8([[[j["b"], j["g"], j["r"]]]])
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    hue = hsv_color[0][0][0]
    sat = hsv_color[0][0][1]
    val = hsv_color[0][0][2]

    print j["b"], j["g"], j["r"]
    print hsv_color
    print hue
    print sat
    print val
    # create bounds for our color filter
    upper_bound = np.array([hue + 25, 255, 255])
    lower_bound = np.array([hue - 25, 100, 100])

    # execute the color filter, returns a binary black/white image
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # display the results of the color filter
    cv2.imshow("image_mask", mask)
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    location = None
    magnitude = 0
    width = 0
    contour_list = []
    max_area = 0.
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500 and area > max_area:
            max_contour = contour
            max_area = area

    if max_contour is not None:
        # calculate the centroid of the results of the color filer
        M = cv2.moments(max_contour)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            magnitude = M['m00']
            location = (cx - 320, cy - 240)  # scale so that 0,0 is center of screen
            # draw a circle image where we detected the centroid of the object
            cv2.circle(image, (cx, cy), 3, (0, 0, 255), -1)

            x, y, w, h = cv2.boundingRect(max_contour)
            img = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            width = w

        F = 260 * 0.24 / 0.15  # F = (P * D) /W
        if width == 0:
            distance = 0
        else:
            distance = 0.15 * F / width  # D' = W * F / P

        message1 = "Distance: " + "%8.2f" % distance
        cv2.putText(image, message1, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))
        print distance, width

    global last_direction
    if last_direction == 0:
        message2 = "Direction: " + "go forward"
    elif last_direction == 1:
        message2 = "Direction: " + "right"
    else:
        message2 = "Direction: " + "left"

    cv2.putText(image, message2, (0, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255))

    # display the original image with the centroid drawn on the image
    cv2.imshow("processing result", image)
    # waitKey() is necessary for making all the cv2.imshow() commands work
    cv2.waitKey(1)
    return location, magnitude, width


class Node:
    def __init__(self):
        # register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)

        # create a publisher for sending commands to turtlebot
        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        rospy.Subscriber('/odom', Odometry, self.get_position)

        # self.tf_listener = tf.TransformListener()

        global x, y, ori_vector

        self.position = (x, y)
        self.init_position = None

        rospy.on_shutdown(self.shutdown)

    def get_position(self, odometry):
        global x, y, yaw
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
             odometry.pose.pose.orientation.w])

    # print roll, pitch, yaw

    def spinWheels(self, u1, u2):
        global r, l
        linear_vel = r / 2 * (u1 + u2)
        ang_vel = r / (2 * l) * (u1 - u2)

        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = ang_vel

        print linear_vel, ang_vel

        self.movement_pub.publish(twist_msg)

    # rospy.sleep(0.2)

    def angle_between(self, v1, v2):
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)

        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    # this function wll get called every time a new image comes in
    # all logic occurs in this function
    def image_callback(self, ros_image):
        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(ros_image))

        global init, return_flag, is_home, r, l, last_direction, count, x, y, yaw

        # run our vision processing algorithm to pick out the object
        # returns the location (x,y) of the object on the screen, and the
        # "size" of the discovered object. Size can be used to estimate
        # distance
        # None/0 is returned if no object is seen
        location, magnitude, width = process_image(cv_image)
        # print location, magnitude, width

        self.position = (x, y)

        if init:
            self.init_position = copy.deepcopy(self.position)
            init = False

        if width == 0.0 and not return_flag:
            print "searching for objects", width
            time = 4
            u1 = math.pi * l / (2 * time * r)
            u2 = - math.pi * l / (2 * time * r)
            turn = False
            self.spinWheels(u1, u2)
            last_direction = 2
            is_home = False
            return

        F = 260 * 0.24 / 0.15  # F = (P * D) /W
        if width == 0:
            distance = 0
        else:
            distance = 0.15 * F / width  # D' = W * F / P

        # log the processing results
        rospy.logdebug("image location: {}\tmagnitude: {}".format(location, magnitude))

        # decide kinematics
        correction = False
        v = 0.4  # m/s

        if distance >= 0.25:
            print "go forward"
            u1 = v / r  # phi_r
            u2 = u1  # phi_l
            last_direction = 0
            is_home = False
        # return
        elif not return_flag:
            rospy.sleep(5)
            return_flag = True
            print "reach target, return"
            print self.init_position
            print self.position
            self.movement_pub.publish(Twist())

        if return_flag:
            print "back to init position"
            vec = [self.init_position[0] - self.position[0], self.init_position[1] - self.position[1]]
            curr_ori = [math.cos(yaw), math.sin(yaw)]
            print "return vec", vec
            print "curr vec",

            angle = self.angle_between(vec, curr_ori)
            print angle

            if angle > 0.2:
                time = 2
                u1 = -angle * l / (time * r)
                u2 = angle * l / (time * r)
            elif angle < -0.2:
                time = 2
                u1 = angle * l / (time * r)
                u2 = -angle * l / (time * r)
            else:
                u1 = 0.4 / r  # phi_r
                u2 = u1  # phi_l

            dis = math.sqrt(
                (self.position[0] - self.init_position[0]) ** 2 + (self.position[1] - self.init_position[1]) ** 2)

            if dis < 0.2:
                is_home = True
                return_flag = False
                self.shutdown()

        if location:
            if (location[0] < - 50 and location[0] > -250) or (location[0] > 50 and location[0] < 250):
                correction = True
            else:
                correction = False

        if correction and not return_flag:
            print "adjust the angular"
            print location, magnitude, width, distance

            drift = copy.deepcopy(location[0]) - 0.0  # init_location[0]

            drift_d = drift * 0.15 / width
            theta = math.atan2(drift, F)

            print drift_d, theta

            time = 4

            u1 += -theta * l / (time * r)
            u2 += theta * l / (time * r)

            if theta > 0:
                last_direction = 2
            else:
                last_direction = 1

        self.spinWheels(u1, u2)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        self.movement_pub.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("lab2_example")
    node = Node()

    # this function loops and returns when the node shuts down
    # all logic should occur in the callback function
    # rospy.spin()

    rospy.spin()
