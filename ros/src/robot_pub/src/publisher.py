#!/usr/bin/env python3

import cv2
from robomaster import robot
import rospy
from sensor_msgs.msg import Image
# from sensor_msgs.msg import Imu
import cv_bridge


class ImuImagePub():
    def __init__(self) -> None:
        self.img_pub = rospy.Publisher("/robomaster/images", Image)
        # self.imu_pub = rospy.Publisher("/robomaster/imu", Imu)
        self.bridge = cv_bridge.CvBridge()
        self.robot = robot.Robot()
        self.robot.initialize(conn_type="ap")
        self.chassis = self.robot.chassis
        self.robot.gimbal.recenter()
        self.robot.gimbal.suspend()
        
        self.cam = self.robot.camera

        self.cam.start_video_stream(display=False)

        while True:
            img = self.cam.read_cv2_image()
            self.publish(img)

    def publish(self, img):
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.img_pub.publish(image_msg)


if __name__ == '__main__':
    rospy.init_node("robot_pub")
    robot_pub = ImuImagePub()
    ep_robot = robot.Robot()
    rospy.spin()
