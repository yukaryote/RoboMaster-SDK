#!/usr/bin/env python3

from gc import callbacks
from mmap import ALLOCATIONGRANULARITY
import cv2
from robomaster import robot
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import cv_bridge
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header


class ImuImagePub():
    def __init__(self) -> None:
        self.img_pub = rospy.Publisher("/robomaster/images", Image)
        self.imu_pub = rospy.Publisher("/robomaster/imu", Imu)
        self.bridge = cv_bridge.CvBridge()
        self.robot = robot.Robot()
        self.robot.initialize(conn_type="ap")
        self.chassis = self.robot.chassis
        self.robot.gimbal.recenter().wait_for_completed()
        self.robot.gimbal.suspend()
        self.imu_info = None
        self.attitude_info = None
        
        self.cam = self.robot.camera

        self.cam.start_video_stream(display=False)

        while not rospy.is_shutdown():
            img = self.cam.read_cv2_image()
            print(img.shape)
            self.publish(img)
        self.robot.close()

    def publish(self, img):
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.chassis.sub_imu(freq=10, callback=self.sub_imu_info_handler)
        self.chassis.sub_attitude(freq=10, callback=self.sub_attitude_info_handler)

        orientation = quaternion_from_euler(self.attitude_info[2], self.attitude_info[1], self.attitude_info[0])
        angular_velocity = Vector3(self.imu_info[3], self.imu_info[4], self.imu_info[5])
        linear_acc = Vector3(self.imu_info[0], self.imu_info[1], self.imu_info[2])

        # TODO: find IMU covariances
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "~base_link"
        imu_msg.linear_acceleration = linear_acc
        imu_msg.orientation = orientation
        imu_msg.angular_velocity = angular_velocity

        self.img_pub.publish(image_msg)
        self.imu_pub.publish(imu_msg)

    def sub_imu_info_handler(self, imu_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
        self.imu_info = imu_info
        print("chassis imu: acc_x:{0}, acc_y:{1}, acc_z:{2}, gyro_x:{3}, gyro_y:{4}, gyro_z:{5}".format(
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z))

    def sub_attitude_info_handler(self, attitude_info):
        yaw, pitch, roll = attitude_info
        self.attitude_info = attitude_info
        print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))


if __name__ == '__main__':
    rospy.init_node("robot_pub")
    robot_pub = ImuImagePub()
    rospy.spin()
