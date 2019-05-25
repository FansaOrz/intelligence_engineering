#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import chardet
import encodings
from sensor_msgs.msg import PointCloud, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
import random

class depth2point():
    def __init__(self):
        rospy.init_node("depth2point")
        self.image_sub = rospy.Subscriber("/pepper_robot/naoqi_driver/camera/depth/image_raw", Image, self.image_callback)
        self.info_sub = rospy.Subscriber("/pepper_robot/naoqi_driver/camera/depth/camera_info", CameraInfo, self.info_callback)
        self.point_pub = rospy.Publisher("/points", PointCloud, queue_size=15)
        self.bridge = CvBridge()
        self.camera_factor = None
        self.camera_cx = None
        self.camera_cy = None
        self.camera_fx = None
        self.camera_fy = None
        self.height = None
        self.width = None
        self.skip_num = 0
        self.cloud_point = PointCloud()
        self.cloud_point.header.frame_id = "CameraDepth_frame"
        rospy.spin()

    def depth_image_to_point(self, image):

        # print "depth2point"
        for i in range(240):
            for j in range(320):#160 470
                if image[i][j] == 0:
                    continue
                temp = Point()
                temp.z = float(image[i][j]) / self.camera_factor
                temp.x = (j - self.camera_cx)*temp.z / self.camera_fx
                temp.y = (i - self.camera_cy)*temp.z / self.camera_fy
                # temp.r = random.randint(0, 255)
                # temp.r = random.randint(0, 255)
                # temp.r = random.randint(0, 255)
                self.cloud_point.points.append(temp)
        self.point_pub.publish(self.cloud_point)
        self.cloud_point = PointCloud()
        self.cloud_point.header.frame_id = "CameraDepth_frame"

    def image_callback(self, msg):
        # print msg.header
        # print msg[0]
        # x = msg.data
        # print type(msg)
        # self.depth_image_to_point(msg)
        image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        if self.skip_num <=2:
            self.skip_num += 1
            self.depth_image_to_point(image)
        else:
            self.skip_num = 0

    def info_callback(self, msg):
        # print msg
        self.width = msg.width
        self.height = msg.height
        self.camera_factor = 1000
        self.camera_fx = msg.K[0]
        self.camera_cx = msg.K[2]
        self.camera_fy = msg.K[4]
        self.camera_cy = msg.K[5]
        self.info_sub.unregister()

class Point():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0

if __name__ == '__main__':
    d2p = depth2point()
