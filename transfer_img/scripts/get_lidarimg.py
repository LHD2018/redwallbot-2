#!/usr/bin/env python2

import sys

import yaml

import cv_bridge

from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,Image
import sensor_msgs.point_cloud2 as pc2

def scale2255(a, min, max, dtype=np.uint8):
	return (((a - min)/ float(max - min))*255).astype(dtype)


def point_cloud_2_birdseye(points):

		side_range=(-5, 5)
		fwd_range=(-10, 10)
		h_range=(-0.9, 2)

		x_points = points[:, 0]
		y_points = points[:, 1]
		z_points = points[:, 2]

		f_filt = np.logical_and((x_points > fwd_range[0]), (x_points < fwd_range[1]))
		s_filt = np.logical_and((y_points > side_range[0]), (y_points < side_range[1]))
		h_filt = np.logical_and((z_points > h_range[0]), (z_points <h_range[1] ))
		fs_filt = np.logical_and(f_filt, s_filt)
		filter = np.logical_and(fs_filt, h_filt)
		indices = np.argwhere(filter)

		x_points = x_points[indices]
		y_points = y_points[indices]
		z_points = z_points[indices]

		res = 0.05

		x_img = (-y_points / res).astype(np.int32)
		y_img = (-x_points / res).astype(np.int32)

		x_img -=int(np.floor(side_range[0] / res))
		y_img +=int(np.ceil(fwd_range[1] / res))


		xy_distances = pow(pow(x_points, 2) + pow(y_points,2), 0.5)
		xy_range=(0.5, 5)
		pixel_values = np.clip(xy_distances, a_min=xy_range[0], a_max=xy_range[1])
		pixel_values = scale2255(pixel_values, min=xy_range[0], max=xy_range[1])

		x_max = 1 + int((side_range[1] - side_range[0]) / res)
		y_max = 1 + int((fwd_range[1] - fwd_range[0])/ res)

		im=np.zeros([y_max, x_max,3],dtype=np.uint8)
		im[y_img, x_img,0] = 125
		im[y_img, x_img,1] = pixel_values
		im[y_img, x_img,2] = 255 - pixel_values

		return im

class pt2brid_eye:
	def __init__(self):
		rospy.init_node('get_lidarimg')
		self.image_pub = rospy.Publisher('/lidar_img', Image, queue_size=10)
		self.pt_sub=rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
		self.bridge = CvBridge()

	def callback(self,lidar):
		lidar = pc2.read_points(lidar)
		points = np.array(list(lidar))
		im = point_cloud_2_birdseye(points)  # im is a numpy array

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(im, "bgr8"))


if __name__ == '__main__':
	
	pt2img=pt2brid_eye()
	rospy.spin()
