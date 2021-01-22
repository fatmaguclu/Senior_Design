#!/usr/bin/python
# -*- coding: utf-8 -*-
from pyzbar import pyzbar
import argparse
import datetime
import rospy
import math
import struct
import numpy as np
import cv2
from random import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# UWB
import serial
import socket
from std_msgs.msg import String
# VRPN
from geometry_msgs.msg import Point, PoseStamped,TwistStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import *
from mavros_msgs.srv import *

PORT_UWB = '/dev/ttyACM0'

##### Global Parameters ######
bridge = CvBridge()	# Bridge Function Between OpenCV and ROS
FREQ = 20.0			# Frequency
TS = 1.0/FREQ 		# Time

img_topic = '/raspicam_node/image_raw'		
vicon_topic= '/vrpn_client_node/SDP20/pose'	

vx, vy = -0., -0.	# Velocities


##########################
##########################
##########################
class DroneLocalization:
	def __init__(self,rate,initial):
		self.ser = serial.Serial(PORT_UWB, 115200, timeout=1.0)
		#self.vicon.cb = rospy.Subscriber( TOPIC_VICON, PositionTarget, self.vicon)
		self.imgCb = rospy.Subscriber('/raspicam_node/image_raw', Image, self.imageCb)
		self.initial_pos = initial
		self.vicon_cb = rospy.Subscriber(vicon_topic, PositionTarget, self.vicon)
		### Publish ###
		self.sp_position = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1) ## Setpoint
		self.pub_ekf = rospy.Publisher('/ekf', Twist, queue_size=1) ## Setpoint
		self.pub_uwb = rospy.Publisher('/uwb', float32, queue_size=1) ## Setpoint
		self.pub_qr_raw = rospy.Publisher('/qr_raw', Twist, queue_size=1) ## Setpoint
		self.pub_qr_det = rospy.Publisher('/qr_det', Twist, queue_size=1) ## Setpoint
		# Parameters
		self.d1 = 0.0
		self.img = 0.0
		self.X_anchor = 0.35
		self.Y_anchor = 3.0
		self.X = np.zeros((2,1)) # EKF state at time t
		self.R = np.zeros((2,2)) # 2x2 motion model covariance
		self.R[0,0], self.R[1,1] = 0.04, 0.04
		self.Vel = np.zeros((2,1))
		self.nu = np.zeros((2,1))
		self.nup = np.zeros((2,1))
		self.Gt = [[1.0, 0.0], [0.0, 1.0]]
		self.g = np.zeros((2,1))
		self.Y = np.zeros((3,1))
		self.Q = np.zeros((3,3)) # 3x3 measurement covariance
		self.Q[0,0], self.Q[1,1], self.Q[2,2] = 0.01, 0.01, 0.01
		self.h = np.zeros((3,1))
		self.hnup = np.zeros((3,1))
		self.Ht = np.zeros((3,2))
		self.Sp = np.zeros((2,2))
		self.S = np.zeros((2,2))
		self.K = np.zeros((2,3))
		# Camera parameters
		self.h_drone = 1.2
		self.f = 3.6*0.001
		self.px = 3.76*0.001/640
		self.py = 2.74*0.001/480
		self.x_bar, self.y_bar = 0.0, 0.0
		### Publish params ###
		self.ekf_pub = Twist()
		self.uwb_pub = 0.0
		self.qrRaw_pub = Twist()
		self.qrDet_pub = Twist()
		# Construct the argument parser and parse the arguments
		ap = argparse.ArgumentParser()
		ap.add_argument("-o", "--output", type=str, default="barcodes.csv",help="path to output CSV file containing barcodes")
		args = vars(ap.parse_args())
		self.csv = open(args["output"], "w")
		self.found = set()
		print('Initialization done...')

	##########################
	##########################
	def vicon(self,msg):
		if not msg == None:
			# Position callback
			self.pose_Current_x = msg.pose.position.x
			self.pose_Current_y = msg.pose.position.y
			self.pose_Current_z = msg.pose.position.z
			quater = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quater)
			self.yaw = euler[2]

	##########################
	##########################
	def imageCb(self,msg):
		# Read image
		self.img = bridge.imgmsg_to_cv2(msg, 'bgr8')
		#print('aaaaaaaaaa')

	##########################
	##########################
	def imageProcessing(self):
		# loop over the frames from the video stream
		barcodes = pyzbar.decode(self.img)
		# loop over the detected barcodes
		for barcode in barcodes:
			# extract the bounding box location of the barcode and draw
			# the bounding box surrounding the barcode on the image
			(x, y, w, h) = barcode.rect
			# Publish vars
			self.qrRaw_pub.linear.x = x
			self.qrRaw_pub.linear.y = y
			self.qrRaw_pub.linear.z = w
			self.qrRaw_pub.angular.x = h
			#cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
			# on our output image we need to convert it to a string first
			barcodeData = barcode.data.decode("utf-8")
			x_qr=int(barcodeData[0:2])
			y_qr=int(barcodeData[2:4])
			#camera pixel(640,480)
			i = x+(w/2)-320
			j = -(y+(h/2))+240
			#real location estimation
			#self.x_bar = x_qr + i*self.px*(self.f-self.h_drone)/(self.f)
			#self.y_bar = y_qr + j*self.py*(self.f-self.h_drone)/(self.f) - 0.025
			self.x_bar = x_qr + j*self.py*(self.f-self.h_drone)/(self.f) - 0.025
			self.y_bar = y_qr - i*self.px*(self.f-self.h_drone)/(self.f)
			# Estiated position from QR code in the image
			print("Estimation:", self.x_bar, ",", self.y_bar)
			# if the barcode text is currently not in our CSV file, write
			#the timestamp + barcode to disk and update the set
			if barcodeData not in self.found:
				self.csv.write("{},{}\n".format(datetime.datetime.now(),barcodeData))
				self.csv.flush()
				self.found.add(barcodeData)
		### Image failed ###
		#print('Image not detected')
		#cv2.imshow('raspicam_node motion vectors visualization', self.img)
		#cv2.waitKey(1)
	
	##########################
	##########################
	def uwb(self):
		for x in xrange(0,2):
			data = self.ser.readline()
			dist = -1
			if data:
				try:
					data = data[:-2]
					values = data.split(',')
					a = values[0][0:2]
					if a == 'mc':
						x1 = values[0][6:14]
						x2 = values[0][15:23]
						x3 = values[0][24:32]
					# Distances between tag and anchors
					self.d1 = int(x1,16)/1000.	# Anchor 0
					d2 = int(x2,16)/10	# Anchor 1
					d3 = int(x3,16)/10	# Anchor 2
					#print('UWB: ', str(d1)) #, '+str(d2)+', '+str(d3))
					#dist = float(values[0])
				except:
					pass

	##########################
	##########################
	def ekf(self):
		self.Vel[0,0] = vx
		self.Vel[1,0] = vy
		# Measurement Model
		self.Y[0,0] = self.x_bar
		self.Y[1,0] = self.y_bar
		self.Y[2,0] = self.d1 
		# Prediction Update
		self.nup = self.nup + self.Vel*TS #PREDICTED MEAN 
		self.Sp = self.S + self.R #PREDICTED COVARIANCE
		# Linearized Measurement Model 
		dist_pred = math.sqrt(pow(self.nup[0,0]-self.X_anchor,2) + pow(self.nup[1,0]-self.Y_anchor,2))
		self.Ht = [[1.0,0.0], [0.0,1.0], [(self.nup[0,0]-self.X_anchor)/dist_pred , (self.nup[1,0]-self.Y_anchor)/dist_pred] ]
		self.hnup[0,0] = self.nup[0,0]
		self.hnup[1,0] = self.nup[1,0]
		self.hnup[2,0] = dist_pred
		# Measurement Update
		self.K = np.matmul(np.matmul(self.Sp, np.transpose(self.Ht)), np.linalg.inv(np.matmul(self.Ht, np.matmul(self.Sp, np.transpose(self.Ht))) + self.Q))
		self.nu = self.nup + np.matmul(self.K,(self.Y-self.hnup))
		self.S = np.matmul((np.identity(2) - np.matmul(self.K, self.Ht)) , self.Sp)

	##########################
	##########################
	def generatePosition(self,px,py,z=0.9,yaw=0.):
		''' instantiate setpoint msg '''
		self.sp_pose = PositionTarget()
		self.sp_pose.position.x = px
		self.sp_pose.position.y = py
		self.sp_pose.position.z = 0.9
		#self.sp_pose.velocity.x = vx
		#self.sp_pose.velocity.y = vy
		#self.sp_pose.velocity.z = vz
		self.sp_pose.yaw =  0.
		#self.sp_pose.yaw_rate = yaw # the yaw rate
		'''
		# Velocity in body frame [vx,vy,pz,omega]
		self.sp_pose.type_mask = PositionTarget.IGNORE_AFX
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_AFY
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_AFZ
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_YAW
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_PX
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_PY
		'''

		# Position in Local NED frame [px,py,pz,yaw]
		# Set self.sp_pose.coordinate_frame = 0 or do not set to any value (default)
		self.sp_pose.type_mask = PositionTarget.IGNORE_AFX
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_AFY
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_AFZ
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_YAW_RATE
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_VX
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_VY
		self.sp_pose.type_mask = self.sp_pose.type_mask + PositionTarget.IGNORE_VZ
		
		
		''' SP coord. frame uint8
		FRAME_LOCAL_NED = 1
		FRAME_LOCAL_OFFSET_NED = 7
		FRAME_BODY_NED = 8
		FRAME_BODY_OFFSET_NED = 9
		'''
		self.sp_pose.coordinate_frame = 0 # Body NED
		return sp_pose


	##########################
	##########################
	def pub_vars(self):
		# Publish rosbag data
		self.ekf_pub.linear.x = self.nu[0,0]
		self.ekf_pub.linear.y = self.nu[1,0]
		self.uwb_pub = self.d1
		self.qrDet_pub.linear.x = self.x_bar
		self.qrDet_pub.linear.y = self.y_bar
		# Publish
		self.pub_ekf.publish(self.ekf_pub)
		self.pub_uwb.publish(self.uwb_pub)
		self.pub_qr_raw.publish(self.qrRaw_pub)
		self.pub_qr_det.publish(self.qrDet_pub)




##########################
##########################
##########################
def main():
	rospy.init_node('main_node', anonymous=True)
	rate = rospy.Rate(FREQ)
	initial_pos = [[2.0,2.0,3.,0.]]
	cnt = DroneLocalization(rate,initial_pos)
	ctr = 0

	while not rospy.is_shutdown():
		now = rospy.get_rostime()
		ctr += 1
		if ctr > 10:
			cnt.imageProcessing()
		cnt.uwb()
		cnt.ekf()
		if ctr <= FREQ*10.:
			cnt.generatePosition(-0.1,-0.1,0.9,0.)
		elif FREQ*10. < ctr <= FREQ*20.:
			cnt.generatePosition(-0.1,0.1,0.9,0.)
		elif FREQ*20. < ctr <= FREQ*30.:
			cnt.generatePosition(0.1,0.1,0.9,0.)
		elif FREQ*30. < ctr <= FREQ*40.:
			cnt.generatePosition(0.1,-0.1,0.9,0.)
		else:
			cnt.generatePosition(0.0,0.0,0.0,0.)
		cnt.sp_position.publish(cnt.sp_pose)
		# Monitor
		print(cnt.x_bar, cnt.y_bar, 'UWB: ', cnt.d1, 'EKF: ', cnt.nu[0,0], cnt.nu[1,0])
		# Fill publish params
		cnt.pub_vars()
		rate.sleep()

##########################
##########################
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass