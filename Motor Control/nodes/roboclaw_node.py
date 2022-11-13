#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf

from math import pi, sin, cos
import serial
import struct
import threading

# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.5}}'

#####################################

def normalizeAngle(angle):
	while angle > pi:
		angle = angle - 2.0*pi
	while angle < -pi:
		angle = angle + 2.0*pi
	return angle
	

class Encoders2Odom:
	def __init__(self, ticks_per_meter, base_width):
		self.TICKS_PER_METER = ticks_per_meter
		self.BASE_WIDTH = base_width
		self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
		self.cur_x = 0
		self.cur_y = 0
		self.cur_theta = 0.0
		self.last_enc_left = 0
		self.last_enc_right = 0
		self.last_enc_time = rospy.Time.now()
	
	def update(self, enc_left, enc_right):
		left_ticks = enc_left - self.last_enc_left
		right_ticks = enc_right - self.last_enc_right
		self.last_enc_left = enc_left
		self.last_enc_right = enc_right
		
		dist_left = left_ticks / self.TICKS_PER_METER
		dist_right = right_ticks / self.TICKS_PER_METER
		dist = (dist_right + dist_left)/2.0
		
		current_time = rospy.Time.now()
		d_time = (current_time - self.last_enc_time).to_sec()
		self.last_enc_time = current_time
		
		if left_ticks == right_ticks:
			d_theta = 0.0
			self.cur_x = self.cur_x + dist * cos(self.cur_theta)
			self.cur_y = self.cur_y + dist * sin(self.cur_theta)
		else:
			d_theta = (dist_right - dist_left) / self.BASE_WIDTH
			R = dist / d_theta
			self.cur_x = self.cur_x + R * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
			self.cur_y = self.cur_y - R * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
			self.cur_theta = normalizeAngle(self.cur_theta + d_theta)
				
		if abs(d_time) < 0.000001:
			vel_x = 0.0
			vel_theta = 0.0
		else:
			vel_x = dist / d_time
			vel_theta = d_theta / d_time
			
		return vel_x, vel_theta
		
	def updateAndPublish(self, enc_left, enc_right):
		# 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
		if abs(enc_left - self.last_enc_left) > 20000:
			rospy.logerr("Ignoring left encoder jump: cur %d, last %d"%(enc_left, self.last_enc_left))
		elif abs(enc_right - self.last_enc_right) > 20000:
			rospy.logerr("Ignoring right encoder jump: cur %d, last %d"%(enc_right, self.last_enc_right))
		else:
			vel_x, vel_theta = self.update(enc_left, enc_right)
			self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)
		
	def publish_odom(self, cur_x,cur_y,cur_theta, vx,vth):
		quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
		current_time = rospy.Time.now()
		
		br = tf.TransformBroadcaster()
		br.sendTransform((cur_x, cur_y, 0),
		                 tf.transformations.quaternion_from_euler(0, 0, cur_theta),
		                 current_time,
		                 "base_footprint",
		                 "odom")
		
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = 'odom'
		
		odom.pose.pose.position.x = cur_x
		odom.pose.pose.position.y = cur_y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation = Quaternion(*quat)
		
		odom.pose.covariance[0]  = 0.01
		odom.pose.covariance[7]  = 0.01
		odom.pose.covariance[14] = 99999
		odom.pose.covariance[21] = 99999
		odom.pose.covariance[28] = 99999
		odom.pose.covariance[35] = 0.01
		
		odom.child_frame_id = 'base_link'
		odom.twist.twist.linear.x = vx
		odom.twist.twist.linear.y = 0
		odom.twist.twist.angular.z = vth
		odom.twist.covariance = odom.pose.covariance
		
		self.odom_pub.publish(odom)
	

#####################################

class Roboclaw:
	
	def __init__(self, device, baudrate_in, address, P, I, D, MAX_QPPS):
		self.ADDRESS = address
		self.checksum = 0
		self.lock = threading.Lock()
		
		self.port = None
		while self.port == None:
			if rospy.is_shutdown():
				raise rospy.ROSInterruptException()
			try:
				self.port = serial.Serial(device, timeout=0.3, writeTimeout=0.3, baudrate=baudrate_in)
			except serial.serialutil.SerialException, e:
				rospy.logerr("Error opening serial connection, will try again. Exception: %s"%e)
				rospy.sleep(1.0)
		
		# try reading something
		self.port.flushInput()
		self.port.flushOutput()
		try:
			print "Try read error state:", self.readErrorState()
		except Exception, e:
			print "Error reading error state: %s"%e
			self.port.flushInput()
			self.port.flushOutput()
			
			# If it doesn't work the second time then let it crash
			print "Try read error state again:", self.readErrorState()
			
		self.last_m1_speed = 0
		self.last_m2_speed = 0
		self.same_speed_sent_times = 0
		
		self.writePID(P, I, D, MAX_QPPS)

		self.resetEncoders()
		self.resetEncoders() # several times, to make sure it gets there
		self.resetEncoders()
		
	def writePID(self, P, I, D, MAX_QPPS):
		P = int(65536 * P)
		I = int(65536 * I)
		D = int(65536 * D)
		
		rospy.loginfo("Specified PIDQ values %i %i %i %i"%(P, I, D, MAX_QPPS))
		
		# If required repeat several times in case there are transmission errors
		for _ in range(4):
			values = self.readM1PID()
			if values == (P, I, D, MAX_QPPS):
				break
			rospy.loginfo("Writing PIDQ to motor 1")
			self.writeM1PID(P, I, D, MAX_QPPS)
			rospy.sleep(0.1)
		values = self.readM1PID()
		if not values == (P, I, D, MAX_QPPS):
			raise Exception("Could not write PID settings for motor 1")

		for _ in range(4):
			values = self.readM2PID()
			if values == (P, I, D, MAX_QPPS):
				break
			rospy.loginfo("Writing PIDQ to motor 2")
			self.writeM2PID(P, I, D, MAX_QPPS)
			rospy.sleep(0.1)
		values = self.readM2PID()
		if not values == (P, I, D, MAX_QPPS):
			raise Exception("Could not write PID settings for motor 2")

	def __del__(self):
		if self.port is not None:
			self.port.close()

	def sendcommand(self, address, command):
		self.checksum = address
		self.port.write(chr(address))
		self.checksum += command
		self.port.write(chr(command))
		return
		
	def writebyte(self, val):
		self.checksum += val
		return self.port.write(struct.pack('>B',val))
		
	def writesword(self, val):
		self.checksum += val
		self.checksum += (val>>8)&0xFF
		return self.port.write(struct.pack('>h',val))
		
	def writeslong(self, val):
		self.checksum += val
		self.checksum += (val>>8)&0xFF
		self.checksum += (val>>16)&0xFF
		self.checksum += (val>>24)&0xFF
		return self.port.write(struct.pack('>l',val))
		
	def SetMixedSpeed(self, m1_speed,m2_speed):
		self.sendcommand(self.ADDRESS,37)
		self.writeslong(m1_speed)
		self.writeslong(m2_speed)
		self.writebyte(self.checksum&0x7F)
		return
		
	def SetMixedDuty(self, m1_speed,m2_speed):
		self.sendcommand(self.ADDRESS,34)
		self.writesword(m1_speed)
		self.writesword(m2_speed)
		self.writebyte(self.checksum&0x7F)
		return
	
	
	def readbyte(self):
		val = struct.unpack('>B',self.port.read(1))
		self.checksum += val[0]
		return val[0]
		
	def readslong(self):
		val = struct.unpack('>l',self.port.read(4))
		self.checksum += val[0]
		self.checksum += (val[0]>>8)&0xFF
		self.checksum += (val[0]>>16)&0xFF
		self.checksum += (val[0]>>24)&0xFF
		return val[0]
		
	def readword(self):
		val = struct.unpack('>H',self.port.read(2))
		self.checksum += (val[0]&0xFF)
		self.checksum += (val[0]>>8)&0xFF
		return val[0]
		
	def readsword(self):
		val = struct.unpack('>h',self.port.read(2))
		self.checksum += (val[0]&0xFF)
		self.checksum += (val[0]>>8)&0xFF
		return val[0]
		
	def readTemperature(self):
		with self.lock:
			self.sendcommand(self.ADDRESS, 82)
			val = self.readword()
			crc = self.checksum&0x7F
			if crc == self.readbyte() & 0x7F:
				return val
			return -1
	
	def readErrorState(self):
		with self.lock:
			self.sendcommand(self.ADDRESS, 90)
			val = self.readword()
			crc = self.checksum&0x7F
			if crc == self.readbyte() & 0x7F:
				return val
			return -1
	
	def readCurrents(self):
		with self.lock:
			self.sendcommand(self.ADDRESS, 49)
			m1 = self.readsword()
			m2 = self.readsword()
			crc = self.checksum&0x7F
			if crc == self.readbyte() & 0x7F:
				return (m1, m2)
			return (-1, -1)
	
	def readEncoder(self, cmd):
		with self.lock:
			self.sendcommand(self.ADDRESS, cmd)
			#c1 = self.checksum
			enc = self.readslong()
			status = self.readbyte()
			#c2 = self.checksum
			crc = self.checksum&0x7F
			packet_checksum = self.readbyte()
			
			#print "checksums %d %d %d p %d"%(c1,c2,crc,packet_checksum & 0x7F)
			
			if crc == packet_checksum & 0x7F:
				return (enc,status)
			return (-1,-1)
			
	def readM1encoder(self):
		return self.readEncoder(16)
	
	def readM2encoder(self):
		return self.readEncoder(17)
	
	def readM1PID(self):
		self.sendcommand(128,55)
		P = self.readslong()
		I = self.readslong()
		D = self.readslong()
		Q = self.readslong()
		crc = self.checksum&0x7F
		if crc==self.readbyte()&0x7F:
			return (P,I,D,Q)
		return (-1,-1,-1,-1)
	
	def readM2PID(self):
		self.sendcommand(128,56)
		P = self.readslong()
		I = self.readslong()
		D = self.readslong()
		Q = self.readslong()
		crc = self.checksum&0x7F
		if crc==self.readbyte()&0x7F:
			return (P,I,D,Q)
		return (-1,-1,-1,-1)
	
	def writeM1PID(self, P,I,D,Q):
		self.sendcommand(self.ADDRESS,28)
		self.writeslong(D)
		self.writeslong(P)
		self.writeslong(I)
		self.writeslong(Q)
		self.writebyte(self.checksum&0x7F)
		return
		
	def writeM2PID(self, P,I,D,Q):
		self.sendcommand(self.ADDRESS,29)
		self.writeslong(D)
		self.writeslong(P)
		self.writeslong(I)
		self.writeslong(Q)
		self.writebyte(self.checksum&0x7F)
		return
		
	def resetEncoders(self):
		self.sendcommand(self.ADDRESS, 20)
		self.writebyte(self.checksum&0x7F)
		return

	def setSpeed(self, m1_speed, m2_speed): # ticks/sec
		if m1_speed==0 and m2_speed==0:
			with self.lock:
				self.SetMixedDuty(0,0) # just set PWM to zero immediately
				self.same_speed_sent_times = 0
		else:
			with self.lock:
				# don't send the same speed values more than 3 times in a row (3 times to make sure it gets there)
				if m1_speed == self.last_m1_speed and m2_speed == self.last_m2_speed:
					if self.same_speed_sent_times >= 3:
						return
					self.same_speed_sent_times += 1
				else:
					self.same_speed_sent_times = 0
				self.SetMixedSpeed(m1_speed,m2_speed) # drive speed with PID
				
		self.last_m1_speed = m1_speed
		self.last_m2_speed = m2_speed
				
######################################

class ReerWheel:
	def __init__(self):
		rospy.init_node('roboclaw_node')
		
		self.paused = False
		
		dev_name = rospy.get_param('~dev','/dev/ttyACM1')
		baud_rate = int(rospy.get_param('~baud','9600'))	
		address = int(rospy.get_param('~address','128'))
		
		self.MAX_SPEED = float(rospy.get_param('~max_speed','2.0')) # m/s
		self.TICKS_PER_METER = float(rospy.get_param('~qp_per_meter'))
		self.BASE_WIDTH = float(rospy.get_param('~base_width'))

		p = float(rospy.get_param('~p','1.0'))
		i = float(rospy.get_param('~i','0.5'))
		d = float(rospy.get_param('~d','0.25'))
		max_qpps = int(rospy.get_param('~max_qpps','20000'))
		
		self.roboclaw = Roboclaw(dev_name, baud_rate, address, p, i, d, max_qpps)
		rospy.on_shutdown(self.shutdown)
		
		self.enc2odom = Encoders2Odom(self.TICKS_PER_METER, self.BASE_WIDTH)
		self.last_set_speed_time = rospy.get_rostime()
		
		rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
		rospy.Subscriber('/pause', Bool, self.pause_callback)
	

	def shutdown(self):
		rospy.loginfo("Stopping the motors...")
		self.roboclaw.setSpeed(0,0)
		self.roboclaw.setSpeed(0,0) # several times, to make sure it gets there
		self.roboclaw.setSpeed(0,0)
		
	def run(self):
		rospy.loginfo('Started')
		
		r = rospy.Rate(10) # hz
		last_stats_time = rospy.get_rostime()
		while not rospy.is_shutdown():
			now = rospy.get_rostime()
			if (now - self.last_set_speed_time).to_sec() > 0.5:
				# Haven't seen cmd_vel messages in a while, let's just stop
				self.roboclaw.setSpeed(0,0)
				self.roboclaw.setSpeed(0,0) # several times, to make sure it gets there
				self.roboclaw.setSpeed(0,0)
				self.last_set_speed_time = now
			try:
				# Wouldn't the encoder values roll over?
				# With the "May" wheels and encoders at the max speed of 2 m/s
				# it generates 76257723 ticks per hour. At this rate 2^31 would overflow in 28 hours.
				# This robot won't run that long at that speed, so we won't be handling overflow for now.
				enc1, status1 = self.roboclaw.readM1encoder()
				if status1 == -1:
					rospy.logerr("readM1encoder error")
					raise struct.error
				enc2, status2 = self.roboclaw.readM2encoder()
				if status2 == -1:
					rospy.logerr("readM2encoder error")
					raise struct.error
				# rospy.loginfo("Encoders %d %d"%(enc1, enc2))
				self.enc2odom.updateAndPublish(enc1, enc2)
			except struct.error:
				self.roboclaw.port.flushInput()
			if (now - last_stats_time).to_sec() > 0.3:
				last_stats_time = now
				try:
					errorState = self.roboclaw.readErrorState()
					temperature = self.roboclaw.readTemperature()
					(m1_current,m2_current) = self.roboclaw.readCurrents()
					# rospy.loginfo("ErrorState:%d, Temperature:%.1f, Currents: %.2f %.2f"%(
					# 	errorState, temperature/10.0, m1_current/100.0, m2_current/100.0) )
				except Exception, e:
					rospy.logerr("Error reading stats: %s"%e)
					self.roboclaw.port.flushInput()
			r.sleep()
			
	def ride(self,linear_x, angular_z, parameter):
		if linear_x >  self.MAX_SPEED: linear_x =  self.MAX_SPEED
		if linear_x < -self.MAX_SPEED: linear_x = -self.MAX_SPEED

		Vr = linear_x + angular_z*self.BASE_WIDTH/2.0 # m/s
		Vl = linear_x - angular_z*self.BASE_WIDTH/2.0

		Vr_ticks = int((Vr * self.TICKS_PER_METER)/parameter) # ticks/s
		Vl_ticks = int((Vl * self.TICKS_PER_METER)/parameter)
		
		print(Vr_ticks)
		print(Vl_ticks)

		self.roboclaw.setSpeed(Vr_ticks ,Vl_ticks)

	
	def cmd_vel_callback(self, twist):
		if not self.paused:
			self.last_set_speed_time = rospy.get_rostime()
			speed = 100

			if (twist.linear.x > 0.4 and twist.linear.x < 0.6 and twist.angular.z < 1.1 and twist.angular.z > 0.9): # left  straight
				print("left  straight")
				self.roboclaw.setSpeed(speed,speed)
			elif (twist.linear.x > 0.4 and twist.linear.x < 0.6 and twist.angular.z < -0.9 and twist.angular.z > -1.1): # right straight
				print("right  straight")
				self.roboclaw.setSpeed(speed,speed)
			elif (twist.linear.x < -0.4 and twist.linear.x > -0.6 and twist.angular.z < -0.9 and twist.angular.z > -1.1): # left backward
				print("left backward")
				self.roboclaw.setSpeed(-speed,-speed)
			elif (twist.linear.x < -0.4 and twist.linear.x > -0.6 and twist.angular.z > 0.9 and twist.angular.z < 1.1): # right backward
				print("right backward")
				self.roboclaw.setSpeed(-speed,-speed)
			elif (twist.linear.x > 0.4 and twist.linear.x < 0.6 and twist.angular.z > -0.1 and twist.angular.z < 0.1): #  forward
				print("forward")
				self.roboclaw.setSpeed(speed,speed)
			elif (twist.linear.x < -0.4 and twist.linear.x > -0.6 and twist.angular.z > -0.1 and twist.angular.z < 0.1): #  backward
				print("backward")
				self.roboclaw.setSpeed(-speed,-speed)
			else:
				print("stop")
				self.roboclaw.setSpeed(0 ,0)
			
		
		
	def pause_callback(self, msg):
		self.paused = msg.data
		rospy.loginfo("Got pause message: %s"%self.paused)
		if self.paused:
			self.roboclaw.setSpeed(0,0)
			self.roboclaw.setSpeed(0,0) # several times, to make sure it gets there
			self.roboclaw.setSpeed(0,0)

######################################

class SteeringWheel:
	def __init__(self):
		rospy.init_node('roboclaw_node')
		print("steering wheel initialized")
		self.paused = False
		
		dev_name = rospy.get_param('~dev2','/dev/ttyACM0')
		baud_rate = int(rospy.get_param('~baud','9600'))	
		address = int(rospy.get_param('~address','128'))
		
		self.MAX_SPEED = float(rospy.get_param('~max_speed','2.0')) # m/s
		self.TICKS_PER_METER = float(rospy.get_param('~qp_per_meter'))
		self.BASE_WIDTH = float(rospy.get_param('~base_width'))

		p = float(rospy.get_param('~p','1.0'))
		i = float(rospy.get_param('~i','0.5'))
		d = float(rospy.get_param('~d','0.25'))
		max_qpps = int(rospy.get_param('~max_qpps','20000'))
		
		self.roboclaw = Roboclaw(dev_name, baud_rate, address, p, i, d, max_qpps)
		rospy.on_shutdown(self.shutdown)
		
		self.enc2odom = Encoders2Odom(self.TICKS_PER_METER, self.BASE_WIDTH)
		self.last_set_speed_time = rospy.get_rostime()
		
		rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
		rospy.Subscriber('/pause', Bool, self.pause_callback)
	

	def shutdown(self):
		rospy.loginfo("Stopping the motors...")
		self.roboclaw.setSpeed(0,0)
		self.roboclaw.setSpeed(0,0) # several times, to make sure it gets there
		self.roboclaw.setSpeed(0,0)
		
	def run(self):
		rospy.loginfo('Started')
		
		r = rospy.Rate(10) # hz
		last_stats_time = rospy.get_rostime()
		while not rospy.is_shutdown():
			now = rospy.get_rostime()
			if (now - self.last_set_speed_time).to_sec() > 0.5:
				# Haven't seen cmd_vel messages in a while, let's just stop
				self.roboclaw.setSpeed(0,0)
				self.roboclaw.setSpeed(0,0) # several times, to make sure it gets there
				self.roboclaw.setSpeed(0,0)
				self.last_set_speed_time = now
			try:
				# Wouldn't the encoder values roll over?
				# With the "May" wheels and encoders at the max speed of 2 m/s
				# it generates 76257723 ticks per hour. At this rate 2^31 would overflow in 28 hours.
				# This robot won't run that long at that speed, so we won't be handling overflow for now.
				enc1, status1 = self.roboclaw.readM1encoder()
				if status1 == -1:
					rospy.logerr("readM1encoder error")
					raise struct.error
				enc2, status2 = self.roboclaw.readM2encoder()
				if status2 == -1:
					rospy.logerr("readM2encoder error")
					raise struct.error
				# rospy.loginfo("Encoders %d %d"%(enc1, enc2))
				self.enc2odom.updateAndPublish(enc1, enc2)
			except struct.error:
				self.roboclaw.port.flushInput()
			if (now - last_stats_time).to_sec() > 0.3:
				last_stats_time = now
				try:
					errorState = self.roboclaw.readErrorState()
					temperature = self.roboclaw.readTemperature()
					(m1_current,m2_current) = self.roboclaw.readCurrents()
					# rospy.loginfo("ErrorState:%d, Temperature:%.1f, Currents: %.2f %.2f"%(
					# 	errorState, temperature/10.0, m1_current/100.0, m2_current/100.0) )
				except Exception, e:
					rospy.logerr("Error reading stats: %s"%e)
					self.roboclaw.port.flushInput()
			r.sleep()
			
	def ride(self,linear_x, angular_z, parameter):

		if linear_x >  self.MAX_SPEED: linear_x =  self.MAX_SPEED
		if linear_x < -self.MAX_SPEED: linear_x = -self.MAX_SPEED

		Vr = linear_x + angular_z*self.BASE_WIDTH/2.0 # m/s
		Vl = linear_x - angular_z*self.BASE_WIDTH/2.0

		Vr_ticks = int((Vr * self.TICKS_PER_METER)/parameter) # ticks/s
		Vl_ticks = int((Vl * self.TICKS_PER_METER)/parameter)
		
		print(Vr_ticks)
		print(Vl_ticks)

		self.roboclaw.setSpeed(Vr_ticks ,Vl_ticks)

	def cmd_vel_callback(self, twist):
		if not self.paused:
			# print("callback started")
			self.last_set_speed_time = rospy.get_rostime()
			speed = 200

			if (twist.linear.x > 0.4 and twist.linear.x < 0.6 and twist.angular.z < 1.1 and twist.angular.z > 0.9): # left  straight
				print("left  straight")
				self.roboclaw.setSpeed(-speed,-speed)
			elif (twist.linear.x > 0.4 and twist.linear.x < 0.6 and twist.angular.z < -0.9 and twist.angular.z > -1.1): # right straight
				print("right  straight")
				self.roboclaw.setSpeed(speed,speed)
			elif (twist.linear.x < -0.4 and twist.linear.x > -0.6 and twist.angular.z < -0.9 and twist.angular.z > -1.1): # left backward
				print("left backward")
				self.roboclaw.setSpeed(-speed,-speed)
			elif (twist.linear.x < -0.4 and twist.linear.x > -0.6 and twist.angular.z > 0.9 and twist.angular.z < 1.1): # right backward
				print("right backward")
				self.roboclaw.setSpeed(speed,speed)
			elif (twist.linear.x < 0.1 and twist.linear.x > -0.1 and twist.angular.z > 0.9 and twist.angular.z < 1.1): #  left turn 
				print("left turn")
				self.roboclaw.setSpeed(-speed,-speed)
			elif (twist.linear.x < 0.1 and twist.linear.x > -0.1 and twist.angular.z < -0.9 and twist.angular.z > -1.1): #  right turn 
				print("right turn")
				self.roboclaw.setSpeed(speed,speed)
			else : 
				print("stop")
				self.roboclaw.setSpeed(0,0)

			
		
		
	def pause_callback(self, msg):
		self.paused = msg.data
		rospy.loginfo("Got pause message: %s"%self.paused)
		if self.paused:
			self.roboclaw.setSpeed(0,0)
			self.roboclaw.setSpeed(0,0) # several times, to make sure it gets there
			self.roboclaw.setSpeed(0,0)



if __name__=="__main__":
	try:
		# reer = ReerWheel()
		steer = SteeringWheel()
		# reer.run()
		steer.run()
		
	except rospy.ROSInterruptException:
		pass
	
	rospy.loginfo("Exiting...")
