#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int64, Float64MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import time

def pressure_callback(data):
  global depth_val
  depth_val = round((data.fluid_pressure-101.5)*0.1023,2)

def imu_callback(data):
	global quat, roll, pitch, yaw, last_z_vec, last_roll, last_pitch
	orientation_quat = data.orientation
	quat = (orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
	roll = 0
	pitch = 0
	# print(["%.2f" % elem for elem in R.from_quat(quat).as_euler('xyz',degrees=True)],'1')
	# print(["%.2f" % elem for elem in R.from_quat(quat).as_euler('xzy',degrees=True)],'2')
	# print(["%.2f" % elem for elem in R.from_quat(quat).as_euler('yxz',degrees=True)],'3')
	# print(["%.2f" % elem for elem in R.from_quat(quat).as_euler('yzx',degrees=True)],'4')
	# print(["%.2f" % elem for elem in R.from_quat(quat).as_euler('zxy',degrees=True)],'5')
	# print(["%.2f" % elem for elem in R.from_quat(quat).as_euler('zyx',degrees=True)],'6')
	roll,pitch,yaw = R.from_quat(quat).as_euler('xyz',degrees=True)
	# q0=quat[0]
	# q1=quat[1]
	# q2=quat[2]
	# q3=quat[3]

	# roll=math.degrees(-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)))
	# pitch=math.degrees(math.asin(2*(q0*q2-q3*q1)))
	# yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
	#r = R.from_quat(quat).as_matrix()
	# rz_vec = (-r[1][1],r[1][2])
	# roll = math.degrees(math.acos(np.dot(rz_vec,(1,0))/(math.sqrt(sum(pow(element, 2) for element in rz_vec)))))
	# if rz_vec[0]>0:
	# 	roll=-1*roll
	# ry_vec = (-r[0][0],r[0][2])
	# pitch = math.degrees(math.acos(np.dot(ry_vec,(1,0))/(math.sqrt(sum(pow(element, 2) for element in ry_vec)))))
	# if ry_vec[0]>0:
	# 	pitch=-1*pitch
	# z_vec = (r[2][0],r[2][1],r[2][2])
	# ch_z = math.degrees(math.acos(np.dot(z_vec,last_z_vec)/(math.sqrt(sum(pow(element, 2) for element in z_vec)))))
	# last_z_vec = z_vec
	# if ch_z<0.1:
	# 	roll = last_roll
	# 	pitch = last_pitch
	# last_roll = roll
	# last_pitch = pitch
	# print('tr',ch_z)
	#(_, _, roll) = R.from_quat(quat).as_euler('zyx',degrees=True)
	#(pitch, _, _) = R.from_quat(quat).as_euler('yzx',degrees=True)
	#print(int(pitch),int(roll))

def move_rov(surge,yaw,sway,roll,pitch,heave):
	surge = round(surge,3)
	yaw = round(yaw,3)
	sway = round(sway,3)
	roll = round(roll,3)
	pitch = round(pitch,3)
	heave = round(heave,3)
	if abs(surge)>1:
		surge = surge/abs(surge)
	if abs(yaw)>1:
		yaw = yaw/abs(yaw)
	if abs(sway)>1:
		sway = sway/abs(sway)
	if abs(roll)>1:
		roll = roll/abs(roll)
	if abs(pitch)>1:
		pitch = pitch/abs(pitch)
	if abs(heave)>1:
		heave = heave/abs(heave)
	global sim_pub
	msg = WrenchStamped()
	msg.header.stamp = rospy.Time.now()  
	msg.wrench.force.x = surge*(5000)
	msg.wrench.force.y = sway*(5000)
	msg.wrench.force.z = heave*(-3000)
	msg.wrench.torque.x = roll*(-20000)
	msg.wrench.torque.y = pitch*(50000)
	msg.wrench.torque.z = yaw*(-6000)
	control_pub.publish(Float64MultiArray(data=[surge,yaw,sway,roll,pitch,heave]))
	sim_pub.publish(msg)


def thruster_callback(msg):
	global thruster_pub
	msg.data = msg.data/400 
	thruster_pub.publish(msg)

def position_hold(surge_val, yaw_val, sway_val,  roll_val, pitch_val, heave_val):
	global roll, pitch, yaw, roll_last_e, roll_i, pitch_last_e, pitch_i, roll_last_time, pitch_last_time, yaw_last_e, yaw_i, yaw_last_time, roll_setpoint, pitch_setpoint, yaw_setpoint
	if not surge_val+yaw_val+sway_val+roll_val+pitch_val+heave_val==0:
		#print('set position')
		move_rov(surge=surge_val, yaw=yaw_val, sway=sway_val, roll= roll_val, pitch=pitch_val, heave=heave_val)
		roll_setpoint = math.radians(roll)
		pitch_setpoint = math.radians(pitch)
		yaw_setpoint = math.radians(yaw)
		limit = math.radians(45)
		if abs(roll_setpoint)>limit:
			roll_setpoint=None
		# elif roll_setpoint<-limit:
		# 	roll_setpoint=-limit
		if abs(pitch_setpoint)>limit:
			pitch_setpoint=None
		# elif pitch_setpoint<-limit:
		# 	pitch_setpoint=-limit
		# if yaw_setpoint>limit:
		# 	yaw_setpoint=limit
		# elif yaw_setpoint<-limit:
		# 	yaw_setpoint=-limit
	else:
		if not (roll_setpoint==None or pitch_setpoint==None):
			roll = math.radians(roll)
			pitch = math.radians(pitch)
			yaw = math.radians(yaw)
			kp=3
			kd=0.25
			ki=0.15
			roll_e = roll_setpoint-roll
			roll_d = (roll_e-roll_last_e)/(time.time()-roll_last_time)
			roll_last_time = time.time()
			roll_i = roll_i+roll_e
			roll_cf = kp*roll_e + kd*roll_d + ki*roll_i
			roll_last_e = roll_e
			kp=3
			kd=0.25
			ki=0.15
			pitch_e = pitch_setpoint-pitch
			pitch_d = (pitch_e-pitch_last_e)/(time.time()-pitch_last_time)
			pitch_last_time = time.time()
			pitch_i = pitch_i+pitch_e
			pitch_cf = kp*pitch_e + kd*pitch_d + ki*pitch_i
			pitch_last_e = pitch_e
			kp=1
			kd=0.05
			ki=0.1
			yaw_e = yaw_setpoint-yaw
			yaw_d = (yaw_e-yaw_last_e)/(time.time()-yaw_last_time)
			yaw_last_time = time.time()
			yaw_i = yaw_i+yaw_e
			yaw_cf = kp*yaw_e + kd*yaw_d + ki*yaw_i
			yaw_last_e = yaw_e
			#print(yaw, yaw_cf, yaw_e, yaw_d, yaw_i)
			move_rov(pitch= pitch_cf, roll= roll_cf, surge=0, yaw=yaw_cf, sway=0, heave=0)
		else:
			#print('Invalid Setpoint')
			move_rov(surge=0, yaw=0, sway=0, roll= 0, pitch=0, heave=0)

def stabilize(surge_val, yaw_val, sway_val, heave_val):
	global roll, pitch, roll_last_e, roll_i, pitch_last_e, pitch_i, roll_last_time, pitch_last_time, depth_val, depth_last_e, depth_last_time, depth_e,depth_i, depth_setpoint
	#STABILIZE ROLL
	roll = math.radians(roll)
	setpoint = 0
	roll_e = setpoint-roll
	kp=3
	kd=0.25
	ki=0.15
	roll_d = (roll_e-roll_last_e)/(time.time()-roll_last_time)
	roll_last_time = time.time()
	roll_i = roll_i+roll_e
	roll_cf = kp*roll_e + kd*roll_d + ki*roll_i
	roll_last_e = roll_e
	#STABILIZE PITCH
	pitch = math.radians(pitch)
	pitch_e = setpoint-pitch
	# if abs(pitch_e)<0.01:
	# 	if pitch_e<0:
	# 		pitch_e=-0.0001
	# 	else:
	# 		pitch_e=0.0001
	kp=3
	kd=0.25
	ki=0.15
	pitch_d = (pitch_e-pitch_last_e)/(time.time()-pitch_last_time)
	pitch_last_time = time.time()
	pitch_i = pitch_i+pitch_e
	pitch_cf = kp*pitch_e + kd*pitch_d + ki*pitch_i
	pitch_last_e = pitch_e
	#print('f',pitch,pitch_cf,pitch_e,pitch_d,pitch_i)
	#STABILIZE DEPTH
	if not abs(heave_val)<0.1:
		depth_setpoint = depth_val
		heave_cf=heave_val
	else:
		print(depth_setpoint)
		depth_e =  depth_val - depth_setpoint
		kp = 1
		kd = 0#.02
		ki = 0.01
		depth_d = (depth_e-depth_last_e)/(time.time()-depth_last_time)
		depth_last_time = time.time()
		depth_i = depth_i+depth_e
		heave_cf = kp*depth_e + kd*depth_d + ki*depth_i
		depth_last_e = depth_e
	#APPLY CORRECTIVE FORCES
	move_rov(pitch= pitch_cf, roll= roll_cf, surge=surge_val, yaw=yaw_val, sway=sway_val, heave=heave_cf)

def joy_s(data):
	global roll, pitch, yaw, arm, old_arm_disarm_val, old_control_mode_val, disarmed, mode, control_mode, acro_surge_val, roll_setpoint, pitch_setpoint, yaw_setpoint, depth_setpoint
	heave_axis = 1
	sway_axis= 2
	yaw_axis = 0
	surge_axis = 3
	roll_right_button= 2
	roll_left_button= 0
	pitch_up_button = 4
	pitch_down_button = 5
	arm_disarm_button = 9
	control_mode_button = 1
	heave_val = data.axes[heave_axis]
	yaw_val = data.axes[yaw_axis]
	sway_val = data.axes[sway_axis]
	surge_val = data.axes[surge_axis]
	pitch_val = data.buttons[pitch_up_button]-data.buttons[pitch_down_button]
	roll_val = data.buttons[roll_right_button]-data.buttons[roll_left_button]
	arm_disarm_val = data.buttons[arm_disarm_button]
	control_mode_val = data.buttons[control_mode_button]
	if control_mode_val==1 and old_control_mode_val==0:
		mode +=1
		mode = mode%len(control_modes)
		control_mode = control_modes[mode]
		#print('Control switched to',control_mode,'Mode')
	mode_pub.publish(control_mode)
	if arm_disarm_val==1 and old_arm_disarm_val==0:
		arm*=(-1)
	arm_pub.publish(arm)
	if arm==1 :
		#print('ROV ARMED')
		disarmed = False
		if not control_mode=='Position Hold':
			roll_setpoint = math.radians(roll)
			pitch_setpoint = math.radians(pitch)
			yaw_setpoint = math.radians(yaw)
		if not (control_mode=='Stabilize' or control_mode=='Stabilized Acro'):
			depth_setpoint = depth_val
		#MANUAL
		if control_mode=='Manual':
			#print('current-mode:',control_mode)
			move_rov(surge=surge_val, yaw=yaw_val, sway=sway_val, roll= roll_val, pitch=pitch_val, heave=heave_val)
		#STABILIZE
		elif control_mode=='Stabilize':
			#print('current-mode:',control_mode)
			stabilize(surge_val, yaw_val, sway_val, heave_val)
		#ACRO
		elif control_mode=='Acro':
			#print('current-mode:',control_mode)
			acro_surge_val += (surge_val*0.03)
			if acro_surge_val>1:
				acro_surge_val=1
			elif acro_surge_val<-1:
				acro_surge_val=-1
			#print('acr',acro_surge_val)
			if abs(acro_surge_val)<0.1:
				surge_val = 0
			else:
				surge_val = acro_surge_val
			move_rov(surge=surge_val, yaw=yaw_val, sway=sway_val, roll= roll_val, pitch=pitch_val, heave=heave_val)
		#POSITION HOLD
		elif control_mode=='Position Hold':
			#print('current-mode:',control_mode)
			position_hold(surge_val, yaw_val, sway_val,  roll_val, pitch_val, heave_val)
		#STABILIZED ACRO
		elif control_mode=='Stabilized Acro':
			#print('current-mode:',control_mode)
			acro_surge_val += (surge_val*0.03)
			if acro_surge_val>1:
				acro_surge_val=1
			elif acro_surge_val<-1:
				acro_surge_val=-1
			#print('acr',acro_surge_val)
			if abs(acro_surge_val)<0.1:
				surge_val = 0
			else:
				surge_val = acro_surge_val
			stabilize(surge_val, yaw_val, sway_val, heave_val)
	else:
		#print('ROV DISARMED')
		disarmed = True
		roll_setpoint = math.radians(roll)
		pitch_setpoint = math.radians(pitch)
		yaw_setpoint = math.radians(yaw)
		depth_setpoint = depth_val
		move_rov(surge=0, yaw=0, sway=0, roll= 0, pitch=0, heave=0)
	old_arm_disarm_val = arm_disarm_val
	old_control_mode_val = control_mode_val
if __name__=='__main__':
		global arm, disarmed, sim_pub, thruster_pub, old_arm_disarm_val, old_control_mode_val, control_mode, mode, acro_surge_val, roll_last_e, depth_val, depth_last_e, depth_last_time, depth_e,depth_i
		global roll_i, pitch_last_e, pitch_i, roll_last_time, pitch_last_time, last_z_vec, last_pitch, last_roll, yaw_last_e, yaw_i, yaw_last_time, roll_setpoint, pitch_setpoint, yaw_setpoint, depth_setpoint
		disarmed = False
		arm = -1
		old_arm_disarm_val = 1
		old_control_mode_val = 1
		mode = 4     ##SET DEFAULT MODE USING THIS
		control_modes = ['Manual','Stabilize','Acro','Position Hold','Stabilized Acro']
		control_mode = control_modes[mode]
		acro_surge_val = 0
		roll_last_e = 0
		roll_i = 0
		roll_last_time = time.time()
		pitch_last_e = 0
		pitch_i = 0
		pitch_last_time = time.time()
		yaw_last_e = 0
		yaw_i = 0
		yaw_last_time = time.time()
		depth_last_e = 0
		depth_i = 0
		depth_last_time = time.time()
		last_z_vec = (0,0,1)
		last_pitch = 0
		last_roll = 0
		roll_setpoint = 0
		pitch_setpoint = 0
		yaw_setpoint = 0
		depth_val = 0
		depth_setpoint = 0

		rospy.init_node('rov_control', anonymous=True)
		sim_pub = rospy.Publisher('/potrov2/thruster_manager/input_stamped',WrenchStamped,queue_size=10)
		thruster_pub = rospy.Publisher('/potrov2/thrusters/0/input_pwm',FloatStamped,queue_size=10)
		mode_pub = rospy.Publisher('potrov2/control_mode',String, queue_size=10)
		arm_pub = rospy.Publisher('potrov2/armed', Int64, queue_size=10)
		control_pub = rospy.Publisher('potrov2/control_vals',Float64MultiArray, queue_size=10)
		rospy.Subscriber('/potrov2/pressure', FluidPressure, pressure_callback)
		rospy.Subscriber('/potrov2/imu', Imu, imu_callback)
		rospy.Subscriber('/potrov2/thrusters/0/input', FloatStamped ,thruster_callback)
		rospy.Subscriber('/joy', Joy ,joy_s)	
		rospy.spin()