#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int64, Float64MultiArray
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu, FluidPressure, Joy, NavSatFix
from std_msgs.msg import Float64
from uuv_sensor_ros_plugins_msgs.msg import DVL
from scipy.spatial.transform import Rotation as R
import math
import time
import numpy as np

def thrust_forces(control_forces, control_torques):
	#                                          Surge    Sway   Heave   Roll    Pitch   Yaw    Thruster
	inverse_configuration_matrix = np.array([[-0.375,  0.375,  0,      0,      0,      0.25],    #1
										  	 [-0.375, -0.375,  0,      0,      0,     -0.25],    #2
											 [ 0.375,  0.375,  0,      0,      0,     -0.25],    #3
											 [ 0.375, -0.375,  0,      0,      0,      0.25],    #4
											 [ 0,      0,      0.33,  -0.33,  -0.33,   0.0],     #5
											 [ 0,      0,      0.33,   0.33,  -0.33,   0.0],     #6
											 [ 0,      0,      0.33,  -0.33,   0.33,   0.0],     #7
											 [ 0,      0,      0.33,   0.33,   0.33,   0.0],])   #8
	#print(inverse_configuration_matrix)
	gen_forces = np.hstack(
		(control_forces, control_torques)).transpose()
	thrust = inverse_configuration_matrix.dot(gen_forces)
	print(thrust)
	thruster0_pub.publish(thrust[0])
	thruster1_pub.publish(thrust[1])
	thruster2_pub.publish(thrust[2])
	thruster3_pub.publish(thrust[3])
	thruster4_pub.publish(thrust[4])
	thruster5_pub.publish(thrust[5])
	thruster6_pub.publish(thrust[6])
	thruster7_pub.publish(thrust[7])

def gps_callback(data):
	global latitude, longitude
	latitude = data.latitude
	longitude = data.longitude

def dvl_callback(data):
	global vx, vy, vz, yaw, latitude, longitude, depth_val, last_time, dist_x, dist_y, init_latitude, init_longitude
	gps_msg = NavSatFix()
	vx = round(data.velocity.z,3)
	vy = round(-data.velocity.y,3)
	vz = round(-data.velocity.x,3)
	#print('*',vx, vy)
	# init_latitude = -56.71897669633431       #LAT & LON at x,y = 0
	# init_longitude = -3.515625000000001
	if depth_val<0.5:
		#print('FOUND GPS CONNECTION')
		dist_x = 0
		dist_y = 0
		init_latitude = latitude
		init_longitude = longitude
		# dist_y = (latitude-init_latitude)*111320
		# dist_x = (longitude-init_longitude)*111320*math.cos(math.radians(latitude))
		# print(dist_x,dist_y)
		#print('GPS POSITION:\nLatitude:',latitude,'\nLongitude:',longitude)
	else:
		#print('GPS CONNECTION LOST! DEAD RECKONING...')
		current_time = time.time()
		dx = vx*(current_time-last_time)
		dy = vy*(current_time-last_time)
		last_time = current_time
		dist_x += dx*math.cos(math.radians(yaw))+dy*math.sin(math.radians(yaw))
		dist_y += dx*math.sin(math.radians(yaw))-dy*math.cos(math.radians(yaw))
		#print(dist_x,dist_y)
		latitude1 = (dist_y/111000)+init_latitude
		longitude1 = (dist_x/(111000*math.cos(math.radians(latitude1))))+init_longitude
		#latitude1, longitude1 = calculate_new_coordinates(init_latitude, init_longitude, ((dist_x)**2+(dist_y)**2)**(0.5),yaw)
		gps_msg.header.stamp = rospy.Time.now()
		gps_msg.latitude = latitude1
		gps_msg.longitude = longitude1
		gps_pub.publish(gps_msg)
		#print('GPS POSITION:\nLatitude:',latitude1,'\nLongitude:',longitude1)
		# m.add_child(folium.Marker(location=(latitude,longitude), popup='Current Location', icon=folium.Icon(color='red')))
		# m.add_child(folium.Marker(location=(latitude1,longitude1), popup='Current Location', icon=folium.Icon(color='blue')))
		# m.save('live_map.html')
		#print('ERROR:\nLatitude:',latitude1-latitude,'\nLongitude:',longitude1-longitude)


def pressure_callback(data):
  global depth_val
  depth_val = round((data.fluid_pressure-101.5)*0.1023,2)
  #print('depth=', depth_val)

def imu_callback(data):
	global quat, roll, pitch, yaw, last_z_vec, last_roll, last_pitch
	orientation_quat = data.orientation
	quat = (orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
	roll,pitch,yaw = R.from_quat(quat).as_euler('xyz',degrees=True)

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
	#for simulation
	msg.wrench.force.x = surge*(5000)
	msg.wrench.force.y = sway*(5000)
	msg.wrench.force.z = heave*(-3000)
	msg.wrench.torque.x = roll*(-20000)
	msg.wrench.torque.y = pitch*(50000)
	msg.wrench.torque.z = yaw*(-6000)
	sim_pub.publish(msg)
	#for actual robot
	force = np.array((surge, sway, heave))
	torque = np.array((roll, pitch, yaw))
	thrust_forces(force,torque)
	#for osd
	control_pub.publish(Float64MultiArray(data=[surge,yaw,sway,roll,pitch,heave]))

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
	global roll, pitch, yaw, roll_last_e, roll_i, pitch_last_e, pitch_i, roll_last_time, pitch_last_time, yaw_last_e, yaw_i, yaw_last_time, roll_setpoint, pitch_setpoint, yaw_setpoint
	if not yaw_val==0:
		yaw_cf = yaw_val
		yaw_setpoint = math.radians(yaw)
	else:
		yaw = math.radians(yaw)
		kp=1
		kd=0.05
		ki=0.1
		yaw_e = yaw_setpoint-yaw
		yaw_d = (yaw_e-yaw_last_e)/(time.time()-yaw_last_time)
		yaw_last_time = time.time()
		yaw_i = yaw_i+yaw_e
		yaw_cf = kp*yaw_e + kd*yaw_d + ki*yaw_i
		yaw_last_e = yaw_e

	global roll, pitch, roll_last_e, roll_i, pitch_last_e, pitch_i, roll_last_time, pitch_last_time, depth_val, depth_last_e, depth_last_time, depth_e,depth_i, depth_setpoint, vx, vx_last_e, vx_last_time, vx_e, vx_i, vy, vy_last_e, vy_last_time, vy_e, vy_i
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
	if not heave_val==0:
		depth_setpoint = depth_val
		heave_cf=heave_val
	else:
		#print(depth_setpoint)
		depth_e =  depth_val - depth_setpoint
		kp = 2
		kd = 0.02
		ki = 0.01
		depth_d = (depth_e-depth_last_e)/(time.time()-depth_last_time)
		depth_last_time = time.time()
		depth_i = depth_i+depth_e
		heave_cf = kp*depth_e + kd*depth_d + ki*depth_i
		depth_last_e = depth_e
	#STABILIZE AGAINST WATER CURRENT
	if not surge_val==0:
		vx_cf = surge_val
	else:
		kp=6
		kd=0.01
		ki=0.15
		vx_e = 0-vx
		vx_d = (vx_e-vx_last_e)/(time.time()-vx_last_time)
		vx_last_time = time.time()
		vx_i = vx_i+vx_e
		vx_cf = kp*vx_e + kd*vx_d + ki*vx_i
		vx_last_e = vx_e
	#print('error',vx_e)
	if not sway_val==0:
		vy_cf = sway_val
	else:
		kp=6
		kd=0.01
		ki=0.15
		vy_e = vy-0
		vy_d = (vy_e-vy_last_e)/(time.time()-vy_last_time)
		vy_last_time = time.time()
		vy_i = vy_i+vy_e
		vy_cf = kp*vy_e + kd*vy_d + ki*vy_i
		vy_last_e = vy_e
	
	#APPLY CORRECTIVE FORCES
	move_rov(pitch= pitch_cf, roll= roll_cf, surge=vx_cf, yaw=yaw_cf, sway=vy_cf, heave=heave_cf)

def joy_s(data):
	global roll, pitch, yaw, arm, old_arm_disarm_val, old_control_mode_val, disarmed, mode, control_mode, acro_surge_val, roll_setpoint, pitch_setpoint, yaw_setpoint, depth_setpoint, vx
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
		print('Control switched to',control_mode,'Mode')
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
		global roll, pitch, yaw, roll_i, pitch_last_e, pitch_i, roll_last_time, pitch_last_time, last_z_vec, last_pitch, last_roll, yaw_last_e, yaw_i, yaw_last_time, roll_setpoint, pitch_setpoint, yaw_setpoint, depth_setpoint
		global vx, vx_last_e, vx_last_time, vx_e, vx_i, vy, vy_last_e, vy_last_time, vy_e, vy_i, last_time, dist_x, dist_y, m, init_longitude, init_latitude, latitude, longitude
		disarmed = False
		roll = 0
		pitch = 0
		yaw = 0
		arm = -1     #DEFAULT DISARMED
		old_arm_disarm_val = 1
		old_control_mode_val = 1
		mode = 0     ##SET DEFAULT MODE USING THIS
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
		vx = 0
		vx_last_e = 0
		vx_i = 0
		vx_last_time = time.time()
		vy = 0
		vy_last_e = 0
		vy_i = 0
		vy_last_time = time.time()
		last_time = time.time()
		dist_x = 0
		dist_y = 0
		init_latitude = 0
		init_longitude = 0
		latitude = 0
		longitude = 0
		
		rospy.init_node('rov_control', anonymous=True)
		sim_pub = rospy.Publisher('/potrov2/thruster_manager/input_stamped',WrenchStamped,queue_size=10)
		thruster0_pub = rospy.Publisher('/potrov2/thrusters/0/input_pwm',Float64,queue_size=10)
		thruster1_pub = rospy.Publisher('/potrov2/thrusters/1/input_pwm',Float64,queue_size=10)
		thruster2_pub = rospy.Publisher('/potrov2/thrusters/2/input_pwm',Float64,queue_size=10)
		thruster3_pub = rospy.Publisher('/potrov2/thrusters/3/input_pwm',Float64,queue_size=10)
		thruster4_pub = rospy.Publisher('/potrov2/thrusters/4/input_pwm',Float64,queue_size=10)
		thruster5_pub = rospy.Publisher('/potrov2/thrusters/5/input_pwm',Float64,queue_size=10)
		thruster6_pub = rospy.Publisher('/potrov2/thrusters/6/input_pwm',Float64,queue_size=10)
		thruster7_pub = rospy.Publisher('/potrov2/thrusters/7/input_pwm',Float64,queue_size=10)
		mode_pub = rospy.Publisher('potrov2/control_mode',String, queue_size=10)
		arm_pub = rospy.Publisher('potrov2/armed', Int64, queue_size=10)
		control_pub = rospy.Publisher('potrov2/control_vals',Float64MultiArray, queue_size=10)
		gps_pub = rospy.Publisher('potrov2/dead_reckon',NavSatFix, queue_size=1)
		rospy.Subscriber('/potrov2/gps', NavSatFix, gps_callback)
		rospy.Subscriber('/potrov2/pressure', FluidPressure, pressure_callback)
		rospy.Subscriber('/potrov2/imu', Imu, imu_callback)
		rospy.Subscriber('/potrov2/dvl', DVL, dvl_callback)
		rospy.Subscriber('/joy', Joy ,joy_s)
		rospy.spin()