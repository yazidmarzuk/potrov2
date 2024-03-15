import rospy
import folium
from sensor_msgs.msg import NavSatFix, Imu, FluidPressure, Image
from PIL import Image as Img
from io import BytesIO
import cv2
import numpy as np
import time
import threading
import math
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge

# def rotate_image(image, angle):
#   #diff = angle-last_angle
#   image_center = tuple(np.array(image.shape[1::-1]) / 2)
#   rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
#   result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
#   return result

def imu_callback(data):
	global arr_img, yaw
	orientation_quat = data.orientation
	quat = (orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
	_,_,yaw = R.from_quat(quat).as_euler('xyz',degrees=True)
	# yaw = int(yaw)
	# arr_img = rotate_image(arr_img,yaw-last_yaw)
	# print(yaw-last_yaw)
	# last_yaw = yaw
# def overlay(l_img, s_img, x_offset, y_offset):
#   y1, y2 = y_offset, y_offset + s_img.shape[0]
#   x1, x2 = x_offset, x_offset + s_img.shape[1]

#   alpha_s = s_img[:, :, 3] / 255.0
#   alpha_l = 1.0 - alpha_s

#   for c in range(0, 3):
#       l_img[y1:y2, x1:x2, c] = (alpha_s * s_img[:, :, c] +
#                                 alpha_l * l_img[y1:y2, x1:x2, c])
#   return l_img

def pressure_callback(data):
  global depth_val
  depth_val = round((data.fluid_pressure-101.5)*0.1023,2)

def draw_pointer(roi, pos):
	global yaw
	head = (int(pos[0]+10*math.sin(math.radians(yaw+90))),int(pos[1]+10*math.cos(math.radians(yaw+90))))
	tl = (int(pos[0]+10*math.sin(math.radians(yaw-45))),int(pos[1]+10*math.cos(math.radians(yaw-45))))
	tr = (int(pos[0]+10*math.sin(math.radians(yaw-135))),int(pos[1]+10*math.cos(math.radians(yaw-135))))
	roi = cv2.line(roi, head,tl,(0,0,0),2)
	roi = cv2.line(roi, head,tr,(0,0,0),2)
	roi = cv2.line(roi, tl,pos,(0,0,0),2)
	roi = cv2.line(roi, tr,pos,(0,0,0),2)
	return roi

def update_img():
	global m, opencvImage
	init_latitude= -56.7189766968521
	init_longitude= -3.51561734046122
	m._png_image = None
	img = m._to_png()
	img = Img.open(BytesIO(img))
	opencvImage = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
	m.location = [22.56479530582229-init_latitude+latitude,88.33771601376584-init_longitude+longitude]
	

def gps_callback(data):
	global m, start_time, update, opencvImage, start, latitude, longitude, arr_img, latitude1, longitude1, depth_val, bridge
	latitude = data.latitude
	longitude = data.longitude
	if depth_val>0.4:
		try:
			# Haversine formula 
			lon2 = longitude1
			lon1 = longitude
			lat2 = latitude1
			lat1 = latitude
			dlon = lon2 - lon1 
			dlat = lat2 - lat1
			a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
		
			c = 2 * math.asin(math.sqrt(a)) 
			
			# Radius of earth in kilometers. Use 3956 for miles
			r = 6371
			
			# calculate the result
			error = (c * r)*10
			#error1 = (((latitude1-latitude)*111000)**(2)+((longitude1-longitude)*111000*math.cos(latitude1-latitude))**(2))**(0.5)
			print('error', error)
			latitude = latitude1
			longitude = longitude1
		except:
			latitude = data.latitude
			longitude = data.longitude
	#m.add_child(folium.Marker(location=(latitude,longitude), popup='Current Location', icon=folium.Icon(color='red')))
	#m.location = [19.07283, 72.88261]
	init_latitude= -56.7189766968521
	init_longitude= -3.51561734046122
	# m.location = [latitude,longitude]
	# m.show_in_browser()
	#m.fit_bounds([[75.790806697+latitude,76.39722734+longitude], [75.791806697+latitude,76.39822734+longitude]]) 22.56479530582229, 88.33771601376584
	if update:
		#m.fit_bounds([[22.56479530582229-init_latitude+latitude,88.33771601376584-init_longitude+longitude], [22.56479530582229-init_latitude+latitude,88.33771601376584-init_longitude+longitude]])
		if start:
			print('1')
			folium.CircleMarker(
				location=[22.56479530582229-init_latitude+latitude,88.33771601376584-init_longitude+longitude],
				radius=250,
				color="black",
				fill=True,
    			fill_opacity=0.6,
				opacity=1
			).add_to(m)
			m.add_child(folium.Marker(location=m.get_bounds()[0], popup='Current Location', icon=folium.Icon(color='red')))
			m.add_child(folium.Marker(location=m.get_bounds()[1], popup='Current Location', icon=folium.Icon(color='blue')))
			print(m.get_bounds())
			m.location = [22.56479530582229-init_latitude+latitude,88.33771601376584-init_longitude+longitude]
			start = False	
			img = m._to_png()
			img = Img.open(BytesIO(img))
			opencvImage = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
		threading.Thread(target=update_img).start()
	if time.time()-start_time>10:
		update = True
		start_time=time.time()
	else:
		update = False
	# if not m._png_image == None:
	# 	img = m._png_image
	#print(opencvImage)
	roi = opencvImage[50:550,50:550].copy()
	w = roi.shape[1]
	h = roi.shape[0]
	act_loc = [22.56479530582229-init_latitude+latitude,88.33771601376584-init_longitude+longitude]
	lat = m.location[0]
	lon = m.location[1]
	tlcor_loc = (lat+(69/111000),lon-((82/111000)*(math.cos(math.radians(lat+(69/111000))))))
	brcor_loc = (lat-(69.5/111000),lon+((81/111000)*(math.cos(math.radians(lat-(69.5/111000))))))
	total_dist_y = (brcor_loc[0]-tlcor_loc[0])*111000
	total_dist_x = (brcor_loc[1]-tlcor_loc[1])*111000*math.cos(math.radians(brcor_loc[0]))
	dist_y = (act_loc[0]-tlcor_loc[0])*111000
	dist_x = (act_loc[1]-tlcor_loc[1])*111000*math.cos(math.radians(act_loc[0]))
	#print(dist_x,dist_y)
	#cv2.circle(roi,(int(h*dist_y/total_dist_y),int(w-(w*dist_x/total_dist_x))),5,(0,0,0),-1)
	roi = draw_pointer(roi, (int(w*dist_x/total_dist_x),int(h*dist_y/total_dist_y)))
	# roi = cv2.cvtColor(roi, cv2.COLOR_BGR2BGRA)
	# roi = overlay(roi, arr_img, int(w*dist_x/total_dist_x)-10,int(h*dist_y/total_dist_y)-10)
	image_message = bridge.cv2_to_imgmsg(roi, "bgr8")
	image_pub.publish(image_message)
	cv2.imshow('test',roi)
	cv2.waitKey(1)

def dr_callback(data):
	global m, latitude1, longitude1
	latitude1 = data.latitude
	longitude1 = data.longitude
	#m.add_child(folium.Marker(location=(latitude,longitude), popup='Current Location', icon=folium.Icon(color='blue')))

global m, start_time, update, opencvImage, start, depth_val, bridge
bridge = CvBridge()
last_yaw = 0
start = True
update = True
start_time = time.time()
m = folium.Map(location=[0,0],zoom_start=100, height=600, width=600, zoom_control=False)
depth_val = 0
# cwd = rov_control.__file__[:-len('rov_control.py')]
# arr_img = cv2.imread(cwd+'res/gps_pointer2.png')
# arr_img = cv2.resize(arr_img,(20,20))
# tmp = cv2.cvtColor(arr_img, cv2.COLOR_BGR2GRAY) 
# _, alpha = cv2.threshold(tmp, 0, 255, cv2.THRESH_BINARY) 
# b, g, r = cv2.split(arr_img) 
# rgba = [b, g, r, alpha] 
# arr_img = cv2.merge(rgba, 4)
#cor_loc = (22.56479530582229, 88.33771601376584)
# print(tlcor_loc)
# m.add_child(folium.CircleMarker(location=brcor_loc, popup='Current Location'))
rospy.init_node('rov_gps', anonymous=True)
rospy.Subscriber('/potrov2/pressure', FluidPressure, pressure_callback)
rospy.Subscriber('/potrov2/gps', NavSatFix, gps_callback)
rospy.Subscriber('/potrov2/dead_reckon', NavSatFix, dr_callback)
rospy.Subscriber('/potrov2/imu', Imu, imu_callback)
image_pub = rospy.Publisher('potrov2/gps_image', Image, queue_size=1)
rospy.spin()
# m.fit_bounds([[52.193636, -2.221575], [52.636878, -1.139759]])
# m._to_png()
# img = m._png_image
# img = Image.open(BytesIO(img))
# opencvImage = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
# w = opencvImage.shape[1]
# h = opencvImage.shape[0]
# cv2.circle(opencvImage,(int(w/2),int(h/2)),10,(0,0,0),-1)
# cv2.imshow('test',opencvImage)
# print(m.location)
# cv2.waitKey(0)