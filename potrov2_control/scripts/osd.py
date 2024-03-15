#!/usr/bin/env python3
# Import the necessary libraries
import rospy # Python library for ROS
import math
import os
from std_msgs.msg import String, Int64, Float64MultiArray
import tf.transformations as tf
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import pyvista as pv
from scipy.spatial.transform import Rotation as R
import numpy as np
from threading import Thread, Event
from flask import Flask, render_template, Response
import signal, sys
#import open3d as o3d
import rov_control
global cwd
cwd = rov_control.__file__[:-len('rov_control.py')]
print(cwd)
global frame
frame = None
event = Event()

def gpsimg_callback(data):
   global gpsframe
   br = CvBridge()
   gpsframe = br.imgmsg_to_cv2(data,desired_encoding = "bgr8")

def mode_callback(msg):
   global control_mode
   control_mode = msg.data

def arm_callback(msg):
   global arm
   arm = msg.data

def control_val_callback(msg):
   global throttle
   throttle = msg.data[0]

def joy_s(data):
  global record
  global old_rec_val
  # global arm
  # global old_arm_disarm_val
  # arm_disarm_val = data.buttons[9]
  # if arm_disarm_val==1 and old_arm_disarm_val==0:
  #   arm*=(-1)
  rec_val = data.buttons[3]
  if rec_val==1 and old_rec_val == 0:
    record*=(-1)
  old_rec_val = rec_val
  #old_arm_disarm_val = arm_disarm_val

def down_callback(data):
    global downframe
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    downframe = br.imgmsg_to_cv2(data,desired_encoding = "bgr8")
    

def downcam(frame):
  global downframe
  w = int(1920/4)
  h = int(1080/4)
  downframe = cv2.resize(downframe,(w,h))
  x_off = 170
  y_off = 1400
  frame[x_off:x_off+downframe.shape[0], y_off:y_off+downframe.shape[1]] = downframe
  return frame

def gpsimg(frame):
  global gpsframe
  w = 400
  h = 400
  gpsframe = cv2.resize(gpsframe,(w,h))
  x_off = 320
  y_off = 0
  frame[x_off:x_off+gpsframe.shape[0], y_off:y_off+gpsframe.shape[1]] = gpsframe
  return frame

def imu_callback(data):
  global quat, roll, pitch, yaw
  orientation_quat = data.orientation
  quat = (orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
  #print(quat)
  # Convert quaternion to Euler angles
  roll,pitch,yaw = R.from_quat(quat).as_euler('xyz',degrees=True)
  # roll = math.degrees(roll)
  # pitch = math.degrees(pitch)
  # yaw = math.degrees(yaw)
  # print("Roll:",roll,"Pitch:",pitch,"Yaw:",yaw)
  # compass_val = math.degrees(math.atan2(data.magnetic_field.y,data.magnetic_field.x))+180
  #print(compass_val)
  global x_axis,y_axis,z_axis, last_angles, plotter
  angles = [roll,-pitch,yaw]
  meshes = [rovframe,foam,containers,thrusters,lights,containers_support]
  diff = []
  for angle,last_angle in zip(angles,last_angles):
      diff.append(angle-last_angle)
  #print(diff[0],angles[0],last_angles[0])
  r = R.from_rotvec(math.radians(diff[0])*(x_axis/np.linalg.norm(x_axis)))
  y_axis = r.apply(y_axis)
  z_axis = r.apply(z_axis)
  for mesh in meshes:
     mesh.rotate_vector(x_axis,diff[0],(0,0,0),inplace=True)
  r = R.from_rotvec(math.radians(diff[1])*(y_axis/np.linalg.norm(y_axis)))
  x_axis = r.apply(x_axis)
  z_axis = r.apply(z_axis)
  for mesh in meshes:
     mesh.rotate_vector(y_axis,diff[1],(0,0,0),inplace=True)
  r = R.from_rotvec(math.radians(diff[2])*(z_axis/np.linalg.norm(z_axis)))
  y_axis = r.apply(y_axis)
  x_axis = r.apply(x_axis)
  for mesh in meshes:
     mesh.rotate_vector(z_axis,diff[2],(0,0,0),inplace=True)
  if angles[0]==0:
     x_axis = (0,-1,0)
  if angles[1]==0:
     y_axis = (-1,0,0)
  if angles[2]==0:
     z_axis = (0,0,1)
  #tm = np.array([np.append(math.radians(diff[0])*(x_axis/np.linalg.norm(x_axis)),0),np.append(math.radians(diff[1])*(y_axis/np.linalg.norm(y_axis)),0),np.append(math.radians(diff[2])*(z_axis/np.linalg.norm(z_axis)),0),[0,0,0,1]])
  #print(tm)
  # for mesh in meshes:
  #    mesh.transform(tm, inplace=True)
  last_angles = angles

def pressure_callback(data):
  global depth_val
  depth_val = round((data.fluid_pressure-101.5)*0.1023,2)

def compass_scale(frame):
  global yaw
  compass_val = (yaw + 180 + 90)%360
  w = 1920
  roi_left = frame[0:120,0:int(w*0.259)].copy()
  roi_right = frame[0:120,int(w*(1-0.259)):w].copy()
  compass_text = ''
  for i in range(4):
    compass_text += ('000'+str(((int(compass_val)-((4-i)*45))%360)))[-3:]
    compass_text += '------'
  compass_text += ('000'+str(int(compass_val)))[-3:]
  compass_text += '------'
  for i in range(4):
    compass_text += ('000'+str(((1+i)*45+int(compass_val))%360))[-3:]
    if not i==3:
        compass_text += '------'
  cv2.putText(frame, compass_text, (65,50), cv2.FONT_ITALIC, 1,(255,255,255),1)
  cv2.putText(frame, 'S     SW     W     NW     N     NE     E     SE     S     SW     W     NW     N     NE     E     SE     S',(int(compass_val*5)-1770,100), cv2.FONT_ITALIC, 2,(255,255,255),1)
  img = frame[:,:].copy()
  img[0:120,0:int(w*0.259)] = roi_left
  img[0:120,int(w*(1-0.259)):w] = roi_right
  return img

def record_vid(frame):
  global record
  global inc
  global file_count
  global out
  w = 1920
  h = 1080
  if record==1:
    if inc:
      inc=False
      _, _, files = next(os.walk(cwd+"recordings/"))
      file_count = len(files)+1
      out = cv2.VideoWriter(cwd+'recordings/recording_'+str(file_count)+'.avi', cv2.VideoWriter_fourcc(*'XVID'), 21.0, (1920, 1080))
    out.write(frame)
    cv2.rectangle(frame, (int(w*0.94),int(h*0.48)),(int(w*0.95)+20,int(h*0.5)+20),(0,0,255),-1)
    cv2.circle(frame, (int(w*0.95),int(h*0.5)),50,(0,0,255),2)
    recording_image = cv2.imread(cwd+"res/recording.png", -1)
    recording_image = cv2.resize(recording_image,(80,80))
    frame = overlay(frame[:,:].copy(), recording_image,90,700)
    #cv2.putText(frame, 'Press (A)', (int(w*0.9257),int(h*0.565)), cv2.FONT_ITALIC, 0.75, (0,0,0),2)
    #cv2.putText(frame, 'to Stop', (int(w*0.9257),int(h*0.585)), cv2.FONT_ITALIC, 0.75, (0,0,0),2)
  else:
    inc = True
    cv2.circle(frame, (int(w*0.95),int(h*0.5)),50,(0,0,255),-1)
    cv2.circle(frame, (int(w*0.95),int(h*0.5)),50,(0,0,0),2)
    #cv2.putText(frame, 'REC', (int(w*0.935),int(h*0.51)), cv2.FONT_ITALIC, 1, (255,255,255),2)
    #record_image = cv2.imread(cwd+"res/record.png", -1)
    #record_image = cv2.resize(record_image,(80,80))
    #frame = overlay(frame[:,:].copy(), record_image,int(w*0.931),int(h*0.468))
    #cv2.putText(frame, 'Press (A)', (int(w*0.926),int(h*0.565)), cv2.FONT_ITALIC, 0.75, (0,0,0),2)
    #cv2.putText(frame, 'to Record', (int(w*0.9256),int(h*0.585)), cv2.FONT_ITALIC, 0.75,(0,0,0),2)
  return frame

def overlay(l_img, s_img, x_offset, y_offset):
  y1, y2 = y_offset, y_offset + s_img.shape[0]
  x1, x2 = x_offset, x_offset + s_img.shape[1]

  alpha_s = s_img[:, :, 3] / 255.0
  alpha_l = 1.0 - alpha_s

  for c in range(0, 3):
      l_img[y1:y2, x1:x2, c] = (alpha_s * s_img[:, :, c] +
                                alpha_l * l_img[y1:y2, x1:x2, c])
  return l_img

# def set_kv(quat):
#     global y_axis,z_axis
    #print(y_axis, z_axis, 'old')
    # r = R.from_quat(quat)
    # tm = r.as_matrix()
    # tm = tm.tolist()
    # tm[0].append(0)
    # tm[1].append(0)
    # tm[2].append(0)
    # tm.append([0.0,0.0,0.0,1.0])
    # tm = np.array(tm)
    # k = (tm[1][0],tm[1][1],tm[1][2])
    # v = (tm[2][0],tm[2][1],tm[2][2])
    #print(k,v)
    # x = quat[0]
    # y = quat[1]
    # z = quat[2]
    # w = quat[3]

    # t0 = +2.0 * (w * x + y * z)
    # t1 = +1.0 - 2.0 * (x * x + y * y)
    # roll = math.atan2(t0, t1)
        
    # t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # pitch = -math.asin(t2)
        
    # t3 = +2.0 * (w * z + x * y)
    # t4 = +1.0 - 2.0 * (y * y + z * z)
    # yaw  = -math.atan2(t3, t4)- np.pi/2
    # roll = math.degrees(roll)
    # pitch = math.degrees(pitch)
    # yaw = math.degrees(yaw)
    # print(roll,pitch,yaw)
    # cr = np.cos(roll)
    # sr = np.sin(roll)
    # cp = np.cos(pitch)
    # sp = np.sin(pitch)
    # cy = np.cos(yaw)
    # sy = np.sin(yaw)


    # k = (cy*cp, sp, sy*cp)
    # y = (0,1,0)
    # s = np.cross(k,y)
    # v = np.cross (s,k)
    # v = (v*cr) + (np.cross(k,v)*sr) 
    # print(k,v)
    # unit_vector_1 = k / np.linalg.norm(k)
    # unit_vector_2 = y_axis / np.linalg.norm(y_axis)
    # dot_product = np.dot(unit_vector_1, unit_vector_2)
    # angle_y = math.degrees(np.arccos(round(dot_product,4)))
    # #print('dot',dot_product)
    # meshes = [rovframe,foam,containers,thrusters,lights,containers_support]
    # if not (np.isnan(angle_y) or angle_y==0):
    #     vector_y = np.cross(k,y_axis)
    #     vector_y = vector_y/np.linalg.norm(vector_y)
    #     #print(angle_y, vector_y,'y')
    #     for mesh in meshes:
    #         tm = R.from_rotvec(math.radians(angle_y)*vector_y).as_matrix()
    #         # tm[0].append(0)
    #         # tm[1].append(0)
    #         # tm[2].append(0)
    #         # tm.append([0.0,0.0,0.0,1.0])
    #         # tm = np.array(tm)
    #         mesh.rotate(tm,center=(0,0,0))
    #         vis.update_geometry(mesh)
    #     y_axis = R.from_rotvec(math.radians(-angle_y)*vector_y).apply(y_axis)
    #     z_axis = R.from_rotvec(math.radians(-angle_y)*vector_y).apply(z_axis)
    # unit_vector_1 = v / np.linalg.norm(v)
    # unit_vector_2 = z_axis / np.linalg.norm(z_axis)
    # dot_product = np.dot(unit_vector_1, unit_vector_2)
    # angle_z = math.degrees(np.arccos(round(dot_product,4)))
    # # vector_z = np.cross(v,z_axis)
    # vector_z = y_axis/np.linalg.norm(y_axis)
    # #print(angle_z, vector_z,'z')
    # if not (np.isnan(angle_z) or angle_z==0):
    #     for mesh in meshes:
    #         tm = R.from_rotvec(math.radians(angle_z)*vector_z).as_matrix()
    #         # tm[0].append(0)
    #         # tm[1].append(0)
    #         # tm[2].append(0)
    #         # tm.append([0.0,0.0,0.0,1.0])
    #         # tm = np.array(tm)
    #         mesh.rotate(tm,center=(0,0,0))
    #         vis.update_geometry(mesh)
    #     z_axis = R.from_rotvec(math.radians(-angle_z)*vector_z).apply(z_axis)
    #print(y_axis,z_axis,'new')

# def rotate_all(meshes, angles,last_angles):
#     diff = []
#     for angle,last_angle in zip(angles,last_angles):
#         if angle == 0:
#             diff.append(-last_angle)
#         else:
#             diff.append(angle-last_angle)
#     for mesh in meshes:
#         mesh.rotate_vector(angles[0], point=(0, 0, 0), transform_all_input_vectors = True, inplace=True)
#         mesh.rotate_y(an[1], point=(0, 0, 0), transform_all_input_vectors = True, inplace=True)
#         mesh.rotate_z(diff[2], point=(0, 0, 0), transform_all_input_vectors = True, inplace=True)
#     return angles

# def set_kv():
#   global x_axis,y_axis,z_axis
#   plotter.clear_actors()
#   # meshes = [rovframe,foam,containers,thrusters,lights,containers_support]
#   plotter.add_arrows(cent=np.array([0,0,0]),direction=np.array(x_axis), mag=1)
#   plotter.add_arrows(cent=np.array([0,0,0]),direction=np.array(y_axis), mag=1)
#   plotter.add_arrows(cent=np.array([0,0,0]),direction=np.array(z_axis), mag=1)

def clone_3d(frame):
  global rovframe,foam,containers,thrusters,lights,containers_support,vis,quat
  # r = R.from_quat(quat)
  # rvr = r.apply(vr)
  # #rvv = R.from_euler('xyz',(45,0,0),degrees=True).apply(rvr)
  # plotter.view_vector(rvr)
  #set_kv()

  # vis.poll_events()

  # vis.update_renderer()
  
  # clone = np.asarray(vis.capture_screen_float_buffer())
  plotter.update()
  clone = plotter.screenshot(transparent_background = True)
  src = cv2.cvtColor(clone, cv2.COLOR_RGB2BGR)
  #print(src)
  src = cv2.resize(src,(int(src.shape[1]/3),int(src.shape[0]/3)))
  tmp = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY) 
  _, alpha = cv2.threshold(tmp, 0, 255, cv2.THRESH_BINARY) 
  b, g, r = cv2.split(src) 
  rgba = [b, g, r, alpha] 
  dst = cv2.merge(rgba, 4)
  frame = overlay(frame,dst,50,50)
  #model_image = cv2.imread(cwd+"res/model.png", -1)
  #model_image = cv2.resize(model_image,(int(model_image.shape[0]/1.5),int(model_image.shape[1]/1.5)))
  #frame = overlay(frame, model_image,50, 50)
  return frame

def createIndicators(frame):
  global depth_val, control_mode, arm
  w = 1920
  h = 1080
  battery_image = cv2.imread(cwd+"res/battery.png", -1)
  battery_image = cv2.resize(battery_image,(120,120))
  frame = overlay(frame, battery_image, int(w*0.86),30)
  cv2.putText(frame,'90%',(int(w*0.92),100),cv2.FONT_ITALIC,1.5,(255,255,255),2)

  depth_image = cv2.imread(cwd+"res/depth.png", -1)
  depth_image = cv2.resize(depth_image,(120,120))
  frame = overlay(frame, depth_image,50, int(h*0.87))
  cv2.putText(frame,str(depth_val)+'m',(170,int(h*0.937)),cv2.FONT_ITALIC,1.5,(0,0,0),2)

  temp_image = cv2.imread(cwd+"res/temp.png", -1)
  temp_image = cv2.resize(temp_image,(100,100))
  frame = overlay(frame, temp_image,65, int(h*0.79))
  cv2.putText(frame,'30.0deg.C',(170,int(h*0.845)),cv2.FONT_ITALIC,1.5,(0,0,0),2)

  light_image = cv2.imread(cwd+"res/light.png", -1)
  light_image = cv2.resize(light_image,(100,100))
  frame = overlay(frame, light_image,65, int(h*0.7))
  onswitch_image = cv2.imread(cwd+"res/onswitch.png", -1)
  onswitch_image = cv2.resize(onswitch_image,(50,50))
  frame = overlay(frame, onswitch_image,175, int(h*0.722))

  controller_image = cv2.imread(cwd+"res/controller.png", -1)
  controller_image = cv2.resize(controller_image,(100,100))
  frame = overlay(frame, controller_image,265, int(h*0.7))
  if arm == 1:
    onswitch_image = cv2.imread(cwd+"res/onswitch.png", -1)
    onswitch_image = cv2.resize(onswitch_image,(50,50))
    frame = overlay(frame, onswitch_image,380, int(h*0.722))
  else:
    offswitch_image = cv2.imread(cwd+"res/offswitch.png", -1)
    offswitch_image = cv2.resize(offswitch_image,(50,50))
    frame = overlay(frame, offswitch_image,380, int(h*0.722))
  
  mode_image = cv2.imread(cwd+"res/mode.png", -1)
  mode_image = cv2.resize(mode_image,(60,60))
  frame = overlay(frame, mode_image,int(w*0.83), int(h*0.9))
  cv2.putText(frame,control_mode,(int(w*0.87), int(h*0.935)),cv2.FONT_ITALIC,1,(0,0,0),2)
  #return
  return frame

def controlIndicator(frame):
  global throttle
  w = 1920
  h = 1080
  x0 = w*0.1*throttle+w*0.5
  if x0>=0.5:
    cv2.rectangle(frame,(int(w*0.5),int(h*0.95)),(int(x0),int(h*0.96)),(0,200,0),-1)
  else:
    cv2.rectangle(frame,(int(x0),int(h*0.95)),(int(w*0.5),int(h*0.96)),(0,200,0),-1)
  cv2.rectangle(frame,(int(w*0.4),int(h*0.95)),(int(w*0.6),int(h*0.96)),(0,0,0),1)
  cv2.putText(frame,'Throttle: '+str(int(throttle*100))+'%',(int(w*0.435),int(h*0.94)),cv2.FONT_ITALIC,1,(0,0,0),2)
  return frame

def aim(frame):
  global roll, pitch
  w = 1920
  h = 1080
  aim_image = cv2.imread(cwd+"res/aim.png", -1)
  aim_image = cv2.resize(aim_image,(int(645*1.5),int(387*1.5)))
  frame = overlay(frame, aim_image,int(w*0.2535), int(h*0.23))
  #Roll Indicator
  roll = int(roll)
  roll = math.radians(roll)
  o = (w/2,h/2)
  x0 = w*0.5-h*0.25-o[0]
  y0 = (h/2)-o[1]
  x1 = w*0.5+h*0.25-o[0]
  y1 = (h/2)-o[1]
  rx0=(math.cos(roll)*x0)-(math.sin(roll)*y0)+o[0]
  ry0=(math.sin(roll)*x0)+(math.cos(roll)*y0)+o[1]
  rx1=(math.cos(roll)*x1)-(math.sin(roll)*y1)+o[0]
  ry1=(math.sin(roll)*x1)+(math.cos(roll)*y1)+o[1]
  #print(x0,y0,rx0,ry0)
  cv2.line(frame, (int(rx0),int(ry0)),(int(rx1),int(ry1)),(0,100,255),2)
  cv2.line(frame, (int(w*0.5-h*0.25),int(h/2)),(int(w*0.5+h*0.25),int(h/2)),(100,255,0),2)
  #Pitch Indicator
  pitch = int(-pitch)*2
  #print(pitch)
  x0 = h*0.25
  x1 = h*0.75
  y = (((x1-x0)*pitch)/360)+((x0+x1)/2)
  cv2.line(frame, (int(w/2),int(x0)),(int(w/2),int(x1)),(100,255,0),2)
  cv2.line(frame, (int(w*0.5-20),int(y)),(int(w*0.5+20),int(y)),(0,100,255),2)
  return frame

def callback(data):
  global frame
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
  w = 1920
  h = 1080
  # Output debugging information to the terminal
  #rospy.loginfo("receiving video frame")
  # Convert ROS Image message to OpenCV image
  frame = br.imgmsg_to_cv2(data,desired_encoding = "bgr8")
  #video_capture = frame[:,:].copy()
  cv2.resize(frame,(w,h))

  #RECORD
  frame = record_vid(frame)

  #AIM
  frame = aim(frame)

  #COMPASS
  frame = compass_scale(frame)

  #Indicators
  frame = createIndicators(frame)

  #Control Indicator
  frame = controlIndicator(frame)

  #3D CLONE IMU
  frame = clone_3d(frame)

  #ADD DOWN FACING CAMERA FRAME
  frame = downcam(frame)

  #ADD GPS FRAME
  frame = gpsimg(frame)

  frame = cv2.imencode(".jpg",frame)[1].tobytes()
  event.set()
  #socketio.emit('video_frame', {'image': cv2.imencode('.jpg', frame)[1].tobytes()})
  # Display image
  # cv2.namedWindow("OSD", cv2.WINDOW_NORMAL)
  # cv2.setWindowProperty("OSD", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
  # cv2.imshow("OSD", frame)
  # cv2.moveWindow('OSD',0,0) 
  # cv2.waitKey(1)

# def generate_frames():
#     while True:
#         if video_capture is not None:
#             ret, buffer = cv2.imencode('.jpg', video_capture)
#             frame = buffer.tobytes()
#             yield (b'--frame\r\n'
#                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
# app = Flask(__name__,template_folder='templates')
# socketio = SocketIO(app)
# @app.route('/')
# def index():
#     return render_template('index.html')

# def background_thread():
#     while True:
#         socketio.sleep(0.1)

app = Flask(__name__)

def get_frame():
    global frame
    event.wait()
    event.clear()
    return frame

@app.route('/')
def index():
    return render_template('index.html')

def gen():
    while True:
        frame = get_frame()
        try:
          yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        except:
           pass

@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

#def main():
# Tells rospy the name of the node.
# Anonymous = True makes sure the node has a unique name. Random
# numbers are added to the end of the name. 
Thread(target=lambda: rospy.init_node('osd', anonymous=True, disable_signals=True)).start()

global test
test = 0
global pitch
global roll
global yaw
global depth_val
pitch = 0
roll = 0
yaw = 0
depth_val = 0
global record
global old_rec_val
record = -1
old_rec_val = 1
global arm
#global old_arm_disarm_val
arm = -1
#old_arm_disarm_val = 1
global inc
global file_count
inc = False
file_count = 0
global out
out = cv2.VideoWriter(cwd+'recordings/recording_'+str(0)+'.avi', cv2.VideoWriter_fourcc(*'XVID'), 21.0, (1920, 1080))
global control_mode
control_mode = 'Manual'
global throttle
throttle = 0
global last_angles
last_angles = [0,0,0]
global downframe
downframe = np.zeros((270,480,3),np.uint8)

global rovframe,foam,containers,thrusters,lights,containers_support,vis,quat,x_axis,y_axis,z_axis
rovframe = pv.read(cwd+'rov/frame_red.stl')
foam = pv.read(cwd+'rov/foam_red.stl')
containers = pv.read(cwd+'rov/containers_red.stl')
thrusters = pv.read(cwd+'rov/thrusters_red.stl')
lights = pv.read(cwd+'rov/lights_red.stl')
containers_support = pv.read(cwd+'rov/containers_support_red.stl')
plotter = pv.Plotter(off_screen = True)
plotter.add_mesh(rovframe, color=[0.3,0.3,0.3])
plotter.add_mesh(foam, color=[230/255,200/255,10/255])
plotter.add_mesh(containers, color=[0.6,0.6,0.6])
plotter.add_mesh(thrusters, color=[250/255,250/255,250/255])
plotter.add_mesh(lights, color=[0.8,0.8,0.8])
plotter.add_mesh(containers_support, color=[10/255,200/255,200/255])
plotter.set_background('black')
x_axis = (0,-1,0)
y_axis = (-1,0,0)
z_axis = (0,0,1)
vr = (0,1,0)
vv = (0,0,1)
plotter.view_vector(vr,vv)
quat = (0,0,0,1)
plotter.zoom_camera(1.19)
plotter.show(auto_close=False)
# rovframe = o3d.io.read_triangle_mesh("/home/zaid-pe/blenderpic/rov/frame.stl")
# rovframe.compute_vertex_normals()
# rovframe.paint_uniform_color([0.3,0.3,0.3])
# foam = o3d.io.read_triangle_mesh(cwd+'rov/foam.stl')
# foam.compute_vertex_normals()
# foam.paint_uniform_color([230/255,200/255,10/255])
# containers = o3d.io.read_triangle_mesh(cwd+'rov/containers.stl')
# containers.compute_vertex_normals()
# containers.paint_uniform_color([0.6,0.6,0.6])
# thrusters = o3d.io.read_triangle_mesh(cwd+'rov/thrusters.stl')
# thrusters.compute_vertex_normals()
# thrusters.paint_uniform_color([250/255,250/255,250/255])
# lights = o3d.io.read_triangle_mesh(cwd+'rov/lights.stl')
# lights.compute_vertex_normals()
# lights.paint_uniform_color([0.8,0.8,0.8])
# containers_support = o3d.io.read_triangle_mesh(cwd+'rov/containers_support.stl')
# containers_support.compute_vertex_normals()
# containers_support.paint_uniform_color([10/255,200/255,200/255])
# global y_axis,z_axis
# # Calculate the axis vectors
# x_axis = (1,0,0)
# y_axis = (0,1,0)
# z_axis = (0,0,1)

# vis = o3d.visualization.Visualizer()

# vis.create_window(width=1920, height=1080, visible = False)
# render_option = vis.get_render_option()
# render_option.background_color = np.asarray([0, 0, 0])  # Set background color to black
# vis.add_geometry(rovframe)
# vis.add_geometry(foam)
# vis.add_geometry(containers)
# vis.add_geometry(thrusters)
# vis.add_geometry(lights)
# vis.add_geometry(containers_support)
# quat = (0.3826834, 0, 0, 0.9238795)
# meshes = [rovframe,foam,containers,thrusters,lights,containers_support]
# for mesh in meshes:
#     mesh.rotate(R.from_euler('XYZ',angles=(-90,0,180),degrees=True).as_matrix(),center=(0,0,0))
#     vis.update_geometry(mesh)
# Node is subscribing to topics
rospy.Subscriber('/potrov2/gps_image', Image, gpsimg_callback)
rospy.Subscriber('/potrov2/control_mode', String ,mode_callback)
rospy.Subscriber('/potrov2/armed', Int64 ,arm_callback)
rospy.Subscriber('/potrov2/control_vals',Float64MultiArray, control_val_callback)
rospy.Subscriber('/potrov2/imu', Imu, imu_callback)
rospy.Subscriber('/potrov2/pressure', FluidPressure, pressure_callback)
rospy.Subscriber('/joy', Joy ,joy_s)
rospy.Subscriber('/potrov2/camera_down/camera_image', Image, down_callback)
rospy.Subscriber('/potrov2/camera_front/camera_image', Image, callback)

# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()

# Close down the video stream when done


if __name__ == '__main__':
  # socketio.start_background_task(target=background_thread)
  # socketio.run(app, host='127.0.0.1', port=5000, debug=True)
  app.run(host='0.0.0.0', port=8080 ,debug=False)
  #cv2.destroyAllWindows()