from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3
from sensor_msgs.msg import Imu
import rospy
import random

def create_disturb(x,y):
	print(x,y)
	wrench = Wrench()
	wrench.force.x = x
	wrench.force.y = y
	wrench.force.z = 0
	wrench.torque.x = 0
	wrench.torque.y = 0
	wrench.torque.z = 0
	apply_wrench(
		'potrov2/base_link',
		'world',
		Point(0, 0, 0),
		wrench,
		rospy.Time().now(),
		rospy.Duration(-1))
	
def imu_callback(_):
	global x, y
	dx = random.choice([1,-1])*random.uniform(0,0.5)
	dy = random.choice([1,-1])*random.uniform(0,0.5)
	x += dx
	y += dy
	lim = 10
	if x>lim:
		x=lim
	elif x<-lim:
		x=-lim
	if y>lim:
		y=lim
	if y<-lim:
		y=-lim
	create_disturb(x,y)

if __name__=='__main__':
	global x, y
	x = 8
	y = 8
	rospy.init_node('disturbances', anonymous=True)
	rospy.Subscriber('/potrov2/imu', Imu, imu_callback)
	rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
	apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
	rospy.spin()
	create_disturb(0,0)