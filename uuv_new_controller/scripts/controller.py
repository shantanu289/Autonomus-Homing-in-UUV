#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist, Vector3, Wrench, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from uuv_control_interfaces import DPControllerBase
import pandas as pd

x = 10.0                # initial x
y = 10.0                # initial y
z = -10.0                # initial z
roll = 0.0             # initial roll   (about x axis)
pitch = 0.87            # initial pitch  (aobut y axis)
yaw = 0.52                # initial yaw    (about z axis)
theta = 0.78539816339745
phi = np.pi/2 - 0.81482691637099
r = 5.8309518948453	

'''
class TutorialDPController(DPControllerBase):
        def __init__(self):
                super(TutorialDPController, self).__init__(self)
'''

def newOdom(msg):
	global x
	global y
	global z
	global yaw
	global roll
	global pitch
	global r 
	global theta 
	global phi
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z
	rot_q = msg.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	(r, phi, theta) = cartesian_to_spherical(x, y, z)
	#print("Roll: ", roll, "Pitch: ", pitch, "Yaw: ", yaw)
        # roll, pitch, yaw is for the heading direction
	# r, phi, theta is for the current robot position 

def cartesian_to_spherical(a, b, c):
	r_hat = (a**2 + b**2 + c**2)**0.5          # always positive
	phi_hat = np.arcsin(c/r_hat)                   # between -pi/2 and pi/2      perpendicular to x-y plane
	theta_hat = atan2(b, a)                    # between -pi and pi          in x-y plane
	return r_hat, phi_hat, theta_hat

        
if __name__ == '__main__':
	k=0
	l=[0] #l is time

	x1=[10]#x-coordinate of robot
	y1=[10]#y-coordinate of robot
	z1=[-10]#z-coordinate of robot

	yaw_arr = [0.52]               # yaw angle list
	pitch_arr = [0.87]             # pitch angle list
	roll_arr = [0]              # roll angle list
	theta_arr = [0.78539816339745]             # theta angle (-pi , pi)
	phi_arr = [np.pi/2 - 0.81482691637099]               # phi angle (-pi/2 , pi/2)
	r_arr = [5.8309518948453]                 # distance from the origin

	angle_1_arr = [0]           # yaw - theta - pi
	angle_2_arr = [0]           # phi + pitch
	dis_err = [0]               # simply distance from the origin
	u1 =[0]
	upitch_arr = [0]
	uyaw_arr =[0]
	yaw_align = [0]
	pitch_align_arr = [0]

	rospy.init_node("speed_controller")
	# obj = TutorialDPController()
	sub = rospy.Subscriber("/rexrov/pose_gt", Odometry, newOdom)
	pub = rospy.Publisher("/rexrov/thruster_manager/input_stamped", WrenchStamped, queue_size = 1)
	speed = WrenchStamped()
	ra = rospy.Rate(20)

	goal = Point()
	goal.x = 0                  # constant goal x
	goal.y = 0                  # constant goal y
	goal.z = 0                  # constant goal z
		
	while not rospy.is_shutdown() and k < 1000 and r > 1:
		
		K_alpha = 0.22
		K_beta = 0.33
		l.append((k+1)/100)

		# add the current x, y, z co-ordinates to their respective lists				
		x1.append(x)
		y1.append(y)
		z1.append(z)

		# yaw is in (0, 2pi) and should be brought to (-pi, pi)		
		# append the yaw and theta angles to their respective lists
		if yaw > np.pi : 
			yaw = yaw - 2*np.pi            
		yaw_arr.append(yaw)
		theta_arr.append(theta)  
		phi_arr.append(phi)
		pitch_arr.append(pitch)

		# calculate the input commands from the control logic
		#u_yaw = -K_alpha*np.sign(yaw - theta + np.pi )# - np.pi)
		u_yaw = 0
		u_pitch = 0
		#yaw_mult = 0
		if theta >= 0 : 
			x_align = theta - np.pi
			yaw_mult = K_alpha*(int(abs(x_align - yaw) >= 0.2)) + 0
			u_yaw = int(x_align <= yaw <= theta)*yaw_mult - int((np.pi >= yaw > theta) or (-np.pi <= yaw < x_align))*yaw_mult
		elif theta < 0 :
			x_align = np.pi + theta
			yaw_mult = K_alpha*(int(abs(x_align - yaw) >= 0.2)) + 0
			u_yaw = int((-np.pi <= yaw < theta) or (x_align < yaw <= np.pi))*yaw_mult - int(theta <= yaw <= x_align)*yaw_mult
		print("yaw: ",yaw, " ", "theta: ", theta, " ", "checker : ", u_yaw, "checker2: ", " ", abs(yaw-x_align))#yaw-theta+np.pi)
		yaw_align.append(x_align - yaw + int(x_align-yaw < -np.pi)*2*np.pi)
		pitch_align = phi
		u_pitch = -int(pitch_align >= pitch)*K_beta + int(pitch_align < pitch)*K_beta
		pitch_align_arr.append(pitch_align - pitch)
		#u_pitch = K_beta*np.sign(pitch + phi)
		#print("phi: ", " ", phi, "pitch: ", " ", pitch, "u_pitch: ", " ", u_pitch)
		dis_err.append(r)
		upitch_arr.append(u_pitch)
		uyaw_arr.append(u_yaw)
		
		# tau = np.array([10, 0, 0, 0, u_pitch, u_yaw])
		# obj.publish_control_wrench(tau)
				
		# put the input commands in the required variable to be published
		speed.header.stamp = rospy.Time.now()
		speed.header.frame_id = None
		speed.wrench.force.x = 1200*(int(r > 0.5))
		speed.wrench.force.y = 0
		speed.wrench.force.z = 0*500*(int(r>0.1))
		speed.wrench.torque.x = 0
		speed.wrench.torque.y = 6000*u_pitch*(int(r>0.5))
		speed.wrench.torque.z = 1700*u_yaw*(int(r>0.5))		
		#speed.angular.z = u_yaw
		#speed.angular.x = u_pitch
		
		print('executed')
		k = k+1
				
		pub.publish(speed)
		ra.sleep()


cols = ['time', 'xpos', 'ypos', 'zpos', 'yaw command', 'yaw error', 'pitch command', 'pitch error']
df = pd.DataFrame(list(zip(l, x1, y1, z1, uyaw_arr, yaw_align, upitch_arr, pitch_align_arr)), columns = cols)
df.to_csv('/home/shantanu18/Desktop/btp_runs/run_data_1.csv')
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x1, y1, z1)
plt.show() 

plt.plot(l, uyaw_arr, label = 'yaw command')
#plt.plot(l, upitch_arr)
plt.plot(l, yaw_align, label = 'yaw alignment error')
plt.legend()
plt.show()

plt.plot(l, upitch_arr, label = 'pitch command')
plt.plot(l, pitch_align_arr, label = 'pitch alignment error')
#plt.plot(l, pitch_arr, label = 'pitch')
#plt.plot(l, phi_arr, label = 'phi')
plt.legend()
plt.show()

plt.plot(l, pitch_align_arr, label = 'pitch alignmenr error')
plt.plot(l, yaw_align, label  = 'yaw alignmner error')
plt.legend()
plt.show()




