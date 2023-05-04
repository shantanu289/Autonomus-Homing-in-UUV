#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist, Vector3, Wrench, WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import atan2
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from uuv_control_interfaces import DPControllerBase
import pandas as pd
import time

x = 0.0                # initial x
y = 0.0                # initial y
z = -50.0                # initial z
roll = 0.0             # initial roll   (about x axis)
pitch = 0.87            # initial pitch  (aobut y axis)
yaw = 0.52                # initial yaw    (about z axis)
x_actual = 10.0 
y_actual = 10.0 
z_actual = -10.0
theta = 0.78539816339745
phi = np.pi/2 - 0.81482691637099
r = 5.8309518948453
rot_matrix = np.zeros((3, 3))	
goal = Point()
goal.x = 5                  # constant goal x
goal.y = 0                  # constant goal y
goal.z = -15                  # constant goal z
xscan = [] 
yscan = [] 
zscan = [] 
x1 = [10]
y1 = [10]
z1 = [-50]
scan_count = []
scan_count_current = 0 
roll_array = [0] 
pitch_array = [0.5] 
yaw_array = [0] 


goal1 = Point(-5, 5, -50) 
goal2 = Point(-5, -5, -50) 
goal3 = Point(5, -5, -50)
goal4 = Point(5, 5, -50)
goal5 = Point(10, 9, -10)
goal6 = Point(10, 8.8, -10)
goal7 = Point(10, 8.6, -10)
goal8 = Point(10, 8.4, -10)
goal9 = Point(10, 8.2, -10)
goal10 = Point(10, 8, -10)
goal11 = Point(10, 7.8, -10)
goal12 = Point(10, 7.6, -10) 
goal13 = Point(10, 7.4, -10)
waypoints = [goal1, goal2, goal3, goal4]#, goal5, goal6, goal7, goal8, goal9, goal10, goal11, goal12, goal13]
# waypoints list can be appended depending on which and how many goals we want the vehicle to follow. 
goal = waypoints[0]  


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
	global x_actual
	global y_actual 
	global z_actual 
	global rot_matrix
	global x1, y1, z1
	x = msg.pose.pose.position.x - goal.x
	y = msg.pose.pose.position.y - goal.y
	z = msg.pose.pose.position.z - goal.z
	x_actual = x + goal.x 
	y_actual = y + goal.y 
	z_actual = z + goal.z 
	rot_q = msg.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	(r, phi, theta) = cartesian_to_spherical(x, y, z)
	#rot_matrix = get_rotation_matrix(yaw, pitch, roll) 
	#x1.append(x_actual)
	#y1.append(y_actual)
	#z1.append(z_actual)
	#print("Roll: ", roll, "Pitch: ", pitch, "Yaw: ", yaw)
        # roll, pitch, yaw is for the heading direction
	# r, phi, theta is for the current robot position 

def cartesian_to_spherical(a, b, c):
	r_hat = (a**2 + b**2 + c**2)**0.5          # always positive
	phi_hat = np.arcsin(c/r_hat)               # between -pi/2 and pi/2      perpendicular to x-y plane
	theta_hat = atan2(b, a)                    # between -pi and pi          in x-y plane
	return r_hat, phi_hat, theta_hat

def get_rotation_matrix(aa, bb, cc):
	    # note that this function is for reverse rotation 
		# we have co-ordinates wrt rotated frame and we want them wrt original (non rotated) frame 
        t11 = np.cos(-aa)*np.cos(-bb)
        t12 = np.sin(-aa)*np.cos(-cc) + np.cos(-aa)*np.sin(-bb)*np.sin(-cc) 
        t13 = np.sin(-aa)*np.sin(-cc) - np.cos(-aa)*np.cos(-cc)*np.sin(-bb)
        t21 = -np.sin(-aa)*np.cos(-bb)
        t22 = np.cos(-aa)*np.cos(-cc) - np.sin(-aa)*np.sin(-bb)*np.sin(-cc)
        t23 = np.cos(-aa)*np.sin(-cc) + np.sin(-aa)*np.sin(-bb)*np.cos(-cc)
        t31 = np.sin(-bb)
        t32 = -np.cos(-bb)*np.sin(-cc)
        t33 = np.cos(-bb)*np.cos(-cc)
        mat = np.array([[t11, t12, t13], [t21, t22, t23], [t31, t32, t33]])
        return mat


def laserscan_callback(msg):
        global rot_matrix 
        global x_actual
        global y_actual
        global z_actual
        global xscan, yscan, zscan, scan_count, scan_count_current
        global yaw, roll, pitch, roll_array, yaw_array, pitch_array
        global x1, y1, z1
        a_min = msg.angle_min 
        a_max = msg.angle_max 
        a_inc = msg.angle_increment
        rphi = 0 
        scan_range = msg.ranges
        scan_count_current += 1
        for i in range(len(scan_range)):
            if scan_range[i] != np.inf : 
                ai = a_min + a_inc*i                                         # calculating scan angle
                ri = scan_range[i]                                           # getting scanned distance
                xi = ri*np.cos(ai)*np.sin(rphi) + 1.15                       # getting x co-ord wrt vehicle
                yi = ri*np.sin(-ai)                                          # getting y co-ord wrt vehicle
                zi = ri*np.cos(ai)*np.cos(rphi) + 0.46                       # getting abs(z) co-ord wrt vehicle
                zi = -zi                                                     # getting z co-ord wrt vehicle
				# note that we have added 1.15 and 0.46 for x and z since we need to consider the location of the SONAR wrt to vehicle 
                rot_matrix = get_rotation_matrix(yaw, pitch, roll)           # get rotation matrix
                new_coord = np.matmul(rot_matrix, np.array([[xi], [yi], [zi]])) #calculate rotated co-ordinates
                #new_coord[2,0] = -new_coord[2,0]  
                act_coord = new_coord + np.array([[x_actual], [y_actual], [z_actual]]) # calculate translated co-ordinates
				# add scan info to respective lists for storage and data analysis
                x1.append(x_actual)                 
                y1.append(y_actual)
                z1.append(z_actual)
                xscan.append(act_coord[0,0])
                yscan.append(act_coord[1,0])
                zscan.append(act_coord[2,0])
                scan_count.append(scan_count_current)
                roll_array.append(roll) 
                yaw_array.append(yaw) 
                pitch_array.append(pitch)
            	    
        #print(len(xscan))
                
if __name__ == '__main__':
	
	#k=0
	l=[0] #l is time

	#x1=[10]#x-coordinate of robot
	#y1=[10]#y-coordinate of robot
	#z1=[-10]#z-coordinate of robot
	


	#yaw_arr = [0.52]               # yaw angle list
	#pitch_arr = [0]             # pitch angle list
	#roll_arr = [0]              # roll angle list
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
	sub2 = rospy.Subscriber("/rexrov/sonar", LaserScan, laserscan_callback)
	pub = rospy.Publisher("/rexrov/thruster_manager/input_stamped", WrenchStamped, queue_size = 1)
	speed = WrenchStamped()
	ra = rospy.Rate(20)
	k = rospy.get_time()
#	while rospy.get_time() - k < 100 :
#		print(rospy.get_time() - k)
#		speed.wrench.torque.z = 100
#		pub.publish(speed)
#		ra.sleep()
	
	for i in range(len(waypoints)):
		goal = waypoints[i]
		
		#rospy.init_node("speed_controller")
		# obj = TutorialDPController()
		#sub = rospy.Subscriber("/rexrov/pose_gt", Odometry, newOdom)
		#pub = rospy.Publisher("/rexrov/thruster_manager/input_stamped", WrenchStamped, queue_size = 1)
		#speed = WrenchStamped()
		#ra = rospy.Rate(20)
		

		#goal = Point()
		#goal.x = 0                  # constant goal x
		#goal.y = 0                  # constant goal y
		#goal.z = 0                  # constant goal z
		time.sleep(2)
			
		while not rospy.is_shutdown() and k < 10000 and r > 1:
			
			K_alpha = 0.22
			K_beta = 0.33
			l.append((k+1)/100)

			# add the current x, y, z co-ordinates to their respective lists				
			#x1.append(x_actual)
			#y1.append(y_actual)
			#z1.append(z_actual)

			# yaw is in (0, 2pi) and should be brought to (-pi, pi)		
			# append the yaw and theta angles to their respective lists
			if yaw > np.pi : 
				yaw = yaw - 2*np.pi            
			#yaw_arr.append(yaw)
			#theta_arr.append(theta)  
			#phi_arr.append(phi)
			#pitch_arr.append(pitch)

			# calculate the input commands from the control logic
			#u_yaw = -K_alpha*np.sign(yaw - theta + np.pi )# - np.pi)
			u_yaw = 0
			u_pitch = 0
			#yaw_mult = 0
			if theta >= 0 : 
				x_align = theta - np.pi
				yaw_mult = K_alpha*(int(abs(x_align - yaw) >= 0.1)) + 0
				u_yaw = int(x_align <= yaw <= theta)*yaw_mult - int((np.pi >= yaw > theta) or (-np.pi <= yaw < x_align))*yaw_mult
			elif theta < 0 :
				x_align = np.pi + theta
				yaw_mult = K_alpha*(int(abs(x_align - yaw) >= 0.1)) + 0
				u_yaw = int((-np.pi <= yaw < theta) or (x_align < yaw <= np.pi))*yaw_mult - int(theta <= yaw <= x_align)*yaw_mult
			#print("yaw: ",yaw, " ", "theta: ", theta, " ", "checker : ", u_yaw, "checker2: ", " ", abs(yaw-x_align))#yaw-theta+np.pi)
			yaw_align.append(x_align - yaw + int(x_align-yaw < -np.pi)*2*np.pi)
			pitch_align = phi
			u_pitch = -int(pitch_align >= pitch)*K_beta*(int(abs(pitch_align - pitch)>=0.2)) + int(pitch_align < pitch)*K_beta*(int(abs(pitch-pitch_align)<0.2)) + 0
			pitch_align_arr.append(pitch_align - pitch)
			#u_pitch = K_beta*np.sign(pitch + phi)
			#print("phi: ", " ", phi, "pitch: ", " ", pitch, "u_pitch: ", " ", u_pitch)
			#dis_err.append(r)
			#upitch_arr.append(u_pitch)
			#uyaw_arr.append(u_yaw)
			
			# tau = np.array([10, 0, 0, 0, u_pitch, u_yaw])
			# obj.publish_control_wrench(tau)
					
			# put the input commands in the required variable to be published
			# change the control parameters below. 
			speed.header.stamp = rospy.Time.now()
			speed.header.frame_id = None
			speed.wrench.force.x = 800*(int(r > 0.1))
			speed.wrench.force.y = 0
			speed.wrench.force.z = 100*(int(r>0.1))
			speed.wrench.torque.x = 0
			speed.wrench.torque.y = 15000*u_pitch*(int(r>0.1))
			speed.wrench.torque.z = 1800*u_yaw*(int(r>0.1))		
			#speed.angular.z = u_yaw
			#speed.angular.x = u_pitch
			
			print('executed : r = %s'%(r))
			k = k+1
					
			pub.publish(speed)
			ra.sleep()
		
		print("Checkpoint: %s"%(i), k)
		
		

	 
	 
		
# storing data, exporting and plotting.                 
print(sum(xscan)/len(xscan), sum(yscan)/len(yscan), sum(zscan)/len(zscan))
#cols = ['time', 'xpos', 'ypos', 'zpos', 'yaw command', 'yaw error', 'pitch command', 'pitch error']
#df = pd.DataFrame(list(zip(l, x1, y1, z1, uyaw_arr, yaw_align, upitch_arr, pitch_align_arr)), columns = cols)
cols_scan = ['xscan', 'yscan', 'zscan', 'scancount']
cols_pos = ['x', 'y', 'z']
cols_or = ['roll', 'pitch', 'yaw']
df1 = pd.DataFrame(list(zip(x1, y1, z1)), columns = cols_pos)
df2 = pd.DataFrame(list(zip(xscan, yscan, zscan, scan_count)), columns = cols_scan)
df3 = pd.DataFrame(list(zip(roll_array, pitch_array, yaw_array)), columns = cols_or)
df1.to_csv('/home/shantanu18/Desktop/btp_runs/pos_run_35.csv')
df2.to_csv('/home/shantanu18/Desktop/btp_runs/scan_run_35.csv')
df3.to_csv('/home/shantanu18/Desktop/btp_runs/or_run_35.csv')
#fig = plt.figure()
#ax = plt.axes(projection='3d')
#ax.plot3D(x1, y1, z1)
#plt.scatter(xscan, yscan) 
#plt.show() 
#plt.scatter(xscan, zscan) 
#plt.show()
#plt.scatter(yscan, zscan)
#plt.show()
#plt.plot(l, uyaw_arr, label = 'yaw command')
#plt.plot(l, upitch_arr)
#plt.plot(l, yaw_align, label = 'yaw alignment error')
#plt.legend()
#plt.show()

#plt.plot(l, upitch_arr, label = 'pitch command')
#plt.plot(l, pitch_align_arr, label = 'pitch alignment error')
#plt.plot(l, pitch_arr, label = 'pitch')
#plt.plot(l, phi_arr, label = 'phi')
#plt.legend()
#plt.show()

#plt.plot(l, pitch_align_arr, label = 'pitch alignmenr error')
#plt.plot(l, yaw_align, label  = 'yaw alignmner error')
#plt.legend()
#plt.show()





