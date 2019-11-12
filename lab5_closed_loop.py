#!/usr/bin/env python

import math
import matplotlib.pyplot as plt
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

class Turtlebot():
    def __init__(self):                                                       #Initalize Variables and Functions
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press CTRL + C to terminate")
	    self.rate = rospy.Rate(10)

        self.pose = Pose2D()
	    self.reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
	    for i in range (10):				#Need a loop which resets odom values, process takes time.
		    self.reset_odom.publish(Empty())
		    self.rate.sleep()
            self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.vel = Twist()
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
	    self.reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

	    self.x_pos = list()			#List that will track path traveled for plotting.
	    self.y_pos = list()

        self.logging_counter = 0
        self.run()
        self.visualization()


    def run(self):
	k = 0	                            #Number of sides of square traveled.
	theta_error = 0	                    #Initialized to set dependent variable "last_theta_error" below.
	while not rospy.is_shutdown():
		desired_theta = k*(math.pi/2)	#After every side traveled we need to turn pi/2 rad.
		last_theta_error = theta_error	#Used in derivative calculation.
        if k < 2:                       #Condition check to reduce redundant computations
            theta_error = (desired_theta - self.pose.theta)
            theta_derivative = (theta_error - last_theta_error) #Derivative of theta_error
		K_p = 3		                 #Values initially randomly selected but then
		K_d = 0.4 	                 #assigned based on experimental results.
		if(abs(theta_error) < 0.05):		#Want to ensure we don't move forward before we completed turn.
			self.vel.linear.x = 0.5		    #Conditional check waits until we are in acceptable
		else:					            #range before starting linear motion again.
			self.vel.linear.x = 0
		if (k == 0 and abs(self.pose.x - 2) < 0.07):	#Using odom readings we can tell when we reach a vertex
			k += 1					                    #of square.
			self.vel.linear.x = 0		                #Linear vel needs to be set to 0 so it'll stop before turning.
            print '2m, x, traveled'
		elif (k == 1 and abs(self.pose.y - 0.5) < 0.07):
			k += 1
			self.vel.linear.x = 0
            print '.5m, y, traveled'
		elif (k == 2):		                            #When our desired angle is pi we need a special case, since Gazebo has range of (-pi, pi).
			if (self.pose.theta < 0):	                #If angle is CCW from pi it'll be read as a value close to -pi.
				theta_error = (-(math.pi) - self.pose.theta)	#Desired theta is adjusted accordingly.
			else:
				theta_error = (desired_theta - self.pose.theta)
            theta_derivative = (theta_error - last_theta_error)
			if (abs(self.pose.x - 0) < 0.07):
				k += 1			           #For last length, desired angle is 3pi/2 but range is (-pi,pi)
				self.vel.linear.x = 0	   #below desired_theta is adjusted to handle this, but it would cause a fast CW
				self.vel.angular.z = 5	   #rotation. Angular vel is set positive for 1 cycle to ensure angle is negative.
				self.vel_pub.publish(self.vel)
                print '-2m, x, traveled'
				self.rate.sleep()
		elif (k == 3):
			desired_theta = -(math.pi/2)	                    #Adjustment from 3pi/2 to deal with Gazebo's range.
			theta_error = (desired_theta - self.pose.theta)		#Calculations need to be redone after adjustment.
			theta_derivative = (theta_error - last_theta_error)
			if(abs(theta_error) < 0.05):	                    #Error recalculated so check needs to be repreformed.
				self.vel.linear.x = 0.5
			else:
				self.vel.linear.x = 0
			if ((abs(self.pose.y - 0)) < 0.07): 	            #Assuming x pos ~ 0, when y pos gets close to 0 we have reached origin.
				self.vel.linear.x = 0
				self.vel_pub.publish(self.vel)
				self.rate.sleep()
				print 'Reached origin, Done!'
				k += 1				                            #k iterated one more time as a stop flag.
		#print ('Error in theta = ', theta_error)
		self.vel.angular.z = (K_p * theta_error) + (K_d * theta_derivative)
		#print 'Angular Vel = ', self.vel.angular.z
		self.vel_pub.publish(self.vel)
		if (k == 4):
			break
		self.rate.sleep()


    def visualization(self):
        plt.plot(self.x_pos, self.y_pos)
	    plt.grid(linestyle = '--')
	    plt.title('Trajectory')
	    plt.xlabel('X Axis')
	    plt.ylabel('Y Axis')
	    plt.show()
    pass


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Logging once every 100 times
        self.logging_counter += 1
	#self.rate.sleep()
        if self.logging_counter == 10:
			self.logging_counter = 0
			rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))
            		# Add your code here to save the trajectory
			self.x_pos.append(self.pose.x)
			self.y_pos.append(self.pose.y)



if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
