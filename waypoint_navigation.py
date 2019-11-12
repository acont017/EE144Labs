#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import matplotlib.pyplot as plt

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

        self.pose = Pose2D()
        self.trajectory = list()
        self.rate = rospy.Rate(10)
        self.reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        for i in range (10):				#Need a loop which resets odom values, process takes time.
            self.reset_odom.publish(Empty())
            self.rate.sleep()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.vel = Twist()
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

        self.logging_counter = 0

        self.T = 3  #Time to travel point to point.

        try:
            self.run()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.visualization()


    def run(self):
        # follow the sequence of waypoints
        WAYPOINTS = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                     [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                     [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0]]
        for point in WAYPOINTS:
            self.move_to_point(point)
        self.stop()
        rospy.loginfo("Action done.")


    def move_to_point(self, point):
        # please complete this function
        # hint: you can have access to x, y by point[0], point[1]
            Mx = np.array([[0, 0 , 0, 1], [self.T**3, self.T**2, self.T, 1], [0, 0, 1, 0], [3*self.T**2, 2*self.T, 1, 0]])
            My = np.array([[0, 0 , 0, 1], [self.T**3, self.T**2, self.T, 1], [0, 0, 1, 0], [3*self.T**2, 2*self.T, 1, 0]])
            x = np.array([self.pose.x, point[0], self.vel.linear.x, 0.25])
            y = np.array([self.pose.y, point[0], self.vel.linear.y, 0.25])
            ax = np.linalg.solve(Mx,x)
            ay = np.linalg.solve(My,y)
            poly_x = np.poly1d([ax[0],ax[1],ax[2],ax[3]])
            poly_y = np.poly1d([ay[0],ax[1],ax[2],ax[3]])
            print 'x =', poly_x, 'y = ', poly_y
            theta = 0
            for t in np.arange(0,self.T,0.1):
                polyder_x = np.polyder(poly_x, 1)
                polyder_y = np.polyder(poly_y, 1)
                self.vel.linear.x = sqrt(polyder_x(t)**2 + polyder_y(t)**2)
                print 't =', t, 'velocity', self.vel.linear.x
                old_theta = theta
                theta = atan2(poly_y(t), poly_x(t))
                self.vel.angular.z = (old_theta - theta)/0.1
                self.vel_pub.publish(self.vel)
                self.rate.sleep()


    def visualization(self):
        # plot trajectory
        data = np.array(self.trajectory)
        #np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        #plt.plot(data[:,0],data[:,1])
        #plt.show()


    def stop(self):
        # send zero velocity to robot
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)


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
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
