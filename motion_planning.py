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

        self.T = 2.1  #Time to travel point to point.
        self.lastpoint = 0 	# Flag for last point.



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
        for i in range(14):
            # Until final point we pass current desired point and next desired point.
            if i < 13:
                self.move_to_point(WAYPOINTS[i], WAYPOINTS[i+1])
            # For final point we pass in the opposite sign of current pos
            # this ensures the robot stops moving facing the direction it is currently
            # traveling.
            else:
                self.lastpoint = 1
                self.move_to_point(WAYPOINTS[i], [-1*self.pose.x,0])
        self.stop()
        rospy.loginfo("Action done.")


    def move_to_point(self, point, next_point):
            # M matrix is the same for both x and y components.
            M = np.array([[0,0,0,0,0,1], [self.T**5,self.T**4,self.T**3,self.T**2,self.T,1],\
             [0,0,0,0,1,0], [5*self.T**4,4*self.T**3,3*self.T**2,2*self.T, 1, 0],[0,0,0,2,0,0],\
             [20*self.T**3,12*self.T**2,6*self.T,2,0,0]])
            # Vector pointed from current pos to desired point calculation.
            c_2_p = [point[0] - self.pose.x, point[1] - self.pose.y]
            # Vector pointed from desired point to next desired point calculation.
            p_2_np = [next_point[0] - point[0], next_point[1] - point[1]]
            # Angle that described c_2_p.
            angle = atan2(c_2_p[1], c_2_p[0])
            # Angle that described p_2_np.
            angle_next = atan2(p_2_np[1], p_2_np[0])
            # Vector of initial condtions for x component.
            # Elements are as follows: (x(0), x(T), dx/dt(0), dx/dt(T), dx^2/d^2t(0), dx^2/d^2t(T)).
            # Initial condtion velocity is set to current velocity to ensure smooth trajectory
            # rather than stopping and turning at each point.
            x = np.array([self.pose.x, point[0], self.vel.linear.x*cos(angle),\
                .2*cos(angle_next), 0, 0])
            # y vector is set in the same fashion as the x vector.
            y = np.array([self.pose.y, point[1], self.vel.linear.x*sin(angle),\
                .2*sin(angle_next), 0, 0])
            if self.lastpoint:
                x = np.array([self.pose.x, point[0], self.vel.linear.x*cos(angle), 0, 0, 0])
                y = np.array([self.pose.y, point[1], self.vel.linear.x*sin(angle), 0, 0, 0])
            # The coefficients of the 5th order polynomials are solved here.
            ax = np.linalg.solve(M,x)
            ay = np.linalg.solve(M,y)
            # Using the coefficients create our time scaled polynomial.
            poly_x = np.poly1d([ax[0],ax[1],ax[2],ax[3],ax[4],ax[5]])
            poly_y = np.poly1d([ay[0],ay[1],ay[2],ay[3],ay[4],ay[5]])

            # p and d coefficients are set for PD controller to control angular velocity.
            kp = 3
            kd = 0.4
            # Theta error
            t_error = 0
            # t is iterates through 0 to T sec at the same refresh rate as our robot.
            for t in np.arange(0,self.T,0.1):
                # We take the derivative of our polynomials.
                polyder_x = np.polyder(poly_x, 1)
                polyder_y = np.polyder(poly_y, 1)
                # The linear velocity is the magnitude of our x and y components.
                # It is scaled by an additional 10% to accomodate for drag on carpet.
                self.vel.linear.x = 1.1*sqrt(polyder_x(t)**2 + polyder_y(t)**2)
                # Velocity vector gives us direction robot should be facing at every time instant.
                # By taking arctan of components we can get angular position.
                theta = atan2(polyder_y(t), polyder_x(t))
                # Error for P control.
                t_error_old = t_error
                t_error = (theta - self.pose.theta)
                # If statements to handle -pi,pi case.
                if t_error > pi:
                    t_error = t_error - 2*pi
                elif t_error < -pi:
                    t_error = t_error + 2*pi
                self.vel.angular.z = kp * (t_error) + kd * (t_error - t_error_old)
                self.vel_pub.publish(self.vel)
                self.rate.sleep()

    def get_path_from_A_star(self):
            start_point = [0, 0]
            end_point = [5, 1]
            obstacles = [[2, -1], [2, 0], [2, 1], ...]
            # you may want to add more boundary grids to obstacles

            open_list = []
            close_list = []
            optimal_path = []

        return optimal_path

    def visualization(self):
        # plot trajectory
        data = np.array(self.trajectory)
        #np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:,0],data[:,1])
        plt.show()


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
        if self.logging_counter == 10:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
