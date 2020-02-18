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
        self.pose_mocap = Pose2D()
        self.trajectory = list()
        self.rate = rospy.Rate(10)
        self.reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.mocap_sub = rospy.Subscriber("mocap_node/robot11/pose2d", Pose2D, self.mocap_callback)
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
        #finally:
            #self.visualization()


    def run(self):
        # get waypoints from A star algorithm
        #T_ow = np.array([[cos(self.pose_mocap.theta), sin(self.pose_mocap.theta), 0, -(self.pose_mocap.x*cos(self.pose_mocap.theta) + self.pose_mocap.y*sin(self.pose_mocap.theta))],\
        #                [-(sin(self.pose_mocap.theta)), cos(self.pose_mocap.theta), 0, -(-self.pose_mocap.x*sin(self.pose_mocap.theta) + self.pose_mocap.y*cos(self.pose_mocap.theta))],\
        #                [0, 0, 1, 0], [0, 0, 0, 1]])
        print 'Transformation matrix created'
        waypoints = self.get_path_from_A_star()

        for i in range(len(waypoints)):
            # Until final point we pass current desired point and next desired point.
            if i < (len(waypoints) - 1):
                self.move_to_point(waypoints[i], waypoints[i+1])
            # For final point we pass in the opposite sign of current pos
            # this ensures the robot stops moving facing the direction it is currently
            # traveling.
            else:
                self.lastpoint = 1
                self.move_to_point(waypoints[i], [-1*self.pose_mocap.x, -1*self.pose_mocap.y])
        self.stop()
        rospy.loginfo("Action done.")
        self.visualization()


    def move_to_point(self, point, next_point):
            # M matrix is the same for both x and y components.
            M = np.array([[0,0,0,0,0,1], [self.T**5,self.T**4,self.T**3,self.T**2,self.T,1],\
             [0,0,0,0,1,0], [5*self.T**4,4*self.T**3,3*self.T**2,2*self.T, 1, 0],[0,0,0,2,0,0],\
             [20*self.T**3,12*self.T**2,6*self.T,2,0,0]])
            # Vector pointed from current pos to desired point calculation.
            c_2_p = [point[0] - self.pose_mocap.x, point[1] - self.pose_mocap.y]
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
            x = np.array([self.pose_mocap.x, point[0], self.vel.linear.x*cos(angle),\
                .2*cos(angle_next), 0, 0])
            # y vector is set in the same fashion as the x vector.
            y = np.array([self.pose_mocap.y, point[1], self.vel.linear.x*sin(angle),\
                .2*sin(angle_next), 0, 0])
            if self.lastpoint:
                x = np.array([self.pose_mocap.x, point[0], self.vel.linear.x*cos(angle), 0, 0, 0])
                y = np.array([self.pose)_mocap.y, point[1], self.vel.linear.x*sin(angle), 0, 0, 0])
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
                t_error = (theta - self.pose_mocap.theta)
                # If statements to handle -pi,pi case.
                if t_error > pi:
                    t_error = t_error - 2*pi
                elif t_error < -pi:
                    t_error = t_error + 2*pi
                self.vel.angular.z = kp * (t_error) + kd * (t_error - t_error_old)
                self.vel_pub.publish(self.vel)
                self.rate.sleep()

    def get_path_from_A_star(self):
            start_point = [round(self.pose_mocap.x), round(self.pose_mocap.y)]
            if (self.pose_mocap.y > 0):
                self.end_point = [2.5, 3.5]
                self.obstacles = [[0,0], [1, 0], [2, 0], [3, 0], [4,0], [5,0],\
                                    [-1,0], [-2,0], [-3,0], [-4,0], [-5,0], [5,1],\
                                    [5,2], [5,3], [5,4], [5,5], [4,5], [3,5], [2,5],\
                                    [1,5], [0,5], [-1,5], [-2,5], [-3,5], [-4,5], [-5,5],\
                                    [-5,4], [-5,3], [-5,2], [-5,1], [-5,0], [-1,1], [-1,2],\
                                    [-1,3], [-2,3], [-2,2], [-2,1], [2,4], [2,3], [2,2],\
                                    [1,2], [1,3], [1,4]]
            else:
                self.end_point = [2.5, -1.5]
                self.obstacles = [[0,0], [1, 0], [2, 0], [3, 0], [4,0], [5,0],\
                                    [-1,0], [-2,0], [-3,0], [-4,0], [-5,0], [5,-1],\
                                    [5,-2], [5,-3], [5,-4], [5,-5], [4,-5], [3,-5], [2,-5],\
                                    [1,-5], [0,-5], [-1,-5], [-2,-5], [-3,-5], [-4,-5], [-5,-5],\
                                    [-5,-4], [-5,-3], [-5,-2], [-5,-1], [-5,0], [-1,-4], [-1,-3],\
                                    [-1,-2], [-2,-2], [-2,-3], [-2,-4], [2,-1], [2,-2], [2,-3],\
                                    [1,-1], [1,-2], [1,-3]]

            open_list = []
            close_list = []
            close_points = []
            optimal_path = []
            optimal_nodes = []
            END_REACHED = 0

            d = 4   # d is directions we can travel.
            gs = 0.5 # Grid sizeself.

            # Current node(point), total cost(prev cost, est ctg), parent node.
            open_list.append([start_point, self.cost(start_point, 0), 0])

            while END_REACHED == 0:
                curr_node = open_list[0]
                curr_point = curr_node[0]
                #print curr_point
                next_point = []
                for i in range(d):
                    if i == 0:
                        next_point = [curr_point[0] + 0.5, curr_point[1]]
                        if (next_point not in self.obstacles) and (next_point not in close_points):
                            open_list.append([next_point, self.cost(curr_node[0], curr_node[1]), curr_node])
                    elif i == 1:
                        next_point = [curr_point[0] - 0.5, curr_point[1]]
                        if (next_point not in self.obstacles) and (next_point not in close_points):
                            open_list.append([next_point, self.cost(curr_node[0], curr_node[1]), curr_node])
                    elif i == 2:
                        next_point = [curr_point[0], curr_point[1] + 0.5]
                        if (next_point not in self.obstacles) and (next_point not in close_points):
                            open_list.append([next_point, self.cost(curr_node[0], curr_node[1]), curr_node])
                    elif i == 3:
                        next_point = [curr_point[0], curr_point[1] - 0.5]
                        if (next_point not in self.obstacles) and (next_point not in close_points):
                            open_list.append([next_point, self.cost(curr_node[0], curr_node[1]), curr_node])
                    if next_point == self.end_point:
                        print 'FOUND END'
                        open_list = []
                        END_REACHED = 1
                        optimal_nodes.append([next_point, self.cost(curr_node[0], curr_node[1]), curr_node])
                if END_REACHED == 0:
                    open_list.remove(curr_node)
                    open_list.sort(key=lambda x:x[1])
                    #print 'sorted & popped:', open_list
                close_list.append(curr_node)
                close_points.append(curr_point)
            #print close_list
            if len(optimal_nodes) != 0:
                final_node = optimal_nodes[0]
                optimal_path.append(final_node[0])
                parent_node = final_node[2]
                while parent_node != 0:
                    optimal_path.append(parent_node[0])
                    parent_node = parent_node[2]
                optimal_path.reverse()
                optimal_path.pop(0)
            print 'Path found: ', optimal_path
            return (np.array(optimal_path)*gs)

    def cost(self, point1, past_cost):
            heur_cost = abs(self.end_point[0] - point1[0]) + abs(self.end_point[1] - point1[1])
            est_cost = heur_cost + past_cost + 1

            return est_cost

    def visualization(self):
        # plot trajectory
        data = np.array(self.trajectory)
        #np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        print data
        plt.plot(data[:,0],data[:,1])
        obstacles = 0.5*np.array(self.obstacles)
        plt.plot(obstacles[:,0], obstacles[:,1])
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
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


    def mocap_callback(self, msg):
        self.pose_mocap.theta = msg.theta
        self.pose_mocap.x = msg.x
        self.pose_mocap.y = msg.y



if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
