#!/usr/bin/python3

import rospy, tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import math

class PIDController():


    def __init__(self):
        
        rospy.init_node('point_follower', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        
        self.k_i = 0
        self.k_p = .5
        self.k_d = 5

        destination = [10, 0]
        
        self.dt = 0.005
        self.v = 0.6
        self.D = 2
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []
        self.errs_d = []
        self.errs.append(0)
        self.errs.append(0)

    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        ))

        return yaw

    def control_effort(self):
        
        p = self.k_p * self.errs[-1] 
        d = self.k_d * (self.errs[-1]-self.errs[-2])
        i = self.k_i * sum(self.errs)
        return p + d + i
    
    def self_position(self):

        msg = rospy.wait_for_message("/odom" , Odometry)

        export = []

        export.append(msg.pose.pose.position.x)
        export.append(msg.pose.pose.position.y)
                      
        return export

    def target_angel(self):
        
        d = math.atan2(-self.self_position()[1], 10 -  self.self_position()[0])

        return d
        
    def distance_clc(self):
        return math.sqrt( ( math.pow(self.self_position()[0] - 10, 2) ) + ( math.pow(self.self_position()[1],2) ) )
    
    def run(self):

        self.errs.append(self.get_heading() - self.target_angel())
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v


        while not rospy.is_shutdown():

            self.cmd_publisher.publish(move_cmd)

            self.errs.append(self.get_heading() - self.target_angel())

            self.errs_d.append(self.distance_clc())
            
            move_cmd.angular.z = -self.control_effort()

            # move_cmd.linear.x = self.k_p * self.errs_d[-1] / 5          
            move_cmd.linear.x = self.v
            rospy.loginfo(f"error : {self.errs[-1]} speed : {move_cmd.linear.x} theta : {move_cmd.angular.z}")
            

            self.r.sleep()

    def on_shutdown(self):

        rospy.loginfo("Stopping the robot...")
        self.cmd_publisher.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.savefig(f"errs_{self.k_p}_{self.k_d}_{self.k_i}.png")
        plt.show()

        rospy.sleep(1)

if __name__ == "__main__":

    controller = PIDController()
    
    controller.run()
    # while True:
    #     # print(controller.target_angel, end="  ")
    #     print(controller.get_heading, end="  ")
