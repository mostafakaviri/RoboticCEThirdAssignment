#!/usr/bin/python3

import rospy, tf, math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("control_node" , anonymous=True)
        rospy.on_shutdown(self.on_shutdown)
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = .1
        self.angular_speed = .2 
        self.stop_distance = .1
        self.curent_x = 0
        self.curent_y = 0

        self.k_p = 2
        self.k_d = 10
        self.k_i = 0

        self.dt = 0.005
        rate = 1/self.dt
        self.r = rospy.Rate(rate)

        self.next_x = 0
        self.next_y = 0

        self.current_yaw = 0

        self.errs = []
        self.errs_d = []

        self.errs.append(0)
        self.errs.append(0)

        self.errs_d.append(0)
        self.errs_d.append(0)

        self.destination_points = [[0,2],[-1,2],[-2,2],[-3,2],[-3,1],[-3,0],[-3,-1],[-3,-2],[-2,-2],[-1,-2],[0,-2],[1,-2],[2,-2],[3,-2],[3,-1],[3,0],[3,1],[3,2],[2,2],[1,2], [0,2]]
        self.destination_points = self.destination_points[::-1]
        print(self.destination_points)

        self.buff = self.destination_points.pop()
        self.next_x = self.buff[0]
        self.next_y = self.buff[1]

        self.get_destination_heading()

         
    def get_destination_heading(self):

        deltay = self.next_y - self.curent_y
        deltax = self.next_x - self.curent_x
        
        self.goal_angle = math.atan2(deltay, deltax)
    
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation

        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        if yaw < 0 :
            yaw += 2 * math.pi


        return yaw

    def self_position(self):

        msg = rospy.wait_for_message("/odom" , Odometry)

        self.curent_x = msg.pose.pose.position.x
        self.curent_y = msg.pose.pose.position.y
                       

    def goal_ajacency(self):

        return math.sqrt(math.pow(self.next_x - self.curent_x, 2) + math.pow(self.next_y - self.curent_y, 2))

    def target_angel(self):
        
        d = math.atan2(self.next_y - self.curent_y, self.next_x - self.curent_x)

        if d < 0:
            d += 2 * math.pi
        

        return d
    
    def status_update(self):

        self.current_yaw = self.get_heading()
        
        self.self_position()

        self.goal_distance = self.goal_ajacency()

        if self.goal_distance < self.stop_distance :

            self.cmd_publisher.publish(Twist())

            self.buff = self.destination_points.pop()

            self.next_x = self.buff[0]
            self.next_y = self.buff[1]

        self.get_destination_heading()

        

        if abs(self.get_heading() - self.target_angel()) < math.pi :

            self.errs.append(self.get_heading() - self.target_angel())

        else :

            if self.target_angel() < math.pi :

                self.errs.append(self.get_heading() - 2 * math.pi - self.target_angel())

            if self.get_heading() < math.pi:

                self.errs.append(self.get_heading() - self.target_angel() + 2 * math.pi)
        
        self.errs_d.append(self.goal_distance)

            
            

    def control_effort(self):
        
        p = self.k_p * self.errs[-1] 
        d = self.k_d * (self.errs[-1]-self.errs[-2])
        i = self.k_i * sum(self.errs)
        return p + d + i
        
    def run(self):
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.linear_speed

        print(f"x = {self.next_x}, y = {self.next_y}")

        while not rospy.is_shutdown():

            # print("goh2")

            self.status_update()

            # print("goh2")
            
            move_cmd.angular.z = -self.control_effort()

            # move_cmd.linear.x = self.k_p * self.errs_d[-1]/10 + self.k_d *  (self.errs_d[-1]-self.errs_d[-2]) /10  + self.k_i * sum(self.errs_d) /10     

            move_cmd.linear.x = self.linear_speed

            self.cmd_publisher.publish(move_cmd)   

            print(f"head = {self.get_heading()} target = {self.target_angel()} error = {self.errs[-1]}")
            
            # rospy.loginfo(f"error : {self.errs[-1]} speed : {move_cmd.linear.x} theta : {move_cmd.angular.z}")

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

    controller = Controller()
    
    controller.run()
    # while True:
    #     print(controller.get_next_destination().next_x)