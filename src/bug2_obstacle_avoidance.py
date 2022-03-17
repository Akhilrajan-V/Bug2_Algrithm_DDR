#!/usr/bin/env python
from turtle import right
import rospy
import math

import numpy as np
from std_msgs.msg import Float64

from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from tf import transformations


goal_x = 10
goal_y = 0

position = None

current_x = 0
current_y = 0
yaw_orient = 0

initial_x = 0
initial_y = 0

go_to_goal = True
follow_obstacle = False
turned_left_1 = False

yaw_precision = 2.0 * (math.pi / 180) 
rot_angle = 90 * (math.pi / 180)

regions_= {
        'right':  0,
        'fright': 0,
        'front':  0,
        'fleft':  0,
        'left':   0,
    }
    

laser_detect_dist = 1.0
traj_threshold = 0.3

# dist_to_traj = 0

turned_left = False

hit_point_x = hit_point_y = hit_point_yaw = 0
leave_point_yaw = 0


def turn_left():
    pub_move1.publish(-control_speed)#forward 
    pub_move2.publish(-control_speed)#forward
    pub_move3.publish(-control_speed)#reverse
    pub_move4.publish(-control_speed)#reverse
    # print('turning left')
    rospy.loginfo("Turning Left")
    rospy.sleep(0.2)


def turn_right():
    pub_move1.publish(control_speed)
    pub_move2.publish(control_speed)
    pub_move3.publish(control_speed)
    pub_move4.publish(control_speed)
    # print('turning right')
    rospy.loginfo("Turning Right")
    rospy.sleep(0.2)

def move_forward():
    pub_move1.publish(-control_speed)
    pub_move2.publish(control_speed)
    pub_move3.publish(-control_speed)
    pub_move4.publish(control_speed)
    rospy.sleep(0.2)

def stop_robot():
    pub_move1.publish(0)#forward 
    pub_move3.publish(0)#forward

    pub_move2.publish(0)#reverse
    pub_move4.publish(0)#reverse



def laser_callback(laser_msg):   
    global regions_
    regions_= {
        'right':  min(min(laser_msg.ranges[0:143]), 10),
        'fright': min(min(laser_msg.ranges[144:287]), 10),
        'front':  min(min(laser_msg.ranges[288:431]), 10),
        'fleft':  min(min(laser_msg.ranges[432:575]), 10),
        'left':   min(min(laser_msg.ranges[576:719]), 10),
    }
    

def odom_callback(odom_msg):
    
    global position 
    global yaw_orient 
    global current_x
    global current_y 
    position = odom_msg.pose.pose.position

    quarternion = (odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w)
    
    quart_to_euler = transformations.euler_from_quaternion(quarternion)
    yaw_orient = quart_to_euler[2]
    current_y  = position.y
    current_x  = position.x


def dist_to_line():
    # global dist_to_traj

    numerator = math.fabs((goal_x - initial_x)*(initial_y - current_y) - (initial_x - current_x)*(goal_y - initial_y))
    denomenator = math.sqrt((math.pow((goal_x - initial_x),2))+(math.pow((goal_y - initial_y),2)))
    dist_to_traj = numerator/denomenator
    return dist_to_traj

def check_goal_reached():
    
    x_dif = goal_x - current_x
    y_dif = goal_y - current_y

    if (abs(x_dif == 0) and abs(y_dif == 0)):
        return True
    else:
        return False


count = 0

def move_to_goal():
    global hit_point_x, hit_point_y, hit_point_yaw, regions_, count
    global go_to_goal, follow_obstacle, turned_left, turn_l_yaw
    
    goal_reached = check_goal_reached()
    
    if goal_reached == False:

        if((turned_left == False) and (regions_['front'] > laser_detect_dist)):
            
            global follow_obstacle
            # move forward
            move_forward()

        if(regions_['front']<=laser_detect_dist):

            if count == 0:
               
                turn_l_yaw = hit_point_yaw + rot_angle
                count += 1

            if(turned_left == False):
                        
                # turn left 
                turn_left()

                # print('current_yaw', yaw_orient)
                # print('tunr',turn_l_yaw)
            
        elif (regions_['front'] > (laser_detect_dist) and regions_['right']< laser_detect_dist):    

            turned_left = True        
            follow_obstacle = True
            go_to_goal = False
            # print('turned')

    elif(goal_reached == True):
        stop_robot()
        rospy.on_shutdown('Done')

def avoid_obstacle():
    global go_to_goal, follow_obstacle, regions_, leave_point_yaw
    dist_to_traj = dist_to_line()
    count_1=0
    global turned_left_1 
    
    if (follow_obstacle == True and regions_['right']<laser_detect_dist):
        # move forward
        pub_move1.publish(-control_speed)
        pub_move2.publish(control_speed)
        pub_move3.publish(-control_speed)
        pub_move4.publish(control_speed)

    elif (dist_to_traj>traj_threshold and regions_['right'] > (laser_detect_dist) ):
        
        #turn right 
        turn_right()
        # print('turning right')
      
    elif(dist_to_traj <= traj_threshold):

        if((turned_left_1 == False) and (regions_['right']<laser_detect_dist)):
                    
            # turn left 
            turn_left()
         
        elif (regions_['right']> 3.0):    
                
            stop_robot()
            
            turned_left_1 = True        
            follow_obstacle = True
            go_to_goal = True
            
        rospy.sleep(1)
             

       
rospy.init_node('model4_new_teleop')

# pub_steer = rospy.Publisher('/model4_new/steering_controller/command', Float64, queue_size=10) 
pub_move1 = rospy.Publisher('/model4_new/drive_wheel1_controller/command', Float64, queue_size=10)
pub_move2 = rospy.Publisher('/model4_new/drive_wheel2_controller/command', Float64, queue_size=10)
pub_move3 = rospy.Publisher('/model4_new/drive_wheel3_controller/command', Float64, queue_size=10)
pub_move4 = rospy.Publisher('/model4_new/drive_wheel4_controller/command', Float64, queue_size=10) 

laser_sub = rospy.Subscriber('/model4_new/laser_scan', LaserScan, laser_callback)
odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, odom_callback)

count = 0
acc = 0.1
target_speed = 0
target_turn = 0
control_speed = 4


def permanent_stop():
    stop_robot()
    rospy.loginfo("Done")
    rospy.on_shutdown()


rate = rospy.Rate(1)


def main():
    while not rospy.is_shutdown():
                
        dist_to_traj = dist_to_line()
        rospy.loginfo("yaw =")
        rospy.loginfo(yaw_orient)
        rospy.loginfo("Velocity = ")
        rospy.loginfo(control_speed)

        goal_reached = check_goal_reached()

        if (goal_reached == True):
            stop_robot()
            rospy.on_shutdown('Done')

        else:
            # print('distance to line =', dist_to_traj)
            if go_to_goal == True and dist_to_traj <= traj_threshold and follow_obstacle == False:

                move_to_goal() 
        
            elif go_to_goal == False and follow_obstacle == True:

                avoid_obstacle()
                # print('Avoiding obstacle')   
            elif(turned_left_1 == True):
                move_forward()
                rospy.sleep(2)
                permanent_stop()
            rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
