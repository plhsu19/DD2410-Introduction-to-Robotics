#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot
import math

## Use to transform between frames
tf_buffer = None
listener = None

## The exploration simple action client
goal_client = None
## The collision avoidance service client
control_client = None
## The velocity command publisher
pub = None

## The robots frame
robot_frame_id = "base_link"

## Max linear velocity (m/s)
max_linear_velocity = 0.5
## Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub

    # Call service client with path
    res = control_client(path)
    # print("The frame is: %s" %res.setpoint.header.frame_id)

    # Transform Setpoint from service client
    transform = tf_buffer.lookup_transform(robot_frame_id, "map", rospy.Time()) 
    transformed_setpoint = tf2_geometry_msgs.do_transform_point(res.setpoint, transform)
    

    # Create Twist message from the transformed Setpoint
    msg = Twist()
    msg.angular.z = 4 * atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    if msg.angular.z > max_angular_velocity:
        msg.angular.z = max_angular_velocity

    msg.linear.x = 0.5 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
    if msg.linear.x > max_linear_velocity:
        msg.linear.x = max_linear_velocity

    # Publish Twist
    pub.publish(msg)
   
    # Call service client again if the returned path is not empty and do stuff again
    new_path = res.new_path

    while new_path.poses:

        res = control_client(new_path)
        #tf_buffer = tf2_ros.Buffer()  #necessary?      
        #listener = tf2_ros.TransformListener(tf_buffer) #necessary?   
        transform = tf_buffer.lookup_transform(robot_frame_id, "map", rospy.Time()) #necessary?   
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(res.setpoint, transform)
        msg = Twist() #necessary?   
        msg.angular.z = 4 * atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        if msg.angular.z > max_angular_velocity:
            msg.angular.z = max_angular_velocity

        msg.linear.x = 0.5 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
        if msg.linear.x > max_linear_velocity:
            msg.linear.x = max_linear_velocity

        pub.publish(msg)

        new_path = res.new_path

        rate.sleep()
    
    print("new_path.poses = " + str(new_path.poses))
    # Send 0 control Twist to stop robot 
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)

    # Get new path from action server



def get_path():
    global goal_client

    while True:
        # Get path from action server
        goal_client.wait_for_server()
        
        goal_client.send_goal(0)

        goal_client.wait_for_result()

        result = goal_client.get_result()

        if result.gain == 0: #and also 'path' is empty?
            break
        print("gain = " + str(result.gain))
        # Call move with path from action server
        move(result.path)


if __name__ == "__main__":
    ## Init node
    rospy.init_node("controller")

    # Create TF buffer and listener
    tf_buffer = tf2_ros.Buffer()    
    listener = tf2_ros.TransformListener(tf_buffer) 

    # Init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #should later publish into topic '/cmd_vel' in move(path)
    rate = rospy.Rate(10) # 10hz

    ## Init simple action client
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Init service client (service provide by Collision_Avoidance)
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Call get path
    get_path()
     
    # Spin
    rospy.spin()
