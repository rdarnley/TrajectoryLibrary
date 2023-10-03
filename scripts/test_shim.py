#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from mmpug_msgs.msg import GoalInput
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node('test_shim')

    ## Create ROS Publishers
    ## Joy Pub Spoofs Behavior Executive So Vehicle Uses Waypoints
    ## WP Pub Publishes Target Waypoint To Trajectory Library
    joy_pub = rospy.Publisher('/ryanlaptop/joy', Joy, queue_size=1, latch=True)
    wp_pub = rospy.Publisher('/rc1/planner/goal_input', GoalInput, queue_size=1, latch=True)
    slam_safe_pub = rospy.Publisher('/rc1/planner/slam_safe_mode', Bool, queue_size=1, latch=True)


    ## Create Joy Message Spoof
    axes = [0.0 for i in range(8)]
    buttons = [0 for i in range(11)]
    buttons[2] = 1

    print(axes)
    print(buttons)


    ## Get Test Waypoint From Param Server
    wp_x = rospy.get_param("/waypoint_x")
    wp_y = rospy.get_param("/waypoint_y")
    wp_targetVel = rospy.get_param("/waypoint_velocity")


    ## Create Joy and WP Messages
    joy_msg = Joy()
    joy_msg.axes = axes
    joy_msg.buttons = buttons

    wp_msg = GoalInput()
    wp_msg.goal_x = wp_x
    wp_msg.goal_y = wp_y
    wp_msg.target_velocity = wp_targetVel

    slam_safe = Bool()
    slam_safe.data = True

    ## Arbitrarily wait so everything is set
    ## TODO - Make this more robust
    rospy.sleep(5)

    ## Publish Joy and WP Messages
    joy_pub.publish(joy_msg)
    wp_pub.publish(wp_msg)
    slam_safe_pub.publish(slam_safe)

    ## Keep Running
    ## TODO - Is this necessary?
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
