#!/usr/bin/env python

import rospy
import math
import unittest
from gazebo_msgs.msg import ModelStates
from mmpug_msgs.msg import GoalInput


## Test Suite : Consists of multiple test cases
class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()
        self.loader = unittest.TestLoader()
        self.addTest(self.loader.loadTestsFromTestCase(TestBotMoves))


## Unit Test Case
class TestBotMoves(unittest.TestCase):
    def test_movement(self):
        rospy.init_node('test_movement', log_level=rospy.DEBUG)

        ## Get ROSPARAM Values To Perform Test
        waypoint_x = rospy.get_param("/waypoint_x")
        waypoint_y = rospy.get_param("/waypoint_y")
        waypoint_threshold = rospy.get_param("/waypoint_tolerance")
        vehicle_name = rospy.get_param("/active_vehicle")

        ## Initialize State
        self.idx = None
        self.x = 0.0
        self.y = 0.0

        self.wp_x = waypoint_x
        self.wp_y = waypoint_y
        threshold = waypoint_threshold
        self.vehicle = vehicle_name

        ## Set Up Subscribers
        ## Gazebo Model States Is Ground Truth For Vehicle Location As Per The Sim
        ## Goal Input Is The Waypoint That The Trajectory Library Is Using
        self.subscriber = rospy.Subscriber( '/gazebo/model_states', ModelStates, self.movement_callback)
        self.wp_sub = rospy.Subscriber( '/rc1/planner/goal_input', GoalInput, self.wp_callback)

        error = self.get_distance_from_wp()
        while error > threshold and not rospy.is_shutdown():
            rospy.loginfo('Error; %s m', round(error, 1))
            rospy.sleep(1)
            error = self.get_distance_from_wp()

        assert error <= threshold


    ## Waypoint Callback
    ## Also Grabbed From Param Server But Just In Case
    def wp_callback(self, data):
        self.wp_x = data.goal_x
        self.wp_y = data.goal_y


    ## Gazebo Model States Callback
    ## Uses Vehicle Name To Determine Which "Gazebo Model" Is In Position
    def movement_callback(self, data):
        if self.idx is None:
            self.idx = 0
            for name in data.name:
                if name == self.vehicle:
                    break
                self.idx += 1

        self.x = data.pose[self.idx].position.x
        self.y = data.pose[self.idx].position.y


    ## Get Distance From Waypoint
    def get_distance_from_wp(self):
        return math.sqrt(abs(self.x - self.wp_x) ** 2 + abs(self.y - self.wp_y) ** 2)


if __name__ == '__main__':
    import rostest

    ## Run single test case
    # rostest.rosrun('traj_lib_sim', 'test_bot_moves', TestBotMoves)

    ## Run test suite
    rostest.rosrun('traj_lib_sim', 'test_bot_moves', 'test_bot_moves.SuiteTest')
