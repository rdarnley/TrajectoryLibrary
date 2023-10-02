#!/bin/bash

for i in 1 2 3 4 5
do
  export TURTLEBOT3_MODEL=burger
  source /home/cmu/catkin_ws/devel/setup.bash
  rostest my_turtlebot_sim my_turtlebot_sim.test   
done
