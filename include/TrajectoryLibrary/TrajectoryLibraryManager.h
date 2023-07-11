#ifndef TRAJECTORY_LIBRARY_MANAGER_H
#define TRAJECTORY_LIBRARY_MANAGER_H

#include <thread>
#include <condition_variable>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> 
#include <deque>
#include <unordered_map>

// Want to get rid of this stuff
#include "tf/transform_datatypes.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"

#include "TrajectoryLibrary/Costmap.h"
#include "TrajectoryLibrary/Trajectory.h"

class TrajectoryLibraryManager {
    public:

        /// @brief Constructor
        TrajectoryLibraryManager(double min_x, double min_y, double resolution, int width, int height);

        // /// @brief Destructor
        // ~TrajectoryLibraryManager();

        /// @brief Configure
        // Load Trajectory Library File
        bool configure(std::string filepath); // give filepath???

        bool processTrajectories(geometry_msgs::TransformStamped & tran);
        // bool getDebug();
        // bool getBestTrajectory();

        std::shared_ptr<Costmap> m_costmap;
        std::unordered_map<int, Trajectory> m_trajectories;


    private:

        // std::shared_ptr<std::vector<Trajectory>> m_trajectories;  // Is vector correct data structure for this???
        // std::unordered_map<int, std::vector<Trajectory>> m_trajectories;
                                                // Might want to use min heap
};

#endif



