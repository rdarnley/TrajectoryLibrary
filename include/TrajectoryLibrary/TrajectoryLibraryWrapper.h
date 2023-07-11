#ifndef TRAJECTORY_LIBRARY_WRAPPER_H
#define TRAJECTORY_LIBRARY_WRAPPER_H

//// Normal Headers ////
#include <thread>
#include <condition_variable>

//// Package Headers ////
#include "TrajectoryLibrary/TrajectoryLibraryManager.h"
#include "TrajectoryLibrary/GoalManager.h"

//// ROS Headers ////
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>

bool fromOccupancyGrid( const nav_msgs::OccupancyGrid& msg, std::shared_ptr<Costmap> & costmap );


class TrajectoryLibraryWrapper {
    public:

        /// @brief Constructor 
        TrajectoryLibraryWrapper();
        
        // /// @brief Destructor
        // ~TrajectoryLibraryWrapper() = default;

        void Loop();

    private:

        bool m_hasOdom;
        bool m_hasCostmap;

        std::string m_vehicleFrame;
        std::string m_baseFrame;

        nav_msgs::OccupancyGrid m_occupancyGrid;

        std::unique_ptr<TrajectoryLibraryManager> p_tlm;
        std::unique_ptr<GoalManager> p_gm;

        ////// ROS Library Pieces //////
        ros::NodeHandle m_nh;                               // Node Handle

        tf2_ros::Buffer m_tfBuffer;                        // TF Buffer
        tf2_ros::TransformListener m_tfListener;           // TF Listener

        ros::Subscriber m_odometry_sub;                     // Odometry Subscriber
        ros::Subscriber m_costmap_sub;                      // Costmap Subscriber
        
        /// @brief Vehicle Odometry Callback
        void odometryCallback(const nav_msgs::Odometry& msg);

        /// @brief Vehicle Costmap Callback
        void costmapCallback(const nav_msgs::OccupancyGrid& msg);

};

#endif



