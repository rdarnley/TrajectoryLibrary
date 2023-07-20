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
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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

        double m_rate;

        std::string m_vehicleFrame;
        std::string m_baseFrame;

        nav_msgs::OccupancyGrid m_occupancyGrid;
        nav_msgs::Odometry m_odometry;

        std::unique_ptr<TrajectoryLibraryManager> p_tlm;

        ////// ROS Library Pieces //////
        ros::NodeHandle m_nh;                               // Node Handle

        tf2_ros::Buffer m_tfBuffer;                        // TF Buffer
        tf2_ros::TransformListener m_tfListener;           // TF Listener

        ros::Subscriber m_odometry_sub;                     // Odometry Subscriber
        ros::Subscriber m_costmap_sub;                      // Costmap Subscriber

        ros::Publisher m_trajectoryPub;
        ros::Publisher m_bestTrajectoryPub;
        ros::Publisher m_goalPub;

        /// @brief Vehicle Odometry Callback
        void odometryCallback(const nav_msgs::Odometry& msg);

        /// @brief Vehicle Costmap Callback
        void costmapCallback(const nav_msgs::OccupancyGrid& msg);

        visualization_msgs::MarkerArray toRviz(geometry_msgs::TransformStamped& tran);
        visualization_msgs::Marker toRviz(geometry_msgs::TransformStamped& tran, Trajectory & traj);
        visualization_msgs::MarkerArray toRviz(Waypoint & currentGoal);

};

#endif



