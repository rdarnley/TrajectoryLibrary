#include "TrajectoryLibrary/TrajectoryLibraryWrapper.h"

//// Utility Functions
bool fromOccupancyGrid( const nav_msgs::OccupancyGrid & msg, std::shared_ptr<Costmap> & costmap ) {
  costmap->resolution = msg.info.resolution;
  costmap->width = msg.info.width;
  costmap->height = msg.info.height;
  
  costmap->min_x = msg.info.origin.position.x;
  costmap->max_x = costmap->min_x + costmap->width*costmap->resolution;
  costmap->min_y = msg.info.origin.position.y;
  costmap->max_y = costmap->min_y + costmap->height*costmap->resolution;
  
  costmap->data.clear();
  for ( auto & cell : msg.data ) {
    costmap->data.push_back( (uint8_t) cell );
  }
  
  return true;
}

void transformMsgToEigen( const geometry_msgs::Transform &m, Eigen::Affine3d &e){
    e = Eigen::Translation3d(   m.translation.x,
                                m.translation.y,
                                m.translation.z ) * 

        Eigen::Quaterniond(     m.rotation.w, 
                                m.rotation.x,
                                m.rotation.y,
                                m.rotation.z  );

    return;
}

/// @brief Constructor
TrajectoryLibraryWrapper::TrajectoryLibraryWrapper()
    :   m_nh{"~"},
        m_tfListener{m_tfBuffer},
        m_hasOdom{false},
        m_hasCostmap{false},
        m_rate{10.0}
{
    std::cout << "[TrajectoryLibraryWrapper] Beginning Initialization" << std::endl;

    //// ROS PARAM Server - Topics, TFs, Etc
    std::string odometry_topic, costmap_topic;
    std::string trajectory_library_filepath;
    std::string trajectory_topic, best_trajectory_topic;
    std::string goal_topic;

    m_nh.param<std::string>("odometry_topic", odometry_topic, "/cmu_rc2/integrated_to_init");
    m_nh.param<std::string>("costmap_topic", costmap_topic, "/cmu_rc2/costmap");
    m_nh.param<std::string>("trajectory_library_filepath", trajectory_library_filepath, "");
    m_nh.param<std::string>("vehicle_frame", m_vehicleFrame, "cmu_rc2_sensor");
    m_nh.param<std::string>("base_frame", m_baseFrame, "cmu_rc2_sensor_init");
    m_nh.param<std::string>("trajectory_topic", trajectory_topic, "/TrajectoryLibrary/trajectories");
    m_nh.param<std::string>("best_trajectory_topic", best_trajectory_topic, "/TrajectoryLibrary/best_trajectory");
    m_nh.param<std::string>("goal_topic", goal_topic, "/TrajectoryLibrary/goal");

    //// ROS PARAM Server - Tuning, Map Values, Etc
    double min_x, min_y, resolution;
    int width, height;

    m_nh.param<double>("min_x", min_x, -50.0);
    m_nh.param<double>("min_y", min_y, -50.0);
    m_nh.param<double>("resolution", resolution, 0.1);
    m_nh.param<int>("width", width, 1000);
    m_nh.param<int>("height", height, 1000);

    m_nh.param<double>("rate", m_rate, 10.0);

    // Setup ROS Subscribers
    m_odometry_sub = m_nh.subscribe(odometry_topic, 1, &TrajectoryLibraryWrapper::odometryCallback, this);
    m_costmap_sub = m_nh.subscribe(costmap_topic, 1, &TrajectoryLibraryWrapper::costmapCallback, this);

    // Setup ROS Publishers
    m_trajectoryPub = m_nh.advertise<visualization_msgs::MarkerArray>(trajectory_topic, 1);
    m_bestTrajectoryPub = m_nh.advertise<visualization_msgs::Marker>(best_trajectory_topic, 1);
    m_goalPub = m_nh.advertise<visualization_msgs::MarkerArray>(goal_topic, 1);

    // Instantiate TrajectoryLibraryManager
    // Load Configuration / Trajectory Files
    p_tlm = std::make_unique<TrajectoryLibraryManager>(min_x, min_y, resolution, width, height);

    if (!p_tlm->configure(trajectory_library_filepath))
    {
        std::cout << "[TrajectoryLibraryWrapper] Failed To Configure PLY File" << std::endl;
        exit(1);
    }

    std::cout << "[TrajectoryLibraryWrapper] Ended Initialization" << std::endl;
}


/// @brief Sequentially loop through process
// First update callbacks, Second update member variables,
// Third verify goal waypoint, Fourth calculate trajectories,
// Fifth publish debug and selected trajectory
// Note : Might make this multi-threaded later once more calculations
// are happening. For now it isn't that much so sequential is fine.
void TrajectoryLibraryWrapper::Loop(){

    std::cout << "[TrajectoryLibraryWrapper] Entering Loop" << std::endl;

    ros::Rate rate(m_rate);

    while (ros::ok())
    {
        // Sleep
        rate.sleep();

        // Update ROS Callbacks
        ros::spinOnce();

        // Verify Costmap / Odometry Received
        if (!m_hasCostmap || !m_hasOdom) {
            std::cout << "[TrajectoryLibraryWrapper] Costmap And/Or Odometry Not Received .. Continuing " 
                            << m_hasCostmap << " " << m_hasOdom << std::endl;
            continue;
        }

        // Retrieve Transform Between Vehicle and Sensor Init
        // Convert From TransformStamped To Eigen
        geometry_msgs::TransformStamped vehicleToBaseTf;
        if (m_tfBuffer.canTransform(m_vehicleFrame, m_baseFrame, ros::Time(0)) ){
            vehicleToBaseTf = m_tfBuffer.lookupTransform(m_baseFrame, m_vehicleFrame, ros::Time(0));
        } else {
            std::cout << "[TrajectoryLibraryWrapper] Cannot Get Transform Between Frames. Continuing" << std::endl;
            continue;
        }
        Eigen::Affine3d eigenMatrix;
        transformMsgToEigen(vehicleToBaseTf.transform, eigenMatrix);

        // Update Costmap and Other Info Given New Callbacks
        fromOccupancyGrid(m_occupancyGrid, p_tlm->m_costmap);

        // Evaluate Trajectories
        p_tlm->setPosition(m_odometry.pose.pose.position.x, m_odometry.pose.pose.position.y);
        if (!p_tlm->processTrajectories(eigenMatrix))
        {
            std::cout << "[TrajectoryLibraryWrapper] Unable To Process Trajectories" << std::endl;
            continue;
        }

        // Publish Selected Trajectory and Debug
        m_trajectoryPub.publish(  toRviz(vehicleToBaseTf)  );

        Trajectory bestTraj = p_tlm->getBestTrajectory();
        m_bestTrajectoryPub.publish(    toRviz(vehicleToBaseTf, bestTraj)); // convert to nav msgs path so controls can respond

        Waypoint currentGoal = p_tlm->getCurrentGoal();
        m_goalPub.publish(  toRviz( currentGoal ));
    }
}

visualization_msgs::MarkerArray TrajectoryLibraryWrapper::toRviz(Waypoint & currentGoal){

    visualization_msgs::MarkerArray array;

    visualization_msgs::Marker m;

    m.header.frame_id = m_baseFrame;
    m.header.stamp = ros::Time::now();
    m.ns = "goals";
    m.id = 0;
    m.type = 2; // Sphere
    m.action = 0;
    m.scale.x = 1.0;    m.scale.y = 1.0;    m.scale.z = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.pose.position.x = currentGoal.x;
    m.pose.position.y = currentGoal.y;
    m.pose.position.z = 0.0;

    array.markers.push_back(m);

    visualization_msgs::Marker m2;

    m2.header.frame_id = m_baseFrame;
    m2.header.stamp = ros::Time::now();
    m2.ns = "ring";
    m2.id = 1;
    m2.type = 2;// sphere
    m2.action = 0;
    m2.scale.x = 10.0;    m2.scale.y = 10.0;    m2.scale.z = 10.0;
    m2.color.r = 0.8;
    m2.color.g = 0.6;
    m2.color.b = 0.8;
    m2.color.a = 0.3;
    m2.pose.position.x = currentGoal.x;
    m2.pose.position.y = currentGoal.y;
    m2.pose.position.z = 0.0;

    array.markers.push_back(m2);

    return array;
}

visualization_msgs::Marker TrajectoryLibraryWrapper::toRviz(geometry_msgs::TransformStamped& tran, Trajectory & traj)
{
    visualization_msgs::Marker m;

    m.header.frame_id = m_baseFrame;
    m.header.stamp = ros::Time::now();
    m.ns = "trajectories";
    m.id = traj.path_id;
    m.type = 4; // Line Strip
    m.action = 0; // Add Modify
    m.scale.x = 0.05;   m.scale.y = 0.05;   m.scale.z = 0.05;

    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    if (traj.score == 0){
        m.color.r = 0.0;
        m.color.g = 1.0;
    } else if (traj.score < 1000) {
        m.color.r = 0.0;
        m.color.b = 1.0;
    }

    m.points.clear();
    m.colors.clear();
    for ( auto state : traj.outputStates ){
        geometry_msgs::Point p;
        p.x = state.x; p.y = state.y; p.z = state.z;
        m.points.push_back(p);
    }

    return m;
}

visualization_msgs::MarkerArray TrajectoryLibraryWrapper::toRviz(geometry_msgs::TransformStamped& tran)
{
    visualization_msgs::MarkerArray array;

    for (auto & [key, traj] : p_tlm->m_trajectories){
        
        visualization_msgs::Marker m = toRviz(tran, traj);

        array.markers.push_back(m);
    }

    return array;
}


void TrajectoryLibraryWrapper::odometryCallback(const nav_msgs::Odometry& msg){
    
    m_odometry = msg;
    m_hasOdom = true;

    return;
} 



void TrajectoryLibraryWrapper::costmapCallback(const nav_msgs::OccupancyGrid& msg) {

    m_occupancyGrid = msg;
    m_hasCostmap = true;

    return;
}

// TODO - Add logic to receive RCDF type data / waypoint string from a high level manager
// This manager should basically take in different types and then create pose array or something to be received by planner


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_library_node");

    // Add any kind of try / exceptions in here???

    TrajectoryLibraryWrapper tlw;
    tlw.Loop();

    return 0;
}