#include "TrajectoryLibrary/TrajectoryLibraryWrapper.h"

//// Utility Functions
// nav_msgs::OccupancyGrid to_OccupancyGrid( void )const {
//   nav_msgs::OccupancyGrid map_msg;
  
//   map_msg.info.resolution = resolution;
//   map_msg.info.width = width;
//   map_msg.info.height = height;
//   map_msg.info.origin.position.x = min_x;
//   map_msg.info.origin.position.y = min_y;
//   map_msg.data.clear();
  
//   for ( const uint8_t & cell : data ) {
//     map_msg.data.push_back( (char) cell );
//   }
  
//   return map_msg;
// }

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

/// @brief Constructor
TrajectoryLibraryWrapper::TrajectoryLibraryWrapper()
    :   m_nh{"~"},
        m_tfListener{m_tfBuffer},
        m_hasOdom{false},
        m_hasCostmap{false}
{
    std::cout << "[TrajectoryLibraryWrapper] Beginning Initialization" << std::endl;

    //// ROS PARAM Server - Topics, TFs, Etc
    std::string odometry_topic, costmap_topic;
    std::string trajectory_library_filepath;
    std::string trajectory_topic;

    m_nh.param<std::string>("odometry_topic", odometry_topic, "/cmu_rc2/integrated_to_init");
    m_nh.param<std::string>("costmap_topic", costmap_topic, "/cmu_rc2/costmap");
    m_nh.param<std::string>("trajectory_library_filepath", trajectory_library_filepath, "");
    m_nh.param<std::string>("vehicle_frame", m_vehicleFrame, "cmu_rc2_sensor");
    m_nh.param<std::string>("base_frame", m_baseFrame, "cmu_rc2_sensor_init");
    m_nh.param<std::string>("trajectory_topic", trajectory_topic, "/TrajectoryLibrary/trajectories");


    //// ROS PARAM Server - Tuning, Map Values, Etc
    double min_x, min_y, resolution;
    int width, height;

    m_nh.param<double>("min_x", min_x, -50.0);
    m_nh.param<double>("min_y", min_y, -50.0);
    m_nh.param<double>("resolution", resolution, 0.1);
    m_nh.param<int>("width", width, 1000);
    m_nh.param<int>("height", height, 1000);

    // Setup ROS Subscribers
    m_odometry_sub = m_nh.subscribe(odometry_topic, 1, &TrajectoryLibraryWrapper::odometryCallback, this);
    m_costmap_sub = m_nh.subscribe(costmap_topic, 1, &TrajectoryLibraryWrapper::costmapCallback, this);

    // Setup ROS Publishers
    m_trajectoryPub = m_nh.advertise<visualization_msgs::MarkerArray>(trajectory_topic, 1);

    // Instantiate TrajectoryLibraryManager
    p_tlm = std::make_unique<TrajectoryLibraryManager>(min_x, min_y, resolution, width, height);

    // Configure TrajectoryLibraryManager With Correct Library Files
    if (!p_tlm->configure(trajectory_library_filepath))
    {
        std::cout << "[TrajectoryLibraryWrapper] Failed To Configure PLY File" << std::endl;
        exit(1);
    }

    // // Instantiate GoalManager
    // p_gm = std::make_unique<GoalManager>();

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

    ros::Rate rate(10);

    while (ros::ok())
    {
        // Sleep
        rate.sleep();

        // Update ROS Callbacks
        ros::spinOnce();

        // Update Costmap and Other Info Given New Callbacks
        fromOccupancyGrid(m_occupancyGrid, p_tlm->m_costmap);

        // Check Waypoint Update From GoalManager
        // p_gm->

        // Calculate Trajectories
        geometry_msgs::TransformStamped vehicleToBaseTf;
        if (m_tfBuffer.canTransform(m_vehicleFrame, m_baseFrame, ros::Time(0)) ){
            vehicleToBaseTf = m_tfBuffer.lookupTransform(m_baseFrame, m_vehicleFrame, ros::Time(0));
        } else {
            std::cout << "[TrajectoryLibraryWrapper] Cannot Get Transform Between Frames. Continuing" << std::endl;
            continue;
        }

        if (!p_tlm->processTrajectories(vehicleToBaseTf))
        {
            std::cout << "[TrajectoryLibraryWrapper] Unable To Process Trajectories" << std::endl;
        }

        // Publish Selected Trajectory and Debug
        m_trajectoryPub.publish(  toRviz(vehicleToBaseTf)  );

    }
}

visualization_msgs::MarkerArray TrajectoryLibraryWrapper::toRviz(geometry_msgs::TransformStamped& tran)
{
    visualization_msgs::MarkerArray array;

    for (auto & [path, traj] : p_tlm->m_trajectories){
        
        visualization_msgs::Marker m;

        m.header.frame_id = m_baseFrame;
        m.header.stamp = ros::Time::now();
        m.ns = "trajectories";
        m.id = traj.path_id;
        m.type = 4; // Line Strip
        m.action = 0; // Add / Modify
        m.scale.x = 0.05;    m.scale.y = 0.05;    m.scale.z = 0.05;

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




int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_library_node");

    // Add any kind of try / exceptions in here???

    TrajectoryLibraryWrapper tlw;
    tlw.Loop();

    return 0;
}