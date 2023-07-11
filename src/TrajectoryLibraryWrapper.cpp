#include "TrajectoryLibraryWrapper.h"

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

bool fromOccupancyGrid( const nav_msgs::OccupancyGrid msg, std::shared_ptr<Costmap> & costmap ) {
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

    m_nh.param<std::string>("odometry_topic", odometry_topic, "/cmu_rc2/integrated_to_init");
    m_nh.param<std::string>("costmap_topic", costmap_topic, "/cmu_rc2/costmap");
    m_nh.param<std::string>("trajectory_library_filepath", trajectory_library_filepath, "");
    m_nh.param<std::string>("vehicle_frame", m_vehicleFrame, "cmu_rc2_sensor");
    m_nh.param<std::string>("base_frame", m_baseFrame, "cmu_rc2_sensor_init");


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
TrajectoryLibraryWrapper::Loop(){

    std::cout << "[TrajectoryLibraryWrapper] Entering Loop" << std::endl;

    ros::Rate rate(20);

    while (ros::ok())
    {
        // Update ROS Callbacks
        ros::spinOnce();

        // Update Costmap and Other Info Given New Callbacks
        fromOccupancyGrid(m_occupancyGrid, p_tlm->m_costmap);

        // Check Waypoint Update From GoalManager
        // p_gm->

        // Calculate Trajectories
        geometry_msgs::TransformStamped vehicleToBaseTf = m_tfBuffer.lookupTransform(m_baseFrame, m_vehicleFrame, ros::Time(0));
        if (!p_tlm->processTrajectories(vehicleToBaseTf))
        {
            std::cout << "[TrajectoryLibraryWrapper] Unable To Process Trajectories" << std::endl;
        }

        // Publish Selected Trajectory and Debug
        // p_tlm->

        // Sleep
        rate.sleep();
    }
}




void TrajectoryLibraryWrapper::odometryCallback(const nav_msgs::Odometry msg){
    
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