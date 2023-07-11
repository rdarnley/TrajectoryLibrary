#include "TrajectoryLibrary/TrajectoryLibraryManager.h"

TrajectoryLibraryManager::TrajectoryLibraryManager(double min_x, double min_y, double resolution, int width, int height)
{
    m_costmap = std::make_shared<Costmap>(min_x, min_y, resolution, width, height);
}


bool TrajectoryLibraryManager::configure(std::string filepath)
{
    FILE *filePtr = fopen(filepath.c_str(), "r");
    if (filePtr == NULL){
        printf("\nCannot read input files, exiting.\n\n");
        exit(1);
    }

    // Read Header
    char str[50];
    int val, pointNum;
    std::string str_cur, str_last;
    while (str_cur != "end_header"){
        val = fscanf(filePtr, "%s", str);
        if (val != 1){
            std::cout << "Val != 1" << std::endl;
            exit(1);
        }

        str_last = str_cur;
        str_cur = std::string(str);

        if (str_cur == "vertex" && str_last == "element"){
            val = fscanf(filePtr, "%d", &pointNum);
            if (val != 1){
                std::cout << "Val != 1" << std::endl;
                exit(1);
            }
        }
    }

    State st;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID, groupID;

    int pathNum = 343;  //fix this

    for (int i = 0; i < pointNum; ++i){
        val1 = fscanf(filePtr, "%le", &st.x);
        val2 = fscanf(filePtr, "%le", &st.y);
        val3 = fscanf(filePtr, "%le", &st.z);
        val4 = fscanf(filePtr, "%d", &pathID);
        val5 = fscanf(filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        printf ("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (pathID >= 0 && pathID < pathNum){
            pointSkipCount++;
            if (pointSkipCount > pointSkipNum){
                m_trajectories[pathID].states.push_back(st);
                m_trajectories[pathID].group_id = groupID;
                m_trajectories[pathID].path_id = pathID;
            }
        }
    }

    fclose(filePtr);

    return true;
}


bool TrajectoryLibraryManager::processTrajectories(geometry_msgs::TransformStamped & tran)
{
    // Iterate over trajectories
    for (auto [id, traj] : m_trajectories)
    {
        // Score Trajectory
        int cost = 0;

        for (auto state : traj.states)
        {
            // Transform State To Sensor Init Frame
            geometry_msgs::Point p;
            p.x = state.x; p.y = state.y; p.z = state.z;
            tf2::doTransform(p, p, tran);

            // Get Value In Costmap
            cost += m_costmap->get(p.x, p.y);
        }

        traj.score = cost;
    
        std::cout << "Trajectory #" << id << " Scored " << cost << std::endl;
    }
    // add to heap???

    return true;
}


// TrajectoryLibraryManager::getDebug()
// {

// }

// TrajectoryLibraryManager::getBestTrajectory()
// {

// }