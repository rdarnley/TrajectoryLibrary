#include "TrajectoryLibrary/TrajectoryLibraryManager.h"

TrajectoryLibraryManager::TrajectoryLibraryManager( double min_x, double min_y, 
                                                    double resolution, int width, 
                                                    int height )
{
    m_costmap = std::make_shared<Costmap>(min_x, min_y, resolution, width, height);
    p_gm = std::make_shared<GoalManager>();

    p_cf = std::make_unique<CostFunction>(m_costmap, p_gm);

    m_pos = Position(0.0, 0.0);

    m_bestTraj = Trajectory();
}


void TrajectoryLibraryManager::setPosition( const double x, const double y ){
    m_pos.x = x;
    m_pos.y = y;
    return;
}


void TrajectoryLibraryManager::setGoal( const double x, const double y ) {

    p_gm->setGoal(x, y);
    return;
}


bool TrajectoryLibraryManager::processTrajectories( const Eigen::Affine3d & tran )
{
    if (p_gm->hasGoal()){
        
        // Check Waypoint Update From GoalManager
        if (p_gm->checkGoalStatus(m_pos)){
            std::cout << "Satisfied Previous Goal. Waiting" << std::endl;
            return false;
        }

        // Score Trajectories With CostFunction
        // Could ultimately get sent to gpu -- this would be device function -- processTrajectories would be kernel calling fx
        p_cf->calculateScores(m_trajectories, tran, m_bestTraj);
    
    } else {
        std::cout << "[TrajectoryLibraryManager] No Goal Set" << std::endl;
        return false;
    }

    return true;
}

Waypoint TrajectoryLibraryManager::getCurrentGoal() const
{
    return p_gm->getCurrentGoal();
}

Trajectory TrajectoryLibraryManager::getBestTrajectory()
{
    std::vector<Trajectory> trajCopy;
    for (auto elem : m_trajectories){
        trajCopy.push_back(elem.second);
    }

    std::make_heap(trajCopy.begin(), trajCopy.end(), greater1());

    m_bestTraj = trajCopy.front();

    return m_bestTraj;
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
    int pointSkipNum = 50;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID, groupID;

    int pathNum = 612;  //fix this

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