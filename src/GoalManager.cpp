#include "TrajectoryLibrary/GoalManager.h"

/// Note: Ideally we remove the functionality from the behavior executive to a degree
/// For now we will only do a minimalist GoalManager so there isn't contention between the two


bool GoalManager::checkGoalStatus(Position pos){

    bool update = false;

    // Check If Minimum Distance Threshold Met For Waypoint
    if (m_currentGoal.isSatisfied(pos.x, pos.y)) { 
        update = true; 
    }

    // Check If Wp Is In Obstacle

    // Handle timer if wp still not reached

    return update;
}



double GoalManager::getEuclideanDistance(double x, double y){
    if ( hasGoal() ){                    
        return m_currentGoal.distance(x, y);
    }
    return 0.0;
}


// GoalManager::getRelativeHeading(){}
