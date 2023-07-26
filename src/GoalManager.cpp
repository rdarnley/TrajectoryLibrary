#include "TrajectoryLibrary/GoalManager.h"

GoalManager::GoalManager(){
    
    // // Example
    // Waypoint wp(10, 0);
    // wpQueue.push(wp);
}


bool GoalManager::checkGoalStatus(Position pos){

    bool update = false;

    // Check If Minimum Distance Threshold Met For Waypoint
    if (wpQueue.front().isSatisfied(pos.x, pos.y)) { update = true; }

    // Check If Wp Is In Obstacle

    // Handle timer if wp still not reached

    return update;
}

bool GoalManager::updateGoal(){
    wpQueue.pop();

    return true;
}

void GoalManager::setGoal(double x, double y){
    Waypoint wp(x, y);
    wpQueue.push(wp);
}

double GoalManager::getEuclideanDistance(double x, double y){
    if (wpQueue.size() > 0){                    // is this the place to have this logic?
        return wpQueue.front().distance(x, y);
    }
    return 0.0;
}

bool GoalManager::hasGoal(){
    return wpQueue.size() > 0;
}

// GoalManager::getRelativeHeading(){}


Waypoint GoalManager::getCurrentGoal(){
    return wpQueue.front();
}

// GoalManager::setGoal(){}