#ifndef GOAL_MANAGER_H
#define GOAL_MANAGER_H

#include <queue>
#include <cmath>
#include <iostream>

/// @brief Position Struct
/// @details Struct to hold x and y position
/// @param x x position
/// @param y y position
/// @return Position Struct
struct Position{
    double x;
    double y;

    Position(){}
    Position(double x1, double y1){
        x = x1;
        y = y1;
    }
};



struct Waypoint{
    double x;
    double y;
    double threshold = 5.0;

    Waypoint(){};

    Waypoint(double x1, double y1){
        x = x1;
        y = y1;
    }

    Waypoint(double x1, double y1, double thres){
        x = x1;
        y = y1;
        threshold = thres;
    }

    double distance(double x2, double y2){
        return sqrt(pow(x2 - x, 2) + pow(y2 - y, 2));
    }

    bool isSatisfied(double x2, double y2){
        return distance(x2, y2) < threshold;
    }
};


class GoalManager {
    public:

        GoalManager() = default;
        ~GoalManager() = default;

        double getEuclideanDistance(double x, double y);

        bool checkGoalStatus(Position pos);

        const bool inline hasGoal() const { return m_hasGoal; }
        
        const Waypoint inline getCurrentGoal() const { return m_currentGoal; };

        void setGoal(double x, double y) {
            m_currentGoal = Waypoint(x, y);
            m_hasGoal = true;
        };

    private:

        Waypoint m_currentGoal;
        bool m_hasGoal;

};

#endif



