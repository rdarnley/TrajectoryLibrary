#ifndef GOAL_MANAGER_H
#define GOAL_MANAGER_H

#include <queue>
#include <cmath>
#include <iostream>

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

        GoalManager();
        // ~GoalManager();

        double getEuclideanDistance(double x, double y);

        bool checkGoalStatus(Position pos);
        bool updateGoal();

        bool hasGoal();
        Waypoint getCurrentGoal();

        void setGoal(double x, double y);

    private:

        std::queue<Waypoint> wpQueue;

};

#endif



