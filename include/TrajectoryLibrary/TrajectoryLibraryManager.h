#ifndef TRAJECTORY_LIBRARY_MANAGER_H
#define TRAJECTORY_LIBRARY_MANAGER_H

#include <thread>
#include <condition_variable>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> 
#include <deque>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#include "TrajectoryLibrary/Costmap.h"
#include "TrajectoryLibrary/Trajectory.h"
#include "TrajectoryLibrary/CostFunction.h"

// Comparator Function For MinHeap
struct greater1{
    bool operator()(const Trajectory& a,const Trajectory& b) const{
        return a.score>b.score;
    }
};

class TrajectoryLibraryManager {
    public:

        /// @brief Constructor
        TrajectoryLibraryManager(double min_x, double min_y, double resolution, int width, int height);

        /// @brief Destructor
        ~TrajectoryLibraryManager() = default;

        /// @brief Load Trajectory Library File
        bool configure(std::string filepath); // give filepath???

        /// @brief Set Position
        /// @param x
        /// @param y
        void setPosition( const double x, const double y);

        /// @brief  Set Goal
        /// @param x 
        /// @param y 
        void setGoal( const double x, const double y);

        bool processTrajectories( const Eigen::Affine3d& tran );
        
        Trajectory getBestTrajectory();
        
        Waypoint getCurrentGoal() const;

        std::shared_ptr<Costmap> m_costmap;     // Should these be private and then have getters -- just return pointer???
        std::unordered_map<int, Trajectory> m_trajectories;     // std::vector<Trajectory> m_trajectories;

    private:

        Position m_pos;
        std::shared_ptr<GoalManager> p_gm;
        std::unique_ptr<CostFunction> p_cf;

        Trajectory m_bestTraj;
};

#endif



