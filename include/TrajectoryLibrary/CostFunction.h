#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include "TrajectoryLibrary/Trajectory.h"
#include "TrajectoryLibrary/Costmap.h"
#include "TrajectoryLibrary/GoalManager.h"

#include <Eigen/Dense>
#include <unordered_map>

class CostFunction {
    public:

        CostFunction(   std::shared_ptr<Costmap>& costmap,
                        std::shared_ptr<GoalManager>& gm    );
        ~CostFunction() = default;

        void calculateScores(   std::unordered_map<int, Trajectory> & trajectories, 
                                const Eigen::Affine3d & tran,  
                                const Trajectory & lastTraj );

    private:

        std::shared_ptr<Costmap> p_costmap;
        std::shared_ptr<GoalManager> p_gm;

        double m_costmapWeight;
        double m_heuristicWeight;
        double m_persistenceWeight;
};

#endif



