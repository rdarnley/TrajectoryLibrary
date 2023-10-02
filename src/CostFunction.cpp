#include "TrajectoryLibrary/CostFunction.h"

CostFunction::CostFunction( std::shared_ptr<Costmap>& costmap,
                            std::shared_ptr<GoalManager>& gm  ) : 
    p_costmap(costmap),
    p_gm(gm)
{
    m_costmapWeight = 1.0;
    m_heuristicWeight = 0.5;
    m_persistenceWeight = 0.0;
}

void CostFunction::calculateScores( std::unordered_map<int, Trajectory> & trajectories, 
                                    const Eigen::Affine3d & tran,
                                    const Trajectory & lastTraj   ){

    // Iterate over trajectories
    for (auto & [path_id, traj] : trajectories)
    {
        // Score Trajectory W.R.T. Costmaps
        int cost = 0;

        traj.outputStates.clear();

        for (auto state : traj.states)
        {
            // Transform State To Sensor Init Frame
            Eigen::Vector3d v;
            v << state.x, state.y, state.z;

            v = tran * v;

            State st;
            st.x = v[0];    st.y = v[1];    st.z = v[2];
            traj.outputStates.push_back(st);

            cost += p_costmap->get(st.x, st.y);
        }

        // Get Heuristic W.R.T. Waypoint
        double dist = p_gm->getEuclideanDistance(traj.outputStates.back().x, traj.outputStates.back().y);

        // Incorporate Persistence W.R.T Old Trajectory
        uint8_t idDelta = abs(lastTraj.path_id - path_id);

        // Calculate Total Cost/Score
        traj.score = m_costmapWeight * cost + m_heuristicWeight * dist + m_persistenceWeight * idDelta;    
    }
}

// should probably get something like this looking really good in sim (don't need to handle continuity / tracking component yet)
// tune with perception. obviously naive and overfit but make this work really well. then can add complexity to cost function,
// worry about continuity/tracking, rates, etc

// Also need to create speed profiles