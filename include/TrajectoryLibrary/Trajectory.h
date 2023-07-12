#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <thread>
#include <condition_variable>
#include <vector>

struct State {
    double x;
    double y;
    double z;
    double yaw;

    State(){}

    State(const double xArg, const double yArg, const double zArg, const double yawArg)
    {
        x = xArg;
        y = yArg;
        z = zArg;
        yaw = yawArg;
    }

};

class Trajectory {
    public:

        /// @brief Constructor
        Trajectory( );
        
        // /// @brief Destructor
        // ~Trajectory();

        std::vector<State> states;
        std::vector<State> outputStates;
        int group_id;
        int path_id;
        int score;

    private:


};

#endif



