#ifndef OPTIMAL_CONTROL_PROBLEM_H_
#define OPTIMAL_CONTROL_PROBLEM_H_

#include <casadi/casadi.h>
#include <control/cpp/optimal_control_problem.h>

namespace OFNC{

class NonlinearModelPredictiveControl: public OptimalControlProblem{
public:
    NonlinearModelPredictiveControl();
};

}

#endif