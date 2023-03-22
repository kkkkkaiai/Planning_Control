#ifndef NONLINEAR_MODEL_PREDICTIVE_CONTROL_H_
#define NONLINEAR_MODEL_PREDICTIVE_CONTROL_H_

#include <control/cpp/optimal_control_problem.h>
#include <casadi/casadi.h>
#include <string>
#include <vector>

namespace OFNC{
struct OCPParams{
    std::string solver;
    int horizon;

    // Number of dimentions of the StateVector
    int nx; 
    // Number of dimentions of the InputVector
    int nu; 

    // Discretization time
    double dt; 
    // Terminal state
    casadi::Mx x_e; 
    // Weighting matrix for the control trajectory
    casadi::MX R; 
    // Weighting matrix for the state trajectory
    casadi::MX Q;
    // Weighting matrix for the terminal state
    casadi::MX P;
    // Indices of the required state constraints
    std::vector<int> x_const_index;
    // Indices of the required control constraints
    std::vector<int> u_const_index;
};

class OptimalControlProblem{
public:
    OptimalControlProblem() = default;

    inline casadi::DM solver(){
        const casadi::Optisol sol = nlp_.solve();
        X_sol_ = sol.value(X_);
        U_sol_ = sol.value(U_);
        return U_sol_;
    }

    inline void Init(const casadi::DM &x_0){
        nlp_.set_value(, ocp_params_.sc_x * x_0);
        nlp_.set_initial(X_, X_sol_);
        nlp_.set_initial(U_, U_sol_);
    }

    virtual void BuildOCP() = 0;

    virtual void ReadParams(const std::string& config_file) = 0;

private:
    OCPParams ocp_params_;
    
    // Constructed NLP using CasADi
    casadi::Opti nlp_;
    // Solution trajectory of the state vector, which includes the scaling factors
    casadi::DM X_sol_;
    // Solution trajectory of the control vector, which includes the scaling factors
    casadi::DM U_sol_;
    // Cost functional
    casadi::MX J_;
    // Discretized state (NLP state parameters)
    casadi::MX X_;
    // Discretized control (NLP control parameters)
    casadi::MX U_;
    // Initial state variable
    casadi::MX X_0_;
};

}

#endif