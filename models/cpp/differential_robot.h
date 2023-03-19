/*
author : Kai CHEN
mail : chenkai0130 @outlook.com
license : MIT
*/
#ifndef DIFFERENTIAL_ROBOT_H
#define DIFFERENTIAL_ROBOT_H

#include <models/cpp/base_robot.h>
#include <cmath>

namespace OFNC
{

    class DifferentialRobot : public BaseRobot
    {
    public:
        DifferentialRobot() = default;

        int getStateDimention() { return 3; }

        int getInputDimention() { return 2; }

        void dynamics(const Eigen::Ref<const StateVector> &x, const Eigen::Ref<const ControlVector> &u, Eigen::Ref<StateVector> f)
        {
            assert(x.size() == getStateDimension());
            assert(u.size() == getInputDimension());
            assert(x.size() == f.size() && "UnicycleModel::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");

            f[0] = u[0] * std::cos(x[2]);
            f[1] = u[0] * std::sin(x[2]);
            f[2] = u[1];
        }

        bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd> &u, TwistInput &twist) const override
        {
            assert(u.size() == getInputDimension());
            twist.linear.x = u[0];
            twist.linear.y = twist.linear.z = 0;

            twist.angular.z = u[1];
            twist.angular.x = twist.angular.y = 0;
            return true;
        }
    }
};

#endif