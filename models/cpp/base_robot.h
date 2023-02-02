#ifndef BASE_ROBOT_
#define BASE_ROBOT_

#include <Eigen/SparseCore>
#include <Eigen/Dense>

namespace OFNC{

using StateVector   = Eigen::VectorXd;

struct Linear{
    double x;
    double y;
    double z;
};

struct Angular{
    double x;   // roll-phi
    double y; // pitch-theta
    double z;   // yaw-psi
};

struct TwistInput{
    Linear linear;
    Angular angular;
};

struct Position{
    double x;
    double y;
    double z;

    Eigen::Quaternionf quat(1,0,0,0);
    Eigen::Vector3d toEuler()  { return quat.matrix().eulerAngels(0,1,2); }
    Eigen::Matrix3d toMatrix() { return quat.matrix(); }
    
    void setOrientation(const double &roll, const double &pitch, const double &yaw) { 
        Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(roll),Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(pitch),Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(yaw),Vector3d::UnitZ()));
        quat = yawAngle * pitchAngle * rollAngle;
    }
};

class BaseRobot{
public:
    using Ptr = std::shared_ptr<BaseRobot>;
    
    BaseRobot() = default;

    void getPoseSE2FromState(const Eigen::Ref<const Eigen::VectorXd>& x, double& pos_x, double& pos_y, double& theta) const
    {
        assert(x.size() == getStateDimension());
        pos_x = x[0];
        pos_y = x[1];
        theta = x[2];
    }

    void setPosition(const vector<float>& position, double yaw){
        p_.x = position[0];
        p_.y = position[1];
        if(position.size() > 2)
            p_.z = positoin[2];
        p_.setOrientation(0, 0, yaw);
    }

private:
    Position p_;
};

}

#endif