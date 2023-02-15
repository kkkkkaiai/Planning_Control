#include "constant_speed_generator.h"
#include "matplotlibcpp.h"
#include <iostream>

using namespace std;
namespace plt = matplotlibcpp;

int main(){
    vector<float> path = {0, 1, 1, 0, 2, 3, 3, 2, 4, 5};
    vector<float> path1 = {1,1, 4,2, 5,3, 3,5, 2,3};
    vector<float> raw_x;
    vector<float> raw_y;

    for(auto i = 0; i < path.size(); i+=2){
        raw_x.push_back(path[i]);
        raw_y.push_back(path[i+1]);
    }

    ConstanctSpeedTrajectoryGenerator cst(10);
    GlobalPath global_path(path);

    TrajPoint pos(0, 0);
    cst.GenerateTraj(pos, global_path, 50, true);
    auto traj = cst.GetTraj();

    auto spline_path = traj.GetLength();
    vector<float> x;
    vector<float> y;

    for(auto &i: traj.traj_){
        x.push_back(i.x_);
        y.push_back(i.y_);
    }

    plt::plot(raw_x, raw_y, "r");
    plt::plot(x, y);
    plt::show();
    return 0;

}