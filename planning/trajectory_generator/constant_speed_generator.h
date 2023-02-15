#ifndef CONSTANT_SPEED_GENERATOR_H
#define CONSTANT_SPEED_GENERATOR_H

#include <types.h>
#include <tinyspline/tinysplinecxx.h>


class ConstanctSpeedTrajectoryGenerator {
 public:
  ConstanctSpeedTrajectoryGenerator(float ref_v){
    local_index_= 0;
    ref_v_ = ref_v;
  }

  ~ConstanctSpeedTrajectoryGenerator() = default;

  void NearestPoint(const TrajPoint& pos, GlobalPath global_path){
    float min_dist = 1000000;
    int min_index = 0;
    for(int i = 0; i < global_path.GetLength(); i++){
        float dist = sqrt(pow(pos.x_ - global_path.path_[i].x_, 2) + pow(pos.y_ - global_path.path_[i].y_, 2));
        if(dist < min_dist){
            min_dist = dist;
            min_index = i;
        }
    }
    local_index_ = min_index;
  }

  LocalTraj GetTraj(){
    return local_traj_;
  }
  
  void SplineCurve(const TrajPoint& pos, const GlobalPath& global_path, const int& points_num, bool use_natural=false){
    std::vector<float> x;
    std::vector<float> y;
    int path_length = global_path.GetLength();

    for(int i = 0; i < path_length; i++){
        x.push_back(global_path.path_[local_index_ + i].x_);
        y.push_back(global_path.path_[local_index_ + i].y_);
    }

    std::vector<tinyspline::real> ctrlp;

    for(int i = 0; i < path_length; i++){
        ctrlp.push_back((tsReal)global_path.path_[i].x_);
        ctrlp.push_back((tsReal)global_path.path_[i].y_);
    }   

    tinyspline::std_real_vector_out points;
    tinyspline::BSpline spline;

    if(use_natural){
        spline = tinyspline::BSpline::
		        interpolateCubicNatural(ctrlp, 2);
        points = spline.sample(points_num);
    }else{
        spline(path_length);
        spline.setControlPoints(ctrlp);
        points = spline.sample(points_num);
    }
    
    // for(int i = 0; i < points.size(); i+=2){
    //     TrajPoint temp_point;
    //     temp_point.x_ = points[i];
    //     temp_point.y_ = points[i+1];
    //     local_traj_.traj_.push_back(temp_point);
    // }

    for(int i = 0; i <= points_num*2; i+=2){
        tsReal eval_pos = (tsReal)i/points_num/2;
        auto pos = spline.eval(eval_pos).resultVec2();
        auto der_1 = spline.derive().eval(eval_pos).result();
        auto der_2 = spline.derive(2).eval(eval_pos).result();
        // std::cout << "1 der " << der_1[0] << " " << der_1[1] << std::endl;
        // std::cout << der_2[0] << " " << der_2[1] << std::endl;

        TrajPoint temp_point;
        temp_point.x_ = pos.x();
        temp_point.y_ = pos.y();
        temp_point.yaw_ = atan2(der_1[1], der_1[0]);
        temp_point.curvature_ = (der_1[0] * der_2[1] - der_1[1] * der_2[0]) / pow(der_1[0] * der_1[0] + der_1[1] * der_1[1], 1.5);
        temp_point.vel_ = ref_v_;
       
        local_traj_.traj_.push_back(temp_point);
    }
  }

  void GenerateTraj(const TrajPoint& pos, GlobalPath global_path, const int& points_num, bool use_natural=false){
    NearestPoint(pos, global_path);
    SplineCurve(pos, global_path, points_num, use_natural);
  }

 private:
  LocalTraj local_traj_;
  int local_index_;
  float ref_v_;

};

#endif
