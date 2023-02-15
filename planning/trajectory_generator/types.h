#ifndef TYPES_H 
#define TYPES_H

#include <vector>
#include <cmath>
#include <iostream>
#include <cassert>

class PathPoint{
public:
    PathPoint(const float& x, const float& y){
        x_ = x;
        y_ = y;
    }

    float x_;
    float y_;
};

class TrajPoint: public PathPoint{
public:
    TrajPoint(const float& x, const float& y, const float& yaw, const float& vel): PathPoint(x, y){
        yaw_ = yaw;
        vel_ = vel;
    }

    TrajPoint(const float& x, const float& y): PathPoint(x, y){
        yaw_ = 0;
        vel_ = 0;
    }

    TrajPoint(): PathPoint(0, 0){
        yaw_ = 0;
        vel_ = 0;
    }

    float yaw_;
    float vel_;
    float curvature_;
};

class GlobalPath{
public:
    GlobalPath(const std::vector<float>& path){
        assert(path.size() % 2 == 0);
        for(int i = 0; i < path.size(); i+=2){
            path_.push_back(PathPoint(path[i], path[i+1]));
        }
    }

    void AddPoint(const float& x, const float& y){
        path_.push_back(PathPoint(x, y));
    }

    void DeletePoint(const int& index){
        path_.erase(path_.begin() + index);
    }

    int GetLength() const {
        return path_.size();
    }

    std::vector<PathPoint> path_;
};

class LocalTraj {
public:
    std::vector<TrajPoint> traj_;

    int GetLength() const{
        return traj_.size();
    }

    void Resize(const int& size){
        traj_.resize(size);
    }
};

#endif