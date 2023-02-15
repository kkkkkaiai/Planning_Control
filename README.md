# Obstacle Formulation and Nonlinear Control
Perception, Planner and Controller

# Prerequest

```bash
# Install matplotlib-cpp
git clone https://github.com/lava/matplotlib-cpp.git
cd matplotlib-cpp
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

# Test
```bash
mkdir build && cd build
cmake ..
make -j4
```
## B-Spline Curve
```bash
./constant_speed_test
```

# Reference
## Path Planning and Trajectory Optimization
https://github.com/libai1943/CartesianPlanner  
https://github.com/icsl-Jeon/traj_gen

## Motion Controller
### General
https://github.com/Shunichi09/PythonLinearNonlinearControl  
https://github.com/tud-amr/localPlannerBench 
### MPC
https://github.com/kohonda/mpc_tracker_ros  
https://github.com/tomcattiger1230/CasADi_MPC_MHE_CPP  
https://github.com/alexliniger/MPCC  
https://github.com/tud-amr/amr-lmpcc  
### CBF
https://github.com/HybridRobotics/cbf  

## Environment Perception
### TSDF, ESDF  
https://github.com/nvidia-isaac/nvblox  
### large convex polytopes  
https://github.com/StarryN/Galaxy  

## Mobile Manipulator
https://github.com/caelan/SS-Replan 

## Multi Robot
https://github.com/LIJUNCHENG001/multi_robot_traj_planner

