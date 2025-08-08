# JetBot MPC Controllers and ROS Implementation

## Project Overview

This project develops advanced **Model Predictive Control (MPC)** strategies for the JetBot robot, featuring two main parts:

### Part I – MATLAB MPC Controllers
Development of various MPC controllers in MATLAB for different navigation tasks:

1. **controllerSimple**  
   Drives a single JetBot from an initial position to a fixed target position.

2. **controllerTracking**  
   Makes a JetBot follow a predefined trajectory.

3. **controllerMovie**  
   Coordinates two robots to orbit a moving point from the right side, simulating a filming scenario.

4. **controllerCooperative**  
   Coordinates two robots, starting from arbitrary positions, to intercept and cover as much as possible of a third robot’s path.

Associated support scripts perform interception point calculations, reference selection, and waypoint interpolation. All controllers consider realistic constraints such as velocity and acceleration.

### Part II – ROS Implementation with CasADi
Implementation of the `controllerSimple` MPC in ROS using the CasADi optimization library. Features include:

- Integration with ROS for odometry and velocity command topics.  
- Simulation using Gazebo and RViz.  
- Real-time control to drive JetBot to a target position.

This ROS implementation builds upon and extends the base package [cybaer-nova/jetbot_ros](https://github.com/cybaer-nova/jetbot_ros).

Technical report available in the `docs/` folder.

---

## Project Structure

