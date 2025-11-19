# Professional Work

## Nonlinear Model Predictive Controller (nMPC) for Differential Drive Mobile Robot

<center>
<video width="70%" controls autoplay loop muted>
  <source src="./videos/nMPC.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

---

### Description
A **nonlinear Model Predictive Controller (nMPC)** based local planner developed for a **Differential Drive Mobile Robot (DDMR)** within the **ROS Noetic** navigation stack.  

Unlike conventional reactive planners, the nMPC predicts future robot states through a **kinematic model** and optimizes control inputs over a finite horizon.  
The controller minimizes a cost function while enforcing **hard constraints** on obstacle clearance, velocity, and input bounds.

The implementation leverages:
- **CasADi** for nonlinear optimization formulation  
- **IPOPT** (Interior Point Optimizer) for efficient real-time solving  
- **Python / ROS Noetic** for seamless runtime integration  

The planner was benchmarked against standard local planners:
- **Dynamic Window Approach (DWA)**
- **Timed Elastic Band (TEB)**

Results demonstrate smoother, dynamically feasible trajectories, particularly in cluttered or narrow environments.  
The entire system was simulated in **Gazebo** using a **TurtleBot**, with a GPU-enabled **Docker** container for reproducibility.

---

### Frameworks & Libraries
- ROS Noetic  
- CasADi  
- IPOPT  
- Python  
- Gazebo  
- Docker  

---

### Links
- [GitHub Repository](https://github.com/KevinEppacher/walle_ws.git)
- [Download unpublished Research Paper (PDF)](./papers/nMPC.pdf)
---

## Automated Sensitivity Measurement System (AIRSKIN)

<center>
<video width="70%" controls autoplay loop muted>
  <source src="./videos/sensibility_measurements.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

---

### Description
A **collaborative project with [Blue Danube Robotics – AIRSKIN](https://www.airskin.io/)** developed at **UAS Technikum Vienna** to automate tactile pad sensitivity measurements.  

The system measures the **force and displacement** required to trigger an AIRSKIN pad at defined grid points. From this, the **spring constant** and **local sensitivity** are derived to detect mechanical weak points and support further product development.

Built entirely with **ROS Noetic** and **Docker**, the system integrates:
- A **UR10** industrial robot  
- A **force–torque (FT) sensor** connected via the UR ROS bridge (TCP/IP)  
- A **custom ImGui C++ HMI** for switching between *Freedrive Mode* (teaching) and *External Control Mode* (automated measurement)  

Once all measurement points are defined, **MoveIt** executes a fully automated sequence. The system visualizes force vectors in **RViz** and overlays a 3D point cloud from an integrated RGB-D camera, enabling intuitive analysis of pad deformation and sensitivity.

A full **Gazebo simulation** replicates the entire setup for safe testing and repeatable experiments.

---

### Frameworks & Libraries
- ROS Noetic  
- MoveIt  
- ImGui (C++ GUI)  
- RViz / Gazebo  
- Docker  
- UR ROS Driver / TCP-IP Bridge  

---

### Links
- [GitHub Repository](https://github.com/KevinEppacher/goldilocks_sensibility_ws.git)

---

### Summary
Automated robotic test bench for AIRSKIN pad calibration — measuring and visualizing tactile sensitivity through force–displacement mapping.


---

🚀 Monte Carlo Localization (Particle Filter) for Mobile Robots 🤖🌟
Last year, during my second master's semester, I had the opportunity to work on a project where I implemented a Monte Carlo Localization (MCL) algorithm, also known as a particle filter, for the localization of Differential Drive Mobile Robots (DDMR).

The core idea behind the particle filter is simple yet powerful: It utilizes a predefined map, a motion model for prediction, and weights each particle based on raycasting within a probabilistic measurement model. I developed a ROS Noetic C++ implementation, simulating a Turtlebot navigating through a small apartment environment within Gazebo.

Key Highlights of My Approach:
- Efficient Particle Usage: Achieving reliable localization with only 100 particles, compared to the typical 500 to 3000 particles used by ROS Adaptive Monte Carlo Localization (AMCL) for a small apartment environment.
- Innovative Resampling Strategy: Introducing randomness by generating 80% of the particles every time. This approach greatly improves the algorithm’s ability to quickly determine the initial pose and recover from localization loss.

Check out the full details and code here: https://lnkd.in/dE9Z88Zq

I'd love to hear your thoughts and feedback! 🚀🤖
Hashtag#Robotics Hashtag#Localization Hashtag#MonteCarlo Hashtag#AMCL Hashtag#MachineLearning Hashtag#ROS Hashtag#C++ Hashtag#Gazebo Hashtag#Simulation

Paths:
videos/mcl.gif

---

Bachelor Thesis:

The recent rapid growth in popularity of UAVs (drones) has led to a variety of new applications and opportunities. At the same time, however, serious safety concerns and potential hazards have emerged, affecting both airports and individuals.

My bachelor thesis aims to address this issue. At the Automation and Control Institute (TU), I was privileged to develop a control system for tracking drones at short distances and high speeds using a pan-tilt (PT) camera. 

I developed a cascaded position and velocity controller to control the pan and tilt axis, which are driven by a permanent magnet synchronous motor (PMSM) with a field oriented control (FOC) algorithm. In addition, the drone's trajectory is smoothed, interpolated, and predicted using the Kalman filter (sensor fusion) in case the drone disappears from the field of view or becomes undetectable. The methods were simulated in the Robot Operating System (ROS) and using OpenCV.

In a reproducible experiment, a laser beam with a defined trajectory was specified for the PT camera to track. During the tracking, the laser point detection algorithm is briefly disabled to further estimate the motion of the respective axes.

The results showed that by using the cascaded position and velocity controller, 1 rad/s can be achieved and therefore the drones can track at a distance of 30 m with a velocity of 30 m/s.

Hashtag#UAV Hashtag#DroneTechnology Hashtag#ControlSystems Hashtag#Automation Hashtag#Research Hashtag#Thesis Hashtag#Innovation

Paths:
videos/OptoFence_Video_3.mp4


# Personal Projects

---
In this project, a 6-DOF robotic arm has been designed and built which is driven by stepper motors and various gears such as 3D printed harmonic drives and timing belts. 

The robot arm is controlled via ROS (Robot Operating System) using the MoveIt package. The MoveIt package visualizes and controls the robot arm on RViz, and in the background, it calculates the collision matrix so that the robot arm cannot collide with itself. 

 Through the visualization of RViz, the robot arm is simulated in the program and this simulation is transferred to the real robot .🦾

Next step is, to make it 👀

 Hashtag#Robot Hashtag#ROS Hashtag#3Dprinted Hashtag#PickAndPlace Hashtag#stepper Hashtag#MoveIt Hashtag#gripper Hashtag#innovation Hashtag#fromscr

 Paths:
 img/6_DOFRA_Rendered_1.png
 img/6_DOFRA_Rendered_2.png
 videos/6dofra.mp4