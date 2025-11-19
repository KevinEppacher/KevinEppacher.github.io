# Professional Work

## 1. **Nonlinear Model Predictive Controller (nMPC) for Differential Drive Mobile Robot**

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

## 2. **Automated Sensitivity Measurement System (AIRSKIN)**

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

## Monte Carlo Localization (Particle Filter) for Mobile Robots

<center>
<img src="./videos/mcl.gif" width="70%" alt="Monte Carlo Localization simulation in Gazebo">
</center>

---

### Description
A **Monte Carlo Localization (MCL)** system — also known as a **Particle Filter** — implemented for **Differential Drive Mobile Robots (DDMR)** using **ROS Noetic** and **C++**.  

The algorithm estimates a robot’s pose on a known map by maintaining a set of weighted samples (“particles”), each representing a possible state hypothesis.  
The approach combines:
- A **motion model** for prediction (based on wheel odometry)  
- A **sensor model** using **raycasting** for probabilistic measurement updates  
- A **resampling step** to reinforce high-likelihood particles and discard low-likelihood ones  

---

### Key Highlights
- **Efficient Particle Usage:** Achieved reliable localization with only **100 particles**, compared to typical **500–3000** used by **AMCL**, while maintaining accuracy in a small apartment map.  
- **Innovative Resampling Strategy:** Introduced controlled randomness by regenerating **80% of the particles** each iteration, improving robustness against localization loss and aiding fast global convergence.  
- **Gazebo Simulation:** Implemented and validated using a **TurtleBot** navigating through a custom indoor apartment environment.  

---

### Frameworks & Libraries
- ROS Noetic  
- C++  
- Gazebo  
- RViz  
- Eigen  
- Docker  

---

### Links
- [GitHub Repository](https://github.com/KevinEppacher/Probabilistic_Lab.git)
- [Download Research Paper (PDF)](./papers/efficient_monte_carlo_localization_for_mobile_robots_implementation_and_evaluation_Eppacher.pdf)

---

### Summary
Robust and efficient Monte Carlo Localization achieving high accuracy with minimal particles through adaptive resampling — enabling fast and reliable robot pose estimation in dynamic indoor environments.

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

## 




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