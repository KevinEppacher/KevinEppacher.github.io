# Professional Work

Below is a selection of my research and engineering projects, spanning semantic exploration, optimization-based control, and applied industrial robotics.

## **1. Nonlinear Model Predictive Controller (nMPC) for Differential Drive Mobile Robot**

<center>
<video width="70%" controls autoplay loop muted>
  <source src="./videos/nMPC.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

---

### Description
A **nonlinear Model Predictive Controller (nMPC)** based local planner developed for a **Differential Drive Mobile Robot (DDMR)**.  

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
- Nav2 
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

## **2. Automated Sensitivity Measurement System (AIRSKIN)**

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

## **3. Monte Carlo Localization (Particle Filter) for Mobile Robots**

<center>
<img src="./videos/mcl.gif" width="70%" alt="Monte Carlo Localization simulation in Gazebo">
</center>

---

### Description
A **Monte Carlo Localization (MCL)** system — also known as a **Particle Filter** — implemented in **C++** for **Differential Drive Mobile Robots (DDMR)** using **ROS Noetic**.  

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
- Nav2
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

## **4. Design of a cascaded position and velocity controller for a pan-tilt camera tracking UAVs (Bachelor Thesis)**

<center>
<video width="70%" controls autoplay loop muted>
  <source src="./videos/OptoFence_Video_3.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

---

### Description
This project presents a **control system for tracking UAVs at high speeds and short distances** using a **pan–tilt (PT) camera**.  
Developed at the **Automation and Control Institute (TU Wien)**, the system addresses safety-critical scenarios by accurately tracking drone motion in real time.

The setup employs a **cascaded position and velocity controller** for the pan and tilt axes, each driven by a **Permanent Magnet Synchronous Motor (PMSM)** under **Field-Oriented Control (FOC)**.  
A **Kalman Filter–based sensor fusion** module smooths and predicts drone trajectories, ensuring continuous tracking even during temporary loss of visual contact.

The full system was modeled and simulated in **Matlab/Simulink**, **OpenCV** and **ROS**, with experiments using a **laser trajectory** as a reproducible target to validate controller performance.

---

### Key Highlights
- Achieved **1 rad/s** rotational speed, enabling tracking of drones up to **30 m distance** at **30 m/s** velocity.  
- Implemented **cascaded position–velocity control** for precise motor actuation.  
- Used **Kalman filtering** to interpolate missing detections and maintain smooth motion estimation.  

---

### Frameworks & Libraries
- ROS (Robot Operating System)  
- OpenCV  
- C++  
- Python  
- Matlab/Simulink
- OpenCV Kalman Filter (Sensor Fusion)  

---

### Summary
Developed a cascaded position–velocity control system with real-time sensor fusion for UAV tracking using a pan–tilt camera — achieving high-speed precision control and robust prediction even under visual occlusions.

# Personal Projects

## **1. 6-DOF Robotic Arm – Design, Simulation, and Control**

<center>
<video width="70%" controls autoplay loop muted>
  <source src="./videos/6dofra.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

<center>
<img src="./img/6_DOFRA_Rendered_1.png" width="45%" style="margin:10px;">
<img src="./img/6_DOFRA_Rendered_2.png" width="45%" style="margin:10px;">
</center>

---

### Description
A fully designed and built **6-DOF robotic arm**, actuated by **stepper motors** and mechanical components such as **3D-printed harmonic drives** and **timing belts**.  

The robot is controlled using the **Robot Operating System (ROS)** with the **MoveIt** motion planning framework.  
MoveIt provides both **motion planning** and **collision avoidance**, ensuring that the arm operates safely in simulation and the physical world.

The project demonstrates the full workflow from **mechanical design**, **simulation**, and **ROS integration** to **hardware control**.  
Through **RViz visualization**, planned trajectories are simulated before execution, and these are then transferred seamlessly to the real robot arm.

---

### Key Highlights
- **Custom-built 6-DOF robotic arm** driven by stepper motors  
- **3D-printed harmonic drives** and timing belt mechanisms for high precision  
- **MoveIt + RViz** integration for planning, visualization, and collision avoidance  
- **ROS-based control pipeline** for synchronized real-world execution  
- Modular architecture designed for future **vision-based pick-and-place** integration  

---

### Frameworks & Tools
- ROS  
- MoveIt  
- RViz  
- Python / C++  
- 3D Printing (Fusion 360 / PLA)  
- Stepper Motor Control (DRV8825 Drivers)  

---

### Summary
A 6-DOF robotic arm designed and controlled entirely through open-source tools — combining 3D-printed mechanics, ROS MoveIt motion planning, and real-to-sim synchronization for flexible robotic manipulation.
