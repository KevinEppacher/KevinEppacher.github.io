# Professional Work

Below is a selection of my research and engineering projects, spanning semantic exploration, optimization-based control, and applied industrial robotics.

---
## **1. ROS 2 Reinforcement Learning Framework**

<center>
<img src="./videos/car_racing_demo.gif" width="70%">
</center>

---

### Description  
A modular **ROS 2 Deep Reinforcement Learning (DRL) framework** developed as a **commissioned project** to provide a standardized, extensible platform for end-to-end learning in robotics.  
The goal was to **lower the entry barrier for students and research teams** by enabling quick prototyping, reproducible training, and real-time introspection within ROS 2.

The framework integrates tightly with **Stable-Baselines3** and supports **plug-in-based environments**, allowing new tasks to be added without modifying the RL core.  
It comes with practical examples (CarRacing, LunarLander, CartPole) and extensive documentation covering **observation/action space design**, **reward shaping**, and **hyperparameter tuning**.

It also supports **vectorized environments** for parallel training and can **introspect live ROS 2 topics during learning** — enabling developers to visualize and debug agent behavior in real time, as shown below.

<center>
<img src="./img/vec_env_car_racing_training.png" width="70%" style="margin:10px;">
</center>

The framework emphasizes **reproducibility, scalability, and transparency**, making it an ideal foundation for both industrial and educational reinforcement-learning applications.

---

### Key Features  
- Unified training and evaluation pipeline for ROS 2 environments  
- Plug-in architecture for easily registering new environments  
- TensorBoard integration and live ROS 2 topic publishing during training  
- Supports vectorized environments for high-throughput parallel learning  
- Real-time introspection of published ROS 2 topics to monitor agent behavior  
- GPU-accelerated PPO, SAC, and TD3 training support  
- Ready-to-use environments (CartPole, LunarLander, CarRacing)  
- Clear YAML-based configuration for all algorithms  

---

### Frameworks & Tools  
![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)  
![Stable-Baselines3](https://img.shields.io/badge/Stable--Baselines3-DRL%20Library-orange)  
![PyTorch](https://img.shields.io/badge/PyTorch-2.2-red?logo=pytorch)  
![Gymnasium](https://img.shields.io/badge/Gymnasium-Environments-lightgrey?logo=openai)  
![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)  
![TensorBoard](https://img.shields.io/badge/TensorBoard-Visualization-orange?logo=tensorflow)  
![Docker](https://img.shields.io/badge/Docker-Reproducibility-blue?logo=docker)

*Built and tested under ROS 2 Jazzy with CUDA-enabled PyTorch 2.2 for GPU training.*

---

### Links  
- [GitHub Repository](https://github.com/KevinEppacher/ros2_reinforcement_learning_framework.git)

---

### Summary  
A professionally developed **ROS 2 reinforcement learning framework** unifying algorithm design, training, and evaluation in robotics.  
It bridges **educational usability** and **research-grade scalability**, empowering students, researchers, and engineers to prototype and deploy intelligent robotic behaviors efficiently.

---

---

## **2. Nonlinear Model Predictive Controller (nMPC) for Differential Drive Mobile Robot**

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

> *See also the [Optimization Lab – PyTorch-based MPC (ROS 2)](#3-optimization-lab-pytorch-based-mpc-for-robotics) for a lightweight educational re-implementation using PyTorch instead of CasADi.*

---

### Frameworks & Libraries  
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue?logo=ros)  
![Nav2](https://img.shields.io/badge/Nav2-Navigation-brightgreen?logo=ros)  
![CasADi](https://img.shields.io/badge/CasADi-Optimization-orange)  
![IPOPT](https://img.shields.io/badge/IPOPT-Solver-lightgrey)  
![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)  
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-blueviolet?logo=ros)  
![Docker](https://img.shields.io/badge/Docker-Containerization-blue?logo=docker)  

---

### Links  
- [GitHub Repository](https://github.com/KevinEppacher/walle_ws.git)  
- [Download unpublished Research Paper (PDF)](./papers/nMPC.pdf)

---

### Summary  
A high-performance nonlinear MPC for mobile robots using CasADi and IPOPT — delivering smooth, constraint-aware motion planning and serving as a foundation for subsequent PyTorch-based re-implementations in ROS 2.

---

## **3. Optimization Lab – PyTorch-Based MPC for Robotics**

<center>
<img src="./videos/mpc_demo_1.gif" width="70%">
</center>

---

### Description  
A **PyTorch-based Model Predictive Control (MPC)** framework developed as part of a **university optimization lab**, demonstrating how numerical optimization can be applied to control and planning problems in robotics.  
Unlike the earlier CasADi-based MPC, this version leverages **PyTorch autograd and optimizers (Adam/LBFGS)** directly — without relying on external NLP solvers — to teach students how to *formulate and solve control problems from first principles*.

Developed as a **commissioned project**, the lab provides a complete **ROS 2 Jazzy package (`mpc_local_planner`)** that serves as both a tutorial and a working local planner.  
It includes comprehensive documentation explaining:
- optimization in localization, planning, and control,  
- MPC cost shaping and constraints,  
- and real-time optimization loops using PyTorch tensors.

---

### Key Highlights  
- Created an **Optimization Lab** for the *UAS Technikum Vienna* robotics curriculum  
- Developed a **didactic ROS 2 Jazzy package**: `mpc_local_planner`  
- Implemented **nMPC with PyTorch optimizers** (Adam, LBFGS, RMSProp)  
- Demonstrated **gradient-based MPC without CasADi/IPOPT**  
- Integrated with **Nav2** for trajectory tracking using costmap penalties  
- Provided **extensive documentation, code comments, and exercises**  
- Used in teaching labs to show **real-time control, optimization, and differentiable robotics** concepts  

---

### Frameworks & Tools  
![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)  
![Nav2](https://img.shields.io/badge/Nav2-Navigation-brightgreen?logo=ros)  
![PyTorch](https://img.shields.io/badge/PyTorch-2.2-red?logo=pytorch)  
![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)  
![TorchOptimizer](https://img.shields.io/badge/Optimizers-Adam%2FLBFGS%2FRMSProp-lightgrey)  
![Docker](https://img.shields.io/badge/Docker-Reproducibility-blue?logo=docker)  

*Built and tested under ROS 2 Jazzy using CUDA-enabled PyTorch 2.2.*

---

### Links  
- [GitHub Repository](https://github.com/KevinEppacher/mpc_local_planner.git)

---

### Summary  
A **university lab project** showcasing optimization for robotics using **PyTorch as a numerical solver**.  
It bridges classical control and differentiable programming by re-implementing nMPC entirely in PyTorch — illustrating how learning-based and optimization-based control can converge within modern ROS 2 pipelines.

## **4. Automated Sensitivity Measurement System (AIRSKIN)**

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
- A **custom ImGui C++ HMI** for switching between *Freedrive Mode* and *External Control Mode*  

Once all measurement points are defined, **MoveIt** executes a fully automated sequence. The system visualizes force vectors in **RViz** and overlays a 3D point cloud from an integrated RGB-D camera.

---

### Frameworks & Libraries  
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue?logo=ros)  
![MoveIt](https://img.shields.io/badge/MoveIt-Motion%20Planning-purple?logo=ros)  
![ImGui](https://img.shields.io/badge/ImGui-C%2B%2B%20GUI-lightgrey)  
![RViz](https://img.shields.io/badge/RViz-Visualization-orange?logo=ros)  
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-blueviolet?logo=ros)  
![Docker](https://img.shields.io/badge/Docker-Containerization-blue?logo=docker)  
![UR ROS Driver](https://img.shields.io/badge/UR--ROS--Driver-UR10%20Control-darkblue)  

---

### Links
- [GitHub Repository](https://github.com/KevinEppacher/goldilocks_sensibility_ws.git)

---

### Summary
Automated robotic test bench for AIRSKIN pad calibration — measuring and visualizing tactile sensitivity through force–displacement mapping.

---

## **5. Monte Carlo Localization (Particle Filter) for Mobile Robots**

<center>
<img src="./videos/mcl.gif" width="70%" alt="Monte Carlo Localization simulation in Gazebo">
</center>

---

### Description
A **Monte Carlo Localization (MCL)** system — also known as a **Particle Filter** — implemented in **C++** for **Differential Drive Mobile Robots** using **ROS Noetic**.  

The algorithm estimates a robot’s pose on a known map by maintaining a set of weighted samples (“particles”), each representing a possible state hypothesis.

---

### Key Highlights
- Reliable localization with only **100 particles**, compared to typical **500–3000 AMCL** particles.  
- **80% randomized resampling** per iteration for fast recovery from localization loss.  
- **Gazebo simulation** using a TurtleBot in an apartment environment.  

---

### Frameworks & Libraries  
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue?logo=ros)  
![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B)  
![Eigen](https://img.shields.io/badge/Eigen-Math%20Library-lightgrey)  
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-blueviolet?logo=ros)  
![RViz](https://img.shields.io/badge/RViz-Visualization-orange?logo=ros)  
![Docker](https://img.shields.io/badge/Docker-Containerization-blue?logo=docker)  

---

### Links
- [GitHub Repository](https://github.com/KevinEppacher/Probabilistic_Lab.git)
- [Download Research Paper (PDF)](./papers/efficient_monte_carlo_localization_for_mobile_robots_implementation_and_evaluation_Eppacher.pdf)

---

### Summary
Robust and efficient Monte Carlo Localization achieving high accuracy with minimal particles through adaptive resampling — enabling fast and reliable robot pose estimation in dynamic indoor environments.

---

## **6. Design of a Cascaded Position and Velocity Controller for a Pan–Tilt Camera Tracking UAVs (Bachelor Thesis)**

<center>
<video width="70%" controls autoplay loop muted>
  <source src="./videos/OptoFence_Video_3.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

---

### Description
A **control system for tracking UAVs** using a **pan–tilt camera** with cascaded position and velocity control.  
Developed at **Automation and Control Institute (TU Wien)**, the system enables accurate drone tracking in real time with predictive correction via **Kalman filtering**.

---

### Frameworks & Libraries  
![ROS](https://img.shields.io/badge/ROS-Control-blue?logo=ros)  
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-green?logo=opencv)  
![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B)  
![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)  
![Matlab](https://img.shields.io/badge/Matlab%2FSimulink-Control-orange?logo=mathworks)  

---

### Summary
Developed a cascaded position–velocity control system with real-time sensor fusion for UAV tracking using a pan–tilt camera — achieving high-speed precision control and robust prediction even under visual occlusions.

---

# Personal Projects

Outside of my academic research and industrial work, I enjoy building and experimenting with robotic systems in my free time — exploring mechanical design, embedded control, and intelligent motion planning.  
These projects allow me to prototype, test, and iterate on new ideas that blend classical robotics with modern AI-driven methods.

---

## **1. 6-DOF Robotic Arm – Design, Simulation & Control**

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

### Frameworks & Tools  
![ROS](https://img.shields.io/badge/ROS-Integration-blue?logo=ros)  
![MoveIt](https://img.shields.io/badge/MoveIt-Motion%20Planning-purple?logo=ros)  
![RViz](https://img.shields.io/badge/RViz-Visualization-orange?logo=ros)  
![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)  
![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B)  
![SolidWorks](https://img.shields.io/badge/SolidWorks-Mechanical%20Design-red)  
![3D Printing](https://img.shields.io/badge/3D%20Printing-Prototyping-darkgreen)

---

### Summary
A 6-DOF robotic arm designed and controlled entirely through open-source tools — combining 3D-printed mechanics, ROS MoveIt motion planning, and real-to-sim synchronization for flexible robotic manipulation.
