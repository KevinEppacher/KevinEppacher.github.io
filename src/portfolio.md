# Professional Work

Last year, I developed a nonlinear Model Predictive Controller (nMPC) based local planner for a Differential Drive Mobile Robot (DDMR) within the ROS Noetic navigation stack.

Unlike conventional reactive planners, an nMPC uses a mathematical model (in this case, a kinematic model) to predict future states and optimize them over a defined horizon. This is done by minimizing a cost function while satisfying both soft and hard constraints, such as obstacle distance, velocity bounds, and input limits.
The local planner was implemented entirely in Python, utilizing the CasADi framework for formulating the nonlinear optimization problem. The IPOPT solver (Interior Point Optimizer) was used to solve the problem at runtime efficiently.

As part of the evaluation, I compared the nMPC planner against established ROS local planners:
- Dynamic Window Approach (DWA)
- Timed Elastic Band (TEB)
The nMPC planner receives a global reference trajectory (e.g., from an RRT-based global planner) and computes optimized control inputs at higher frequency to achieve smooth and safe local obstacle avoidance, even in dynamic and narrow environments.

- The system was simulated in Gazebo with a TurtleBot platform and fully integrated with ROS Noetic.
- 🐳 A GPU-enabled Docker container is provided for fast and reproducible setup.
- 📁 The GitHub repository includes the full source code, parameter tuning, scientific paper, and presentation slides.
🔗 https://lnkd.in/dgpq7y2Y

Hashtag#Robotics Hashtag#MPC Hashtag#nMPC Hashtag#CasADi Hashtag#IPOPT Hashtag#PathPlanning Hashtag#ROS Hashtag#ROSNoetic Hashtag#

/home/kevin/Documents/Allgemein/KevinEppacher.github.io/videos/nMPC.mp4

---

Collaborative Project with AIRSKIN– Automated Sensitivity Measurement System 🤖📏
Last year, Moritz Dönges and I had the opportunity to design an automated measurement system in collaboration with AIRSKIN at the University of Applied Sciences Technikum Wien.
🎯 The goal was to measure the force required to trigger an AIRSKIN pad and the displacement at that moment. This allowed us to calculate the spring constant, thereby defining the sensitivity at each measurement point. This system plays an important role in identifying weak points and supporting further development of AIRSKIN pads.
🛠️ The system was built using ROS Noetic and Docker. We developed a custom C++ ImGui HMI that allows the UR10 robot to be switched between Freedrive Mode for teaching points and External Control Mode for automated measurements. Thanks to the ROS UR hardware interface, the robot can seamlessly switch between modes, start/stop programs, and communicate with the FT sensor via UR ROS bridge (TCP/IP protocol).
📍 Once all points are taught, a MoveIt MoveGroup program automatically moves the robot to each point, measuring and visualizing the applied force in RViz. A 3D camera visualizes a point cloud, allowing for intuitive analysis of the AIRSKIN pad and the corresponding force vectors.
🚧 Future plans include integrating RGB-D-based obstacle avoidance to dynamically navigate between measurement points.
The entire system was also implemented in Gazebo for simulation purposes.
🖥️ The project is available here:
https://lnkd.in/dYwSsFtR

Hashtag#Robotics Hashtag#ROS Hashtag#AIRSKIN Hashtag#UR10 Hashtag#Automation Hashtag#ForceMeasurement Hashtag#HumanRob

Paths:
videos/sensibility_measurements.mp4


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