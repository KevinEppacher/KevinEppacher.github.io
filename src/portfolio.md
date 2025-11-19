# Professional Work

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