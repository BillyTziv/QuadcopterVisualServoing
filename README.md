# Autonomous Quadrotor Landing using Visual Servoing
My diploma thesis project implementing the landing proedure of a quadrotor model.

# Thesis Overview
The total project had two phases. In the first phase I developed a simulation of a quadrotor model in MATLAB software. In the second phase, I tried to convert the simulation principles in the real model and solve any extra problems and issues that may occur. There are still some issues due to the lack of budget though. So please have in mind that the code must be used with caution since we talk about UAV.

## IBVS
IBVS stands for Image Based Visual Servoing. We usually have some desired points of interest and the current points. A camera usually feed the necessary frames to a controller and then through the robot model, the controller outputs the desired velocities required to reach the destination points.

## Implementations
In this project we developed two core implementations. In the first one, we have four (4) steady points of interest and the quadrotor tries to reach them. In the second scenario we have again four (4) points of interest but now they can have a linear velocity in 2D space. Image those points like they were in the top of a car or platform in the ocean.

## Requirement
We require the points of interest to be visible by the camera. In this implementation we do not use any kind of GPS or similar mechanism that scan and find the points of interest.

## Disturbances
We have added a varialbe that implement gausian white noise as the air disturbance.
 
# Files & Folder Structure
The files and folders structure will be listed below. If you find any error or bug please let me know.

- ** distanceLanding ** contains an arduino sketch which was used to get the height from the ground in cm
- ** Screenshots ** contain several screenshots used in this project (visual media always helps)
- ** QuadLandingSimulation ** contains MATLAB source code that implement the simulation.

## Simulation Source Code File Structure
- ** main.m ** file contains the main loop and this is the first file that you should run.
- ** visualize_FULL.m ** contains the visualization fo the MATLAB code. A 3D plot and many diagrams that allows you to get data feedback. 

# Licence
This project is licensed under the MIT License - see the LICENSE.md file for details

# Contact
For any questions or interest for cooperation please drop me an email and I will get back to you asap. My email is vtzivaras[at]gmail[dot]com. You can also find more contact details in my website http://vasilistzivaras.gr
