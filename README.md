# Linking Top Down Views with First Person Views - Webots simulation of robot
# Jeff Krichmar, September 22, 2022
#    September 22, 2022: Initial release for Webots R2022a.  
Xing, J., Chrastil, E., Nitz, D., Krichmar, J. "Linking Global Top-Down Views to First-Person Views in the Brain", PNAS.

This repository contains the Webots simulation code for the robot exploration portion of our paper.  The controller for the Khepera robot is written in C. The top-down view looking down on the environment, first-person from the robot's camera, and the position information of the robot are saved in time-stamped files every 100 time steps.  The files are written in the controllers directory.   
