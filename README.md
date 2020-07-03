# Repository Containing files for Matlab Simulation and Visualization of a 3 DOF Underwater Vehicle

## To run this code:
1. Modify the path and file name in the file *plot_trajectory_2D.m* to choose where to save the video created.

2. In *sim_3DOF.m*, choose how you want to view results (static plots/animation) by commenting out the other section. If you try and use both methods simultaneously, the animation gets splits across the wrong subplots (possibly a bug in Matlab).
3. Run *sim_3DOF.m*.


## This repository contains the following files:
1. sim_3DOF.m: 	      	    script to execute simulation. Run this script

2. get_3DOF_modelparams.m:   function to set vehicle model parameters
3. DYN_3DOF.m:         	     function that evaluates the kinematics and dynamics of the vehicle
4. control_3DOF_FA.m:  	     control function for a fully-actuated vehicle
5. control_3DOF_UA.m:  	     control function for an under-actuated vehicle
6. J2.m:	      	         skew-symmetric operator, converts a scalar to so(2) 
7. plot_graphic_JHUROV.m:    plots a graphic of the JHUROV, indicating the COM frame and the l frame
8. plot_graphic_Iver.m:      plots a graphic of the JHUROV, indicating the COM frame and the l frame *(incomplete)*
9. plot_tank.m:              plots a graphic of the Hydrodynamic test tank
10. plot_controls_3DOF.m:    used to plot the controls once the simulation has run
11. plot_states_3DOF.m:      used to plot the states once the simulation has run
12. plot_trajectory_2D.m:    used to make an animation of the vehicle, state and control trajectories, and saves the animation as a avi file
13. scratch.m:               file used for dubugging purposes only. 

