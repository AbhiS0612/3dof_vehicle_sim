# Matlab Simulation and Visualization of a 3-DOF Underwater Vehicle
This repository contains code to simulate the dynamics of an underwater vehicle in 3 degrees of freedom. The simulation can be easily configured for use with any underwater vehicle by modifying the model parameters in file 2. One can implement a variety of controllers by modifying one of the existing control functions(4 and 5 below) or writing their own control function. The results of the simulation can be visualized using the plot functions (7-12 below). Function 12 creates a dashboard that shows an animation the vehicle executing the given task and also provides time-synced state information and control input information. See the image below for a screenshot of this dashboard. 


## To run this code:
- Modify the path and file name in the file *plot_trajectory_2D.m* to choose where to save the video created.

- In *sim_3DOF.m*, choose how you want to view results (static plots/animation) by commenting out the other section. If you try and use both methods simultaneously, the animation gets splits across the wrong subplots (possibly a bug in Matlab).
- Run *sim_3DOF.m*.


## This repository contains the following files:
1. sim_3DOF.m: 	      	     script to execute simulation. Run this script
2. get_3DOF_modelparams.m:   function to set vehicle model parameters
3. DYN_3DOF.m:         	     function that evaluates the kinematics and dynamics of the vehicle
4. control_3DOF_FA.m:  	     control function for a fully-actuated vehicle
5. control_3DOF_UA.m:  	     control function for an under-actuated vehicle
6. J2.m:	      	           skew-symmetric operator, converts a scalar to so(2) 
7. plot_graphic_JHUROV.m:    plots a graphic of the JHUROV, indicating the COM frame and the l frame
8. plot_graphic_Iver.m:      plots a graphic of the JHUROV, indicating the COM frame and the l frame *(incomplete)*
9. plot_tank.m:              plots a graphic of the Hydrodynamic test tank
10. plot_controls_3DOF.m:    used to plot the controls once the simulation has run
11. plot_states_3DOF.m:      used to plot the states once the simulation has run
12. plot_trajectory_2D.m:    used to make an animation of the vehicle, state and control trajectories, and saves the animation as a avi file
13. scratch.m:               file used for dubugging purposes only. 

