# AZTEK
Computation of AZTEK trajectories for 3D radial UTE MR acquistions
Generate a text file with the amplitude values of the three gradient components Gx, Gy, Gz and the orientation sign which sustains the sequence overall 3D radial trajectory in kspace. It was developed in the framework of a GE MR system but it could drive gradients of any other manufacturers. 

# Intructions
Compile on Start_trajectory_comp.c or execute Execute_AZTEK_trajectory_comp.exe
Enter (1) the number of spokes (2) AZTEK_Twist (3) AZTEK_Shuffle (4) AZTEK_Speed when prompted to do so. Separate every number by a space.

# Dataset
An output example text file in 
AZTEK_trajectory_Spokes40000_Twist1_Shuffle1_Speed4.txt 
is provided for 40,000 spokes and parameters AZTEK_Twist=1, AZTEK_Shuffle=1, AZTEK_Speed=4 as implemented in the reference below.

# References and mentions
AZTEK: Adaptive Zero TE K-space trajectories
Tanguy Boucneau, Brice Fernandez, Florent Besson, Anne Menini, Florian Wiesinger, Emmanuel Durand, Caroline Caramella, Luc Darrasse, and Xavier Maître

