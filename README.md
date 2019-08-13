# PTHatControl
New method (as of 13th August 2019) of controlling the Pulse Train Hat with python3 instead of having to create a UWP. This lower level allows control of  the motors through Node-Red using a modified Python-Shell node

## Description of Files  
*DeltaIKS:* A module that takes in an XYZ coordinate and outputs a theta for that position (needs to be ran 3 times, one for each arm with XY rotated but 120 degrees each time)  
*Bezier3D:* A practise code showing how a path is generated from XYZ locations and breaking it into small segments so that the arm follows a curve  
*CommandMaker:* A module that uses the angles between segments and generates how many pulses the driver must fire and generates the serial string commands for the PTHat
*PTHModule:* The Module for the Pulse Train Hat found on *http://pthat.com/index.php/using-with-raspberry-pi/*
*FullArmSim:* The final combination of the above modules with *Bezier3D* as the backbone of the program
