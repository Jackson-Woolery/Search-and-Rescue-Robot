# Final-Demo
Final-Demo for Group 007 in SEED Lab

__Purpose of Repository:__

This repository contains code for Demo-2 for SEED Lab. This code for both Arduino and Raspberry Pi
programs a constructed robot to navigate a path around a convex shape made up of six Aruco marker
"beacons". The robot detects the nearest marker and determines the distance and angle which is 
eight inches to the right of the beacon using computer vision techniques. The robot then drives to that 
location before beginning to turn while searching for the next beacon. This process is repeated until 
the final beacon is reached.



__File Organization:__

CV_CameraCalibration.py: Raspberry Pi program to get the camera calibration data. Includes image
	capture, image processing, calibration data generation, and storage of data to file.

CV_CameraCalibrationData.npz: Output file for 'CV_CameraCalibration.py'. Contains camera
	calibration data used as input for 'CV_Final-Demo.py'.

CV_Final-Demo.py: Main Raspberry Pi computer vision module for detecting Aruco markers, discerning
	the nearest marker, determining the angle and distance to a number of inches to the right of
	this marker, and sending state information along with the distance and angle to the Arduino.

CV_Final-Demo_Disconnected: Raspberry Pi computer vision module which mirrors the main CV module,
	but has slight alterations to work on its own without the Arduino. Used for testing the computer 
	vision module.
