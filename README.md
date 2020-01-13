# Smart-Luggage
People Tracking Smart Luggage in Python and YOLO.
## Libraries
pyrealsense2, OpenCV, breezycreate2
## Hardware
Nvidia Nano, D435 depth camera, iRobot 
## Introduction
It may be common to see a suitcase is tracking people in the airport hence passagers could free their hands. However, most of this kind of luggage needs a remote controller to direct the luggage. This project innovation is to replace the controller with a camera on the luggage. Hence, it is not necessary for the passenger to carry the remote controller any more, what they need is only the luggage case.
## Method
To track people without seeing the face from the target. We use YOLO for detecting people. YOLO data sets and weights are used to reduce the training time during the process. Deep neural network module is used for training the input.
On the other hand, to get the distance for the target people, we use the real sense depth camera D435. We use the breezycreate2 library to control the robot after the command is received from the PC. The command could be transferred from the PC to the microcontroller with the socket. 
## Code 
The iRobot.py file runs on the microcontroller and the trackpeople.py runs on the PC. The codes used socket to communicate with each other.
## Demo Video
https://youtu.be/pIGHeG1ak5M

