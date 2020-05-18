# SniperBot
This was my final project for the Computer Science 2 course I took at Montgomery County Community College in 2015. We created an autonomous, color seeking robot, called Sniper Bot. We had one month to complete this project.

**Project Members**
* Michael Giuliano
* Christian Renner
* David Taylor

**Core Hardware Used**
* Arduino - Performs object detection and controls all the servos for the wheels and camera movement. **(SniperBot.cpp)**
* Raspberry Pi - Takes captures from the camera and performs color detection to aquire a target. **(Main.cpp, GPIO.cpp/h, ColorDetection.cpp/h)**
* USB Webcam - Movement is controlled by two servos; one for looking left and right, one for looking up and down.
* Laser Pointer - Attached to the top of the webcam. Controlled by the Arduino.
* Portable USB Battery Bank - Supplies power to the whole robot.

**Robot Operations**
* Roam while performing object detection using three ultrasonic sensors.
* Detect colors in its view through a camera.
* When a large blob of a specific color is detected, it stops moving, aims the laser pointer at the target color, and fires the laser repeatedly while playing a ticking sound through a speaker.
