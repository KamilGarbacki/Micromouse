# Micromouse
In this project I designed, built and programmed my own Micromouse.
This project is being carried out as part of the SKN Creative AGH science club.

## What is a micromouse
A Micromouse is a small autonomous robot designed to navigate through a maze, with the goal of finding the optimal route to reach the center. 
The mouse needs to keep track of where it is, discover walls as it explores, map out the maze and detect when it has reached the goal. 
Having reached the goal, the mouse will typically perform additional searches of the maze until it has found an optimal route from the start to the finish. 
Once the optimal route has been found, the mouse will traverse that route in the shortest achievable time. 

## Project Developed with:
- PlatformIO
- Arduino UNO
- Ultrasonic Sensors US-015
- Micro DC Motors with Encoder-SJ02 SKU FIT0458
- Quad Motor Driver Shield for Arduino SKU DRI0039

## How does it work?
Robot uses Flood Fill algorithm to navigate through maze, while using ultrasonic sensors to detect any obstacles on its way. 
The encoders balance out the speed of the motors to prevent the Mice from drifting to one side, and ensure we travel the given distance. 
Robot moves in an X by Y grid, while trying to reach user defined goal from user defined start cell.
The maze and path of the robot is updated after each move.

## DONE:
- ~~Simple Pid System to ensure balanced speed of both motors~~
- ~~Flood Fill Algorithm for maze solving~~
- ~~Sensor integration for detecting maze walls in real time~~
- ~~Maze Management system for updating maze state in real time~~

## TODO:
- More advanced pid algorithm reduce rate of failues caused by motors inaccurate
   
