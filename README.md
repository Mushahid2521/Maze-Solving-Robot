# Maze Solving Robot

This Maze Solver Robot was built for a Robotics Comepetition organized by Islamic University of Technology in 2017. There were two rounds. In the first round the robot had to follow a defined line. In the second round the maze had to be solved. 

## Hardware Used
1. Arduino
2. 8 IR sensor and receiver (for detecting line)
3. Motor Driver
4. 2 Gear motors
5. LiPo Battery

## Algorithm
Step 1: By following the Right Hand rule it travaerse the maze. Means in a particular intersection it turn right. For each intersection it stores the turns as 'L' (Left), 'R' (Right), 'S' (Straight), 'B' (Back) 
Step 2: At the finishing point it generates an array containing the turns. It generates the shortest path by replacing the three turns into a single short turn. 
```RBL->B```
```RBS->L```
```LBR->B```
```SBR->L```
```SBS->B```
```RBR->S```
