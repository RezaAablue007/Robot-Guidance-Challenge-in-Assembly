# Robot-Guidance-Challenge-in-Assembly

## Project Introduction
The final project of this course involves programming the eebot mobile robot with a navigation system to navigate through a maze. The robot must successfully find its way to the destination and then backtrack to its starting point. The robot starts at the entry point of the maze, navigates through "S" turns, and makes decisions at junctions. If it encounters a dead-end, it executes a 180-degree turn to explore other branches. The correct turns are recorded in a stack for the path-retracing operation.

## Description of Completed Project
The completed project consists of six crucial components: Navigation Manager, Guidance Routine, Sensor Scanner, Motor Speed Control, Rotation Counters, and Bumper Detectors. The Navigation Manager manages the robot's navigation through the maze, using routines to search for correct turns at junctions. The Guidance Routine interprets the guider readings and controls the motors accordingly. The Sensor Scanner reads photodetection sensors to determine open-ended tracks and dead-ends. The Motor Speed Control manages the relative speeds of the motors for steering and turning. The Rotation Counters count correct turns taken by the robot, while the Bumper Detectors detect collisions.

## Problems Encountered & Solution
The team faced challenges while implementing the guidance routine and sensor scanner. They had to find a way to use the six scanners to detect valid paths. By understanding the guider documentation, they created a checking method for the robot, which involved assigning numbers to paths and detecting obstacles. Combining the knowledge from previous labs and understanding the documentation helped compile the components together.

## How Would You Do Things Differently?
One key feature that could be added to the robot is a backtrack functionality using a stack. The correct path taken by the robot could be accumulated into the stack in the correct order. Upon reaching the destination, the robot could then backtrack to the starting position using the stack's LIFO (last in, first out) property.

## What Worked Well
Four subsystems that worked well in the project were the Navigation Manager, Rotation Counters, Bumper Detectors, and Motor Speed Control. These subsystems were adapted from previous labs and played crucial roles in maneuvering the robot through the maze efficiently.

## Conclusion
The robot successfully navigates through the maze using the implemented subsystems. The navigation manager, rotation counters, bumper detectors, guidance routine, sensor scanner, and motor speed control work together to allow the robot to detect paths and obstacles, solve the maze, and potentially add a backtrack feature using a stack. Overall, the project was a success and demonstrated the proficiency of the robot's navigation capabilities.
