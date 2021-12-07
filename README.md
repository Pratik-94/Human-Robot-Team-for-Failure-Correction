# Sociopulator- A Interactive Robot that uses Human Assistance to successfully complete tasks

**Author**: [Varun Walimbe](https://github.com/varun7860) Pratik Kumbhare

![node_graph](assets/workspace.png)

## About Sociopulator
Sociopulator is a manipulator designed to perform manipulation tasks like "pick and place" successfully by interacting with humans. In this project the manipulator has the task of sorting the three packages (Green,Red,Blue) in their respective bin.(Green - Green Bin, Red - Red Bin, Blue - Blue Bin). The human can give the robot any of these 3 tasks through voice command. The robot asks for feedback after completing each task given by the human. whenever the robot is not able to perfom the given task, it asks for assistance from the human. Human provides different kinds of solutions to the robot to resolve the problem faced and robot tries to complete the tasks with the help of these solutions.

The main aim of this project is to demonstrate the power of human robot interaction and human robot team.This project will show how an efficient communication between human and robot can help tacling dynamic real life environment issues faced in today's world. These real life problems are mentioned in the research question section below. We have tried to demonstrate some of these real life issues with a help of simple environment in ROS and pick-place task.

## Research Question
The pick and place task of 3 boxes kept on a table is pretty simple and straightforward to be solved autonomously by the robot. But consider a complex manipulation task of pick-placing more than 100 of boxes with different orientations. In real world, considering the physics and other mechanical factors,
solving this task autonomously becomes quite challenging.No matter how much you try to make the robot accurate in its mechanical, electrical or other properties there will always be a chance of robot failing due to mechanical shocks,electrical issues, physical issues ,singularity and many more. some examples of these issues are : "Robots fails to compute inverse kinematics", "Robot Drops the package", "Robot misbehaves" ....

Due to all this, these problems are inevitable atleast for now. In all these scenarious, assistance of human to the robot is a necessity for completing all such super complex tasks.For assisting the robots,there is a need for making robot understand the language of the humans.(voice to text...). And for understanding the language or what the human is exactly trying say the communication should be efficient,lucid and interleaving.This makes us conclude with the research question 

`How to communicate with a robot more efficiently and incorporate failure actions?`


## Installations
ROS --Version

- ROS Melodic : Ubuntu 18.04
- Other versions might be supported. It just that we have tested our project on ros melodic and it works completely fine

Install ROS Controller

- `sudo apt-get update` (optional)
- `sudo apt-get upgrade` (optional)
- `sudo apt-get dist-upgrade` (optional)
- `rosdep update` (optional)
- `sudo apt-get install ros*controller*`

Install Moveit

- `sudo apt-get install moveit`

Install Communication Libraries

- `sudo apt-get install espeak`

- `pip install pyttsx3`

- `pip install python-espeak`

- `pip install SpeechRecognition`

- `sudo apt-get install pyaudio`

## How to use the package

Building the package

- `catkin build sociopulator scarapulator_control`

- `source ~/catkin_ws/devel/setup.bash`

Launching the Socioulator Robot

- `roslaunch sociopulator robot.launch`

Run the Node for communication with the Robot

- `rosrun sociopulator robot.py`


## Success and Failure





