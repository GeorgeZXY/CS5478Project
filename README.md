# Default-Course-Project
Default Course Project of NUS CS4278/CS5478 Intelligent Robots: Algorithms and Systems. This repo is the solution to the cousrework project in AY 24/25 Semester 1.

Carried out by a group of 3 students:

Wang Wenzhao A0276544W, Wei Xiaochen A0279556J, Zhou Xiangyu A0291289N

The initial files are cloned from this repo `https://github.com/NUS-LinS-Lab/Mobile-Manipulation` as provided by the course.

Deliverables:<br/>
NavigationAndPickMug.mp4 - video of the entire workflow. the robot first navigates from start position to goal position then picks the mug and place it in the open drawer. <br/>
MotionPlanningTests.mp4 - video showing the robot reach the 3 points for motion planning test.<br/>
PickAndPlaceMug.mpr - video of 3 successful pick and place operations. the navigation part is neglected in this video<br/>
CS5478Report.pdf - final report of the project.<br/>

File structures:

/simulation/main.py - main program to run the simulation. to run motion planning tests, set motionTest = True at line 25<br />
/simulation/nav/A_star.py - A* algorithm for path planning <br />
/simulation/nav/map.py - map of the environment with obstacles <br /> 
/simulation/nav/move_robot.py - motion control algorithm for driving the robot <br />
/simulation/motionPlanningTests.py - algorithm for performing the motion planning tests <br />
/simulation/pickMug.py - control functions related to the picking and placing mug task <br />
Other files are from the original repo.

Main branch contains everything of the solution. Other branches are used for developing part of the tasks or recording the videos
