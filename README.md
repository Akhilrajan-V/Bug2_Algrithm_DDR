# Bug2_Algrithm_DDR

# This is a readme file created by Akhilrajan Vethirajan for ENPM 690 Homework 3
# simulated in gazebo/ros using RosPy (Python)

Please make sure all libraries are preinstalled in your system to run the two python scripts

1. Copy the model4_new directory into the source (src) directory of catkin workspace in your system
2. Go to the src directory in model4_new directory and open a new terminal 
3. Type "chmod +x teleop.py" to make the script executable
3. Now make bug2_obstacle_avoidance.py executable (type -> "chmod +x bug2_obstacle_avoidance.py")
4. Cd into catkin workspace and type catkin_make, to build and setup the workspace
5. Open a new terminal and type "roslaunch model4_new project1.launch" to launch gazebo simulation environment
6. For Teleop control:
	
	*Open a new terminal and type "rosrun model4_new teleop.py"
	*Use the keys i,j,l,m to control robot 
	*Press k to stop the robot
		
	Note: You have to stop the robot before trying to change its direction, press k, before pressing each key   

7. For obstacle avoidance:
	 *Type "roslaunch model4_new project1.launch" to launch gazebo simulation environment in a terminal. 
         *And in a new terminal type "rosrun model4_new bug2_obstacle_avoidance.py".
         Code will terminate after completion.
         
Two video files have been attached as answers to the given questions. 
Their respective scripts are in src/ directory.
