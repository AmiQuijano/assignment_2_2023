# ROS robot simulator

Assignment 2 for Research Track 1 Course

MSc Robotics Engineering at University of Genova

-------------------

# Simulator description 
This ROS simulator encompasses the visualization of a mobile robot able to reach a desired target by avoiding obstacles. The obstacles are recognized by an onboard laser. 
The ROS package *assignment_2_2023* containing all the files for this simulator was obtained from this [repository](https://github.com/CarmineD8/assignment_2_2023.git)

### Visualization tools
* **RViz**: a ROS 3D visualization tool which allows the user to view the robot model as well as the output of its sensors (i.e. laser, camera). It allows to see what the robot "sees". In this case, it visualizes the robot, its motion on a grid surface and the output of the laser, i.e. the obstacles or walls detected by the robot's laser according to the laser's reach.
* **Gazebo**: a ROS 3D realistic simulation tool that serves as a physical simulator. It allows to see the robot as an external observer. In this case, it visualizes the robot, its motion and the world or arena where the robot navigates including the obstacles and walls in it.

### Folders description
* **world**: Contains the .world file corresponding to the information to load the environment or arena where the robot will navigate in Gazebo.
* **urdf**: Contains files describing the control and structure of the robot including information regarding joints and links and their position, orientation, dynamics, among others.
* **scripts**: There are 3 already given Pyhton scripts, `wall_follow_service.py`, `bug_as.py` and `go_to_point_service.py`. These work together to make the robot move towards a goal sent by an action-client, making the robot head towards the goal but also follow obstacles until it can again heards the goal (surroung wall until there isn't wall anymore).
* **action**: Contains the .action file used by the action-server `bug_as.py`.
* **launch**: Contains the .launch file which executes the nodes (i.e. the .py scripts) as well as the simulation environments
* **config**: Contains .rviz files which store the configuration settings, layouts and displays used by RViz.

# Assignment description
The task carried out was to write and add to the previously described package the necessary scripts, message and service files for creating the following 3 nodes:
1. **Node (a)**: Create a node that implements an action-client, allowing the user to input in the terminal a target (x, y) or to cancel it. Use the feedback of the action-server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x, y, vel_x, vel_z), by relying on the values published on the topic /odom.
2. **Node (b)**: Create a service node that, when called, returns the coordinates of the last target sent by the user. 
3. **Node (c)**: Create a service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed. The size of the averaging window is set as a parameter in the launch file.

As well as editing the .launch file to make these nodes run as well.

# Installing and Running
The simlator and its files require Ubuntu 20.04, Python 3 (already installed within Ubuntu 20.04) and ROS Noetic. If ROS Noetic is not yet installed in your Ubuntu system follow the steps found in the [Ubuntu install of ROS Noetic ](http://wiki.ros.org/noetic/Installation/Ubuntu) making sure to install the *Desktop-full* version.

To download the simulator, install git and clone this repository in the *src* folder of your *ROS workspace*. Create it if you don't have one by following the [Create a Workspace tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
$ git clone https://github.com/AmiQuijano/RT1_ROS_assignment.git
```
Once all the dependencies are installed,

1. Open a terminal and start ROS
```
$ roscore
```
2. Open another terminal and build the workspace
```
$ catkin_make
```
3. Launch the simulation with the roslaunch command
```
$ roslaunch assignment_2_2023 assignment1.launch
```
# Assignment solution 
## Node (a)
The code in the file `acNode_a.py` contains the following functctions and main explained as follows:

Inside the class `ActionClient` the implemented functions are:

### __init__(self)
This function initialized the ROS action-client, subscriber, publisher and message as required

Arguments:
* `self`: Instance of the class `ActionClient`.

Pseudocode:
```
Function __initi__(self)
  Initialize action-client the /reaching_goal server with the PlanningAction action type
  Wait for the action-server to start
  Initialize goal variable as a PlanningGoal message type to send to the action-server
  Initialize subscriber for /odom topic with Odometry message type and odom_callback function


  Create goal message for the action server
        Subscribe to /odom topic for robot position and velocity
        Create publisher for custom message /PosVel

    Method get_user_input():
        Print prompt for user input
        Infinite loop:
            Check for user input:
                If 'c', cancel the current goal
                Else:
                    Try to get goal coordinates from user input
                    Set goal coordinates in the goal message
                    Send the goal to the action server with feedback callback

    Method odom_callback(msg):
        Extract position and velocity information from /odom
        Create custom message PosVel
        Publish the custom message

    Method feedback_callback(feedback):
        Print feedback from the action server

Main section:
    Initialize ROS action-client node
    Create an object of ActionClient
    Wait for 2 seconds
    Prompt user for input target
    Keep the script running until the node is shut down
```

## Node (b)

## Node (c)

## Launch file
The launch file was changed adding the following:
* Launch of **Node (a)** with the script `acNode_a.py` with screen output.
* Launch of **Node (b)** with the script `srvNode_b.py` with screen output.
* Launch of **Node (c)** with the script `srvNode_c.py` with screen output.
* Parameter `window size` to set the number of last velocity readings to averaging in **Node (c)**. Currently set as 100.

## Visualization/Confirmation of nodes functionality
To check that the nodes work optimally or to visualize the messages created:
* **Feedback**: `actual_pose` and `status` feedbacks are visible automatically in the same terminal where the package was launched. To confirm the values are correct, in another terminal run
```
$ rostopic echo /reaching_goal/feedback
```
* **(x, y, vel_x, vel_y) custom messages**: To visualize the custom messages `PosVel` published in the topic `/PosVel`, open another terminal and run
```
$ rostopic echo /PosVel
```
To confirm that these values are correctly obtained from the `/odom` topic, in another terminal run
```
$ rostopic echo /odom
```
* **Last target**: To visualize the last target sent by the user it is necessary to call the service `/get_last_target` in order to get the responds. For this, open a new terminal and run
```
$ rosservice call /get_last_target
```
The user itself can confirm if the displayed target coordinate is indeed the last one inputted.
* **Distance from target and average speed**: To visualize the distance from the target and average speed it is necessary to call the service `/get_dist_speed` in order to get the response. For this, open a new terminal and run
```
$ rosservice call /get_dist_speed
```

## Possible improvements
The solution implemented fullfills the requirements stated by the *Assignment description*. However, there are improvement opportunities to make the simulator clearer for the user or make it have more functionalities in case of user mistake:
* **Goal mark in Gazebo**: It would be useful to have a visualization of the goal that the user has inputted in order to have a clear idea of where the robot is heading or whether or not it is heading/arriving to the goal.
* **Error message for unreachable goals**: It would be more user-friendly and time-saving to have an error message in the terminal if the user inputs a goal coordinate that is unreachable by the robot given the used world's limits (in case the world is an enclosed arena like the one used in this simulation). This would avoid having the user cancel the goal until realizing that the robot cannot reach it. The cancel option should still be kept in case the user wants to cancel the current inputted goal.
* **Launch files for opening useful terminals**: It would be very useful to have 
