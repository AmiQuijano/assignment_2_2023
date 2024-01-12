# ROS robot simulator

Assignment 2 for Research Track 1 Course

MSc Robotics Engineering at University of Genova

-------------------

# Simulator description 
This ROS simulator encompasses the visualization of a mobile robot able to reach a desired target by avoiding obstacles. The obstacles are recognized by an onboard laser. 
The ROS package of this simulator was obtained from this [repository](https://github.com/CarmineD8/assignment_2_2023.git)

### Visualization tools
* **RViz**: a ROS 3D visualization tool which allows the user to view the robot model as well as the output of its sensors (i.e. laser, camera). It allows to see what the robot "sees". In this case, it visualizes the robot, its motion on a grid surface and the output of the laser, i.e. the obstacles or walls detected by the robot's laser according to the laser's reach.
* **Gazebo**: a ROS 3D realistic simulation tool that serves as a physical simulator. It allows to see the robot as an external observer. In this case, it visualizes the robot, its motion and the world or arena where the robot navigates including the obstacles and walls in it.

# Folders description
* **world**: Contains the .world file corresponding to the information to load the environment or arena where the robot will navigate in Gazebo.
* **urdf**: Contains files describing the control and structure of the robot including information regarding joints and links and their position, orientation, dynamics, among others.
* **scripts**: There are 3 already given Pyhton scripts, *wall_follow_service.py*, *bug_as.py* and *go_to_point_service*. These work together to make the robot move towards a goal sent by an action-client, making the robot head towards the goal but also follow obstacles until it can again heards the goal (surroung wall until there isn't wall anymore).
* **action**: Contains the .action file used by the action-server *bug_as.py*.
* **launch**: Contains the .launch file which executes the nodes (i.e. the .py scripts) as well as the simulation environments
* **config**: Contains .rviz files which store the configuration settings, layouts and displays used by RViz.

# Assignment description
