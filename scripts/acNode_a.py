#!/usr/bin/env python3

# Ami Sofia Quijano Shimizu
# Reseach Track 1 - Assignment 2 a)

# Instructions: Create a node that implements an action-client, allowing the user to input in the terminal a target (x, y) or to cancel it. Use the feedback of the action-server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x, y, vel_x, vel_z), by relying on the values published on the topic /odom.


# Useful imports
import rospy
import sys
import select
import actionlib
import actionlib.msg
from nav_msgs.msg import Odometry # Import message type of /odom topic
from assignment_2_2023.msg import PlanningAction, PlanningGoal # Import action message type
from assignment_2_2023.msg import PosVel # Import custom message type


class ActionClient:
    
    def __init__(self):
    # Initialization function for the ROS action-client, subscriber, publisher and message as required
        
        # Create an action-client for /reaching_goal server
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        
        # Wait for action-server to start up
        self.action_client.wait_for_server()
        
        # Create the goal message to send to the action-server
        self.goal = PlanningGoal()
	
	# Create a subscriber that listens to /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Create a publisher to publish position and velocity (x, y, vel_x, vel_z) as a custom message in the new /PosVel topic
        self.posVel_publisher = rospy.Publisher('/PosVel', PosVel, queue_size=10)


    def get_user_input(self):
    # Function that asks the user for a goal coordinate or allows user to cancel current goal. 

        print("Enter the goal coordinates as 'x y' or enter 'c' to cancel the goal: ")
        
        # Inifinite loop to keep active the receiving of inputs on terminal
        while not rospy.is_shutdown():
            
            user_input, output, exception  = select.select([sys.stdin], [], [], 1)
            
            # If there is an input, read it
            if (user_input):
                user_input = sys.stdin.readline().rstrip()
                
                # If the input is 'c' then cancel the current goal
                if user_input.lower() == 'c': 
                    self.action_client.cancel_goal()
                
                # If the input is other than 'c', 
                else: 
                    try:
                        # Split user's input into two substrings based on the space ' ' and convert each substring into a floating-point number
                        goal_x, goal_y = map(float, user_input.split())
                        
                        # Set goal coordinates x and y
                        self.goal.target_pose.pose.position.x = goal_x
                        self.goal.target_pose.pose.position.y = goal_y
                        
                        # Send the goal to the action-server and receive its feedback (actual pose and state)
                        self.action_client.send_goal(self.goal, feedback_cb=self.feedback_callback)
                        
                    
                    except ValueError:
                    # Display error message if the user's input is neither 'c' nor a pair of floats separated by a space ' '
                        print("INVALID INPUT. Please enter goal coordinates as 'x y' or enter 'c' to cancel the goal: ")
              

    def odom_callback(self, msg):
    # Subscriber callback function that saves the x and y position and the x and z velocity from the /odom topic in a custom message and publishes it
    # Input msg: Message from the /odom topic
    
        # Create custom message for position and velocity information (from /odom)
        custom_msg = PosVel()
        custom_msg.x = msg.pose.pose.position.x
        custom_msg.y = msg.pose.pose.position.y
        custom_msg.vel_x = msg.twist.twist.linear.x
        custom_msg.vel_z = msg.twist.twist.angular.z

        # Publish the custom message
        self.posVel_publisher.publish(custom_msg)


    def feedback_callback(self, feedback):
    # Action callback function that displays the feedback from the action-server 
    # \input feedback: Feedback from the action-server
  
        # Print in the terminal the feedback (actual position and status)
        rospy.loginfo("\n" + str(feedback))
        

if __name__ == '__main__':
    
    try:
        # Initialize the action-client node
        rospy.init_node('action_client_node')
        
        # Create an object for the class ActionClient
        action_client = ActionClient()
        
        # Wait 2 seconds before user prompt
        rospy.sleep(2)
       
        # Prompt user for input target
        action_client.get_user_input()
        
        # Keep the script running until the node is shut down
        rospy.spin()
        
    except rospy.ROSInterruptException:
        # If for some issue the previous lines could't be executed, print this message:
        print("Program interrupted before completion", file=sys.stderr)
        
        
