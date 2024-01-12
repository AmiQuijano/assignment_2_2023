#!/usr/bin/env python3

# Ami Sofia Quijano Shimizu
# Reseach Track 1 - Assignment 2 b)

# Instructions: Create a service node that, when called, returns the coordinates of the last target sent by the user. 


# Useful imports
import rospy
from assignment_2_2023.srv import GetLastTarget, GetLastTargetResponse  # Import the custom service message
from assignment_2_2023.msg import PlanningActionGoal # Import the message type from /reaching_goal/goal topic


class GetLastTargetService:

    def __init__(self):
    # Initialization function for the ROS service, subscriber and variable as required
    
        # Variable to store the last target coordinates
        self.last_target = []

        # Create a service named 'get_last_target' with the custom service message type
        self.service = rospy.Service('get_last_target', GetLastTarget, self.handle_get_last_target)

        # Subscribe to the '/reaching_goal/goal' topic to get the last target coordinates
        self.target_subscriber = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.target_callback)
    

    def target_callback(self, msg):
    # Subscriber callback function to update the last_target variable when a new target is received in the /reaching_goal/goal topic
        
        # Save in last_target variable the x and y position message from /reaching_goal/goal topic
        if self.last_target == []:
            self.last_target.append(msg.goal.target_pose.pose.position.x)
            self.last_target.append(msg.goal.target_pose.pose.position.y)
        
        else:
            self.last_target[0] = msg.goal.target_pose.pose.position.x
            self.last_target[1] = msg.goal.target_pose.pose.position.y


    def handle_get_last_target(self, req):
    # Service callback function to handle requests for the last target coordinates
        
        # Create a response object using the custom service message type
        response = GetLastTargetResponse()  
        
        # If a last target exists, populate the response fields with its coordinates and the success confirmation  
        if self.last_target:  
            response.x = self.last_target[0]
            response.y = self.last_target[1]
            response.success = True
        
        # If no last target exists, set the success field to False
        else:
            response.success = False
        
        # Return the response to the service caller
        return response  
        

if __name__ == '__main__':

    try:
        # Initialize the ROS node
        rospy.init_node('get_last_target_server')
    
        # Instantiate the GetLastTargetServer class and start the ROS node
        get_last_target_service = GetLastTargetService()
    
        # Keep the script running until the node is shut down
        rospy.spin()  
    
    except rospy.ROSInterruptException:
        # If for some issue the previous lines could't be executed, print this message:
        print("Program interrupted before completion", file=sys.stderr)
    

