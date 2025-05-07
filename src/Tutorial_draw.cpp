/**
 * @file Tutorial_turtle.cpp
 * @brief Tutorial using the Turtle class
 *
 * @author Alberto Bono 
 * @date 06/05/2025
 *
 * ServiceClient:<BR>
 *		/kill
 *
 * Description:<BR>
 * This tutorial demonstrates the use of a custom Draw class.
 * It uses the /kill service to remove the default turtle from the turtlesim_node.
 * A Turtle object for "turtle2" is instantiated, the pen color is set to red,
 * and the turtle is commanded to draw three shapes in sequence:
 *  - A circle
 *  - A triangle
 *  - A square
 * 
 * The movement logic is controlled via a state machine running in a loop at 10 Hz.
 */

#include "ros/ros.h"
#include "turtlebot_controller/turtle.h"
#include <turtlesim/Kill.h>
#include "turtlebot_controller/draw.h"


/**
 * @brief Main function that initializes the ROS node, removes the default turtle,
 * creates a Turtle object, defines a Draw object with goals and shape parameters,
 * and executes a state machine loop to control the turtle's behavior.
 */

int main(int argc, char **argv)
{
 	// Initialize the node, setup the NodeHandle for handling the communication
 	// with the ROS system.
	ros::init(argc,argv,"turtlebot_subscriber");
	ros::NodeHandle nh;
	
	// Create a ServiceClient to send the kill request for "turtle1"
	ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("kill");
	turtlesim::Kill kill_srv;
	kill_srv.request.name= "turtle1";
	kill_client.call(kill_srv);
	
	// Instantiate the Turtle object for "turtle2" and set the pen to red (255, 0, 0) with width 5
	Turtle t1(nh,"turtle2",5,5,0);
	t1.setPen(255, 0, 0, 5, false);
	
	// Define the target goal position
    turtlesim::Pose goal1;
    goal1.x = 0.5;
    goal1.y = 4;
    //Define the shape parameters
    float size_shape=7;
    int number_of_shape =3;
    
    // Instantiate the Draw object 
    Draw draw(nh,t1,goal1,size_shape,number_of_shape);
    
    // Initialize state variable
    int state=1;
    
    // Set the loop frequency at 10 Hz
    ros::Rate loop_rate(10);
    
	while (ros::ok()) 
	{
		switch (state)
    	{
    		case 1:
    			if (draw.circle()){
    				ROS_INFO("Circle completed.");
    				state++;
    			}
    			break;
    		case 2:	
    			if (draw.triangle()){
    				ROS_INFO("Triangle completed.");
    				state++;
    			}
    			break;
    		case 3:
    			if (draw.square()){
    				ROS_INFO("Square completed.");
    				state++;
    			}
    			break;
    	}
		ros::spinOnce(); //Process any incoming messages or callbacks
		loop_rate.sleep(); //Sleep to maintain loop rate
	}
	ros::spin();// Keeps the node alive to handle any remaining callbacks (blocks execution)
	return 0;
}
