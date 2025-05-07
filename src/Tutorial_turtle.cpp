/**
 * @file Tutorial_turtle.cpp
 * @brief Tutorial using the Turtle class
 * @author Alberto Bono 
 * @date 05/05/2025
 *
 * ServiceClient:<BR>
 *		/kill
 *
 * Description:<BR>
 * This tutorial demonstrates the use of a custom Turtle class.
 * The Kill service is used to remove the default turtle from the turtlesim_node.
 * A Turtle object for "turtle2" is instantiated.
 * The pen color is set to red with a width of 5 using: setPen(red, green, blue, width, pen_off).
 * Two goal positions are defined. 
 * A loop at 10 Hz executes the movement logic, which includes:
 *  - Moving to the first goal
 *  - Aligning to the second goal
 *  - Moving straight toward the second goal
 */

#include "ros/ros.h"
#include "turtlebot_controller/turtle.h"
#include <turtlesim/Kill.h>
/**
 * @enum TurtleState
 * @brief Represents the finite states of the turtle's behavior.
 *
 * This enum defines meaningful names for each stage of the turtle's movement sequence:
 * - GOTO_GOAL1: Move toward the first target position.
 * - ALIGN_TO_GOAL2: Rotate to face the direction of the second goal.
 * - MOVE_TO_GOAL2: Move straight toward the second goal after alignment.
 * - DONE: Final state where no further movement is performed.
 */
enum class TurtleState
{
    GOTO_GOAL1,
    ALIGN_TO_GOAL2,
    MOVE_TO_GOAL2,
    DONE
};
// Initialize the current state to start with moving toward the first goal
TurtleState current_state = TurtleState::GOTO_GOAL1;
/**		
 * @brief Handles the robot movement states
 * @param turtle A Turtle object used to control the robot
 * @param goal1 The first goal position
 * @param goal2 The second goal position
 */
void moveLogic(Turtle& turtle,turtlesim::Pose& goal1,turtlesim::Pose& goal2)
{
	switch (current_state)
    {
        case TurtleState::GOTO_GOAL1:
            if (turtle.goto_position(goal1))
                current_state = TurtleState::ALIGN_TO_GOAL2;
            break;

        case TurtleState::ALIGN_TO_GOAL2:
            if (turtle.align(goal2))
                current_state = TurtleState::MOVE_TO_GOAL2;
            break;

        case TurtleState::MOVE_TO_GOAL2:
            if (turtle.go_straight(goal2))
                current_state = TurtleState::DONE;
            break;

        case TurtleState::DONE:
            // No more actions; could add stop logic or goal reached message
            break;
    }
}
/**
 * @brief Main function that initializes the ROS node, removes the default turtle,
 * creates a Turtle object, defines goals, and executes the state machine loop
 * to control the turtle's behavior.
 */

int main(int argc, char **argv)
{
 	//Initialize the node, setup the NodeHandle for handling the communication
 	//with the ROS system.
	ros::init(argc,argv,"turtlebot_subscriber");
	ros::NodeHandle nh;
	
	//Create a ServiceClient to send the kill request for "turtle1"
	ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("kill");
	turtlesim::Kill kill_srv;
	kill_srv.request.name= "turtle1";
	kill_client.call(kill_srv);
	
	//Instantiate the Turtle object for "turtle2" and set the pen to red (255, 0, 0) with width 5
	Turtle t1(nh,"turtle2",5,5,0);
	t1.setPen(255, 0, 0, 5, false);
	
	//Define the target goal position
    turtlesim::Pose goal1,goal2;
    goal1.x = 1;
    goal1.y = 1;
    goal2.x = 5;
    goal2.y = 5;
    
    //Set the loop frequency at 10 Hz
    ros::Rate loop_rate(10);
	while (ros::ok()) 
	{
		moveLogic(t1,goal1,goal2); //executes the movement logic
		ros::spinOnce(); //Process any incoming messages or callbacks
		loop_rate.sleep(); //Sleep to maintain loop rate
	}
	ros::spin();// Keeps the node alive to handle any remaining callbacks (blocks execution)
	return 0;
}
