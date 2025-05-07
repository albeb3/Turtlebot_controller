/**
* @file move_action_server.cpp
* @brief ROS Action Server for controlling a turtle in turtlesim.
* @author Alberto Bono 
* @date 05/05/2025
*
* Topics:
 *  - Subscribed: /<turtle_name>/pose
 *  - Published:  /<turtle_name>/cmd_vel
 * 
 * Action:
 *  - Move.action: defines the goal (x, y, task_type), feedback, and result.
*
* Description:<BR>
* This node implements an action server that handles two motion tasks:
 * - "align": rotate the turtle to face a goal position.
 * - "go_straight": move the turtle in a straight line toward the goal.
 * 
 * The server provides real-time feedback on angular or positional error and
 * reports success once the turtle reaches the desired orientation or position.
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_controller/MoveAction.h>
#include <cmath>
#include <turtlesim/Pose.h>
#include "turtlebot_controller/turtle.h"

class Move
{
	protected:
	
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<turtlebot_controller::MoveAction> as_;  ///< The action server
	std::string action_name_;											  ///< Name of the action
	turtlebot_controller::MoveFeedback feedback_;						  ///< Feedback to the client
	turtlebot_controller::MoveResult result_;							  ///< Final result sent to the client
	turtlesim::Pose current_pose_;										  ///< Current pose of the turtle
	geometry_msgs::Twist vel_;											  ///< Velocity command to be published
	ros::Publisher vel_pub_;											  ///< Publisher for velocity commands
	ros::Subscriber pose_sub_;											  ///< Subscriber for turtle pose

	public:
	/**
     * @brief Constructor for the Move action server.
     * @param action_name Name of the action, used to create the server.
     */
	Move(std::string action_name)  
		: as_(nh_, action_name, boost::bind(&Move::executeCB, this, _1), false),
		  action_name_(action_name)
	{
		as_.start(); // Start the action server
	}
	~Move(void){}
	/**
     * @brief Callback function executed when a goal is received.
     * @param goal Pointer to the received goal containing x, y, turtle_name and task_type.
     */
	void executeCB(const turtlebot_controller::MoveGoalConstPtr &goal)
	{
		ros::Rate r(10); // Loop at 10 Hz
		std::string turtle_name = goal->turtle_name;
		vel_pub_ =nh_.advertise<geometry_msgs::Twist>("/"+ turtle_name + "/cmd_vel",10);
		pose_sub_=nh_.subscribe("/"+turtle_name+"/pose",10,&Move::poseCB,this);
		bool success = true;
		float x_goal= goal->x;
		float y_goal= goal->y;
    	ROS_INFO("%s: Executing, %s action is executed to the goal [%f,%f] ", action_name_.c_str(),goal->task_type.c_str(), goal->x,goal->y );
    	// Initialize velocity
    	vel_.linear.x=0.0;
    	vel_.angular.z=0.0;
    	float Kp=0.0;
    	float error_theta = 99.0;  // Placeholder large error
    	float error_position = 99.0;
		while (ros::ok()) 
		{
			float x = current_pose_.x;
			float y = current_pose_.y;
			float theta = current_pose_.theta;
			if(goal->task_type == "align")
			{
				float desired_theta = std::atan2( y_goal - y , x_goal - x );
				error_theta = desired_theta - theta;	
				// Normalize angle to [-pi, pi]
				while (error_theta > M_PI) error_theta -= 2 * M_PI;
		        while (error_theta < -M_PI) error_theta += 2 * M_PI;  
		        Kp = 2.0;
				vel_.angular.z = Kp * error_theta;
				feedback_.error_theta=error_theta;
				as_.publishFeedback(feedback_);
			}
			else if (goal->task_type == "go_straight")
			{
				float dx=x_goal - x;
				float dy=y_goal - y;
				error_position= std::sqrt(dx*dx + dy*dy);	
				Kp = 1.0;
				vel_.linear.x = Kp * error_position;
				feedback_.error_theta=error_position;
				as_.publishFeedback(feedback_);
			}
			// Check for preemption
			if (as_.isPreemptRequested() || !ros::ok())
		 	{
		 		ROS_INFO("%s: Preempted", action_name_.c_str());
		 		// set the action state to preempted
		 		as_.setPreempted();
		 		success = false;
		 		break;
		    }
		    // Goal condition met
		    if (std::abs(error_theta) < 0.025 || std::abs(error_position)<0.05) break;
			// Publish velocity command
			vel_pub_.publish(vel_);	
			r.sleep();	
		}
    	if(success)
    	{
    		result_.result = true;
    		// publish info to the console for the user
    		ROS_INFO("%s: Succeeded", action_name_.c_str());
    		as_.setSucceeded(result_);
    	}
    }
    /**
     * @brief Callback to update the current turtle pose.
     * @param msg The current pose message.
     */
    void poseCB(const turtlesim::Pose::ConstPtr& msg)
    {
      current_pose_=*msg;
    }
};
/**
 * @brief Main function: initializes the ROS node and starts the Move action server.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move"); // Initialize the node with name "move"
  Move move("move");			 // Start the action server
  ros::spin();					 // Keep the node alive and processing callbacks
  return 0;
}
