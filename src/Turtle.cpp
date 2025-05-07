/**
 * @file turtle.cpp
 * @brief Implementation of the Turtle class for controlling the robot in the turtlesim environment.
 * @author Alberto Bono 
 * @date 05/05/2025
 */
#include "turtlebot_controller/turtle.h"
#include <angles/angles.h>
/**
 * @brief Constructor: creates a turtle at a given position and orientation.
 * @param nh ROS NodeHandle reference.
 * @param name The name of the turtle (e.g., "turtle1").
 * @param x X-coordinate for spawning the turtle.
 * @param y Y-coordinate for spawning the turtle.
 * @param theta Orientation angle in radians.
 */
Turtle::Turtle(ros::NodeHandle& nh, const std::string& name, float x, float y, float theta): nh_(nh),name_(name){
	spawn_client_ = nh_.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn spawn_srv;
	spawn_srv.request.x=x;
	spawn_srv.request.y=y;
	spawn_srv.request.theta=theta;
	spawn_srv.request.name = name_;
	spawn_client_.call(spawn_srv);
	// Publisher to send velocity commands to the turtle
	vel_pub_ =nh.advertise<geometry_msgs::Twist>("/"+ name_ + "/cmd_vel",10);
	// Subscriber to listen to the turtle's pose
	pose_sub_=nh.subscribe("/"+name_+"/pose",10,&Turtle::poseCallback,this);
	// Service client to set the pen for the turtle
	pen_client_ = nh_.serviceClient<turtlesim::SetPen>("/"+ name_+"/set_pen");
}

/**
 * @brief Move the turtle to the specified position using an action client.
 * @param position The target position to reach.
 * @return True if movement was successful.
 */
bool Turtle::goto_position(turtlesim::Pose position){
	setPen(255, 0, 0, 5, true);
	geometry_msgs::Twist vel;
	float dx=position.x-getPose().x;
	float dy=position.y-getPose().y;
	float theta =pose_.theta;
	float desired_theta = std::atan2( dy , dx );
	// Compute the errors in position and orientation
	float error_position= std::sqrt(dx*dx + dy*dy);
	float error_theta = desired_theta - theta;
	// Normalize the angle error to the range [-pi, pi]
	while (error_theta > M_PI) error_theta -= 2 * M_PI;
    while (error_theta < -M_PI) error_theta += 2 * M_PI;
	
	float Kp_lin = 1.0;
	float Kp_ang = 2.0;
	vel.linear.x = Kp_lin*error_position;
	vel.angular.z = Kp_ang * error_theta;
	vel_pub_.publish(vel);
	if (std::abs(error_position)<=0.1)
	{
		setPen(255, 0, 0, 5, false);
		ROS_INFO("Il robot ha raggiunto correttamente l'obiettivo.");
		return true;
	}
	else
	{
      	return false;
  	}
}
/**
 * @brief Rotate the turtle to face the desired direction.
 * @param position The target position to align to.
 * @return True if alignment was successful.
 */
bool Turtle::align(turtlesim::Pose position){
	actionlib::SimpleActionClient<turtlebot_controller::MoveAction> ac("move", true);
    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer(); // blocca finché il server non è pronto
    ROS_INFO("Action server started, sending goal...");
    turtlebot_controller::MoveGoal goal;
    goal.turtle_name = name_;
    goal.task_type = "align";
    goal.x = position.x;   // imposta qui le coordinate desiderate
    goal.y = position.y;
    
    ac.sendGoal(goal);
    
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0)); // timeout opzionale
	if (finished_before_timeout){
    	turtlebot_controller::MoveResultConstPtr result = ac.getResult();
    	if (result->result){
    		ROS_INFO("Il robot ha raggiunto correttamente l'obiettivo.");
    		return true;
    	}
    	else{
      		//ROS_WARN("Il robot NON ha raggiunto l'obiettivo.");
      		return false;
  		}
  	}
  	else {
  		ROS_ERROR("Timeout scaduto prima di ricevere un risultato.");
  		return false;
    }	
}
/**
 * @brief Move the turtle in a straight line toward the given position.
 * @param position The target position to reach.
 * @return True if movement was successful.
 */
bool Turtle::go_straight(turtlesim::Pose position){
	actionlib::SimpleActionClient<turtlebot_controller::MoveAction> ac("move", true);
    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer(); // blocca finché il server non è pronto
    ROS_INFO("Action server started, sending goal...");
    turtlebot_controller::MoveGoal goal;
    goal.turtle_name = name_;
    goal.task_type = "go_straight";
    goal.x = position.x;   // imposta qui le coordinate desiderate
    goal.y = position.y;
    
    ac.sendGoal(goal);
    
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0)); // timeout opzionale
	if (finished_before_timeout){
    	turtlebot_controller::MoveResultConstPtr result = ac.getResult();
    	if (result->result){
    		ROS_INFO("Il robot ha raggiunto correttamente l'obiettivo.");
    		return true;
    	}
    	else{
      		//ROS_WARN("Il robot NON ha raggiunto l'obiettivo.");
      		return false;
  		}
  	}
  	else {
  		ROS_ERROR("Timeout scaduto prima di ricevere un risultato.");
  		return false;
    }	
}
/**
 * @brief Set or disable the turtle's pen.
 * @param r Red value (0-255).
 * @param g Green value (0-255).
 * @param b Blue value (0-255).
 * @param width Pen width.
 * @param off Set true to disable the pen, false to enable.
 */
void Turtle::setPen(uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off){
	turtlesim::SetPen pen_srv;
	pen_srv.request.r = r;
	pen_srv.request.g = g;
	pen_srv.request.b = b;
	pen_srv.request.width = width;
	pen_srv.request.off = off;
	pen_client_.call(pen_srv);
}
/**
 * @brief Get the current pose of the turtle.
 * @return A turtlesim::Pose representing the current position and orientation.
 */
turtlesim::Pose Turtle::getPose() const {
	return pose_;
}
/**
 * @brief Callback to update the current pose of the turtle.
 * @param msg The received pose message.
 */			
void Turtle::poseCallback(const turtlesim::Pose::ConstPtr& msg){	
	pose_=*msg;
}
