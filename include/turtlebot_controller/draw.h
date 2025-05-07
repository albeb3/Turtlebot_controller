/**
* @file draw.h
* @brief Header file defining the Draw class used to draw shapes with a turtlesim robot.
* @author Alberto Bono 
* @date 06/05/2025
**/
#ifndef DRAW_H
#define DRAW_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include "turtlebot_controller/turtle.h"
#include <string>

/**
 * @class Draw
 * @brief Class to draw geometric shapes with a turtle in the turtlesim environment.
 * 
 * This class provides functionalities to:
 *  - Draw a square. 
 *  - Draw a triangle.
 *  - Draw a circle.
 */
class Draw{
	public:
		/**
		 * @brief Constructor: initializes the Draw class.
		 * @param nh ROS NodeHandle reference.
		 * @param turtle Reference to a Turtle object.
		 * @param start_drawing_position Starting pose for drawing.
		 * @param size Desired size of the shape.
		 * @param number_of_shapes Total number of shapes to draw.
		 */
		Draw(ros::NodeHandle& nh, Turtle& turtle, turtlesim::Pose start_drawing_position, float size, int number_of_shapes);
		/**
		 * @brief Draws a square.
		 * @return True if the square is completed, false otherwise.
		 */
		bool square();
		/**
		 * @brief Draws a triangle.
		 * @return True if the triangle is completed, false otherwise.
		 */
		bool triangle();
		 /**
		 * @brief Draws a circle.
		 * @return True if the circle is completed, false otherwise.
		 */
		bool circle();
		
		
		
	private:
		/**
		 * @brief Resets the internal state after completing a shape.
		 */
		void reset();
		/**
		 * @brief Updates the size of the shape based on the number of shapes to draw.
		 * @param l Reference to the size value to be adjusted.
		 */
		void UpdateSize(float l);
		
		ros::NodeHandle nh_;
		Turtle& turtle_;
		turtlesim::Pose start_drawing_position_;
		float size_;
		int number_of_shapes_;
		int count_ = 1;
		float upper_limit_=10.5;
		float lower_limit_=0.5;
		float space_=0.5;
		float theta_=M_PI/6;
		bool circle_initialized_ = false;
		turtlesim::Pose waypoint_;
		/**
		 * @enum ShapeState
		 * @brief Represents the states of the finite state machine for drawing shapes.
		 *
     	 * These states correspond to different waypoints or steps during shape drawing.
		 */
		enum class ShapeState
    	{
    		INIT,
		    vertice_0_0,
		    vertice_05_1,
		    vertice_0_1,
		    vertice_1_0,
		    vertice_1_1,
		    CIRCLE_MOTION,
		    DONE
    	};
    	ShapeState state_ =ShapeState::INIT;
	
		
	
};

#endif // DRAW_H
	
