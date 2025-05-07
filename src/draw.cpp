/**
 * @file draw.cpp
 * @brief Implementation of the Draw class for drawing shapes with a turtlesim robot.
 * @author Alberto Bono 
 * @date 06/05/2025
 */
#include "turtlebot_controller/turtle.h"
#include "turtlebot_controller/draw.h"
#include <angles/angles.h>
/**
 * @brief Constructor: initializes the Draw class.
 * @param nh ROS NodeHandle reference.
 * @param turtle Reference to a Turtle object.
 * @param start_drawing_position Starting pose for drawing.
 * @param size Desired size of the shape.
 * @param number_of_shapes Total number of shapes to draw.
 */
Draw::Draw(ros::NodeHandle& nh, Turtle& turtle, turtlesim::Pose start_drawing_position, float size,int number_of_shapes): 
	nh_(nh),
	turtle_(turtle),
	start_drawing_position_(start_drawing_position),
	size_(size),
	number_of_shapes_(number_of_shapes)
{
}
/**
 * @brief Resets the internal state after completing a shape.
 */
void Draw::reset(){
	state_=ShapeState::INIT;
	theta_ = M_PI/6;
    circle_initialized_ = false;
}
/**
 * @brief Updates the size of the shape based on the number of shapes to draw.
 * @param l Reference to the size value to be adjusted.
 */
void Draw::UpdateSize(float l)
{
    // Check if the shape exceeds the vertical (y) space limit
    float occupied_space_y = l + start_drawing_position_.y;
    if (occupied_space_y > upper_limit_) {
        l = upper_limit_ - start_drawing_position_.y;
        ROS_WARN("Occupied vertical space (Y): %f. The height of the shape will be reduced to %f.", occupied_space_y, l);
    }

    // Check if the shape exceeds the horizontal (x) space limit
    float occupied_space_x = number_of_shapes_ * (l + space_) + space_ + start_drawing_position_.x;
    if (occupied_space_x >= upper_limit_) {
        l = (upper_limit_ - start_drawing_position_.x - space_ * (number_of_shapes_-1) ) / number_of_shapes_;
        ROS_WARN("Occupied horizontal space (X): %f. The width of the shape will be reduced to %f.", occupied_space_x, l);
    }

    size_ = l;
}
/**
 * @brief Draws a square.
 * @return True if the square is completed, false otherwise.
 */
bool Draw::square(){
	float& l = size_;
	if (count_==1) UpdateSize(l);	
	switch (state_)
    {
    	case ShapeState::INIT:
            waypoint_ = start_drawing_position_;
            
            if(turtle_.goto_position(waypoint_)){
            	count_++;
            	//ROS_INFO("Il robot deve raggiungere: [%f,%f].",waypoint_.x,waypoint_.y);
                state_ = ShapeState::vertice_0_1;
				waypoint_.y+=l;
            }
			return false;
        case ShapeState::vertice_0_1:
		     if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		        	state_ = ShapeState::vertice_1_1;
		            waypoint_.x+=l;
		        }
		    }  
            return false;

        case ShapeState::vertice_1_1:
        	 if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		         	state_ = ShapeState::vertice_1_0;
		            waypoint_.y-=l;
		        }
		    }    
            return false;

        case ShapeState::vertice_1_0:
		    if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		         	state_ = ShapeState::vertice_0_0;
		            waypoint_.x-=l;
		        }  
		    }
            return false;
        
        case ShapeState::vertice_0_0:
        	 if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		        	state_ = ShapeState::DONE;
		            start_drawing_position_=waypoint_;
		            start_drawing_position_.x+= l + space_;
		        }
		    }    
            return false;

        case ShapeState::DONE:
        	if (turtle_.goto_position(start_drawing_position_)){
        		reset();
        		return true;
        	}
            return false;
    }
	
	return false;
}
/**
 * @brief Draws a triangle.
 * @return True if the triangle is completed, false otherwise.
 */
bool Draw::triangle(){
	float& l = size_;
	if (count_==1) UpdateSize(l);	
	switch (state_)
    {
    	case ShapeState::INIT:
            waypoint_ = start_drawing_position_;
            
            if(turtle_.goto_position(waypoint_)){
            	count_++;
            	//ROS_INFO("Il robot deve raggiungere: [%f,%f].",waypoint_.x,waypoint_.y);
                state_ = ShapeState::vertice_05_1;
				waypoint_.y+=l;
				waypoint_.x+=l/2;
				
            }
			return false;
        case ShapeState::vertice_05_1:
		     if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		        	state_ = ShapeState::vertice_1_0;
		            waypoint_.x+=l/2;
		            waypoint_.y-=l;
		        }
		    }  
            return false;

        case ShapeState::vertice_1_0:
		    if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		         	state_ = ShapeState::vertice_0_0;
		            waypoint_.x-=l;
		        }  
		    }
            return false;
        
        case ShapeState::vertice_0_0:
        	 if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		        	state_ = ShapeState::DONE;
		            start_drawing_position_=waypoint_;
		            start_drawing_position_.x+= l + space_;
		        }
		    }    
            return false;

        case ShapeState::DONE:
        	if (turtle_.goto_position(start_drawing_position_)){
        		reset();
        		return true;
        	}
            return false;
    }
	
	return false;
}	
/**
 * @brief Draws a circle.
 * @return True if the circle is completed, false otherwise.
 */
bool Draw::circle(){
	float& l = size_;
	if (count_==1) UpdateSize(l);
		
	switch (state_)
    {
    	case ShapeState::INIT:
    		if (!circle_initialized_) {
    			count_++;
				start_drawing_position_.y += l / 2;
				waypoint_=start_drawing_position_;
				circle_initialized_ = true;
    		}

			//waypoint_ = start_drawing_position_;
			if(turtle_.goto_position(waypoint_)){
				//ROS_INFO("Il robot deve raggiungere: [%f,%f].",waypoint_.x,waypoint_.y);
				state_ = ShapeState::CIRCLE_MOTION;
			}
    		return false;
        case ShapeState::CIRCLE_MOTION:
        	if(theta_ >= 2 * M_PI) {
        		state_ = ShapeState::DONE;
        		start_drawing_position_=waypoint_;
		        start_drawing_position_.x+= l + space_;
		        start_drawing_position_.y-= l/2;
        		return false;
        	}
        	waypoint_.x = start_drawing_position_.x + l/2 - l/2 * cos(theta_);
            waypoint_.y = start_drawing_position_.y + l/2 * sin(theta_);
        		        	       	
		     if (turtle_.align(waypoint_)) {
		        if (turtle_.go_straight(waypoint_)){
		        	theta_ += M_PI/6.0;;
		        }
		    }  
            return false;
		case ShapeState::DONE:
            // opzionale: spostamento al punto finale
            reset();
            return true;
	}
	return false;

}

	




