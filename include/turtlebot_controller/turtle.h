/**
 * @file turtle.h
 * @brief Header file defining the Turtle class used to control a turtlesim robot.
 * @author Alberto Bono 
 * @date 05/05/2025
 */
#ifndef TURTLE_H
#define TURTLE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <actionlib/client/simple_action_client.h>
#include <turtlebot_controller/MoveAction.h>

#include <string>

/**
 * @class Turtle
 * @brief Class to control and manage a turtle in the turtlesim environment.
 * 
 * This class provides functionalities to:
 *  - Spawn a new turtle at a specified position.
 *  - Move the turtle to a target pose.
 *  - Align or move straight toward a goal.
 *  - Change the turtle's pen color or disable it.
 *  - Track the turtle's pose via subscription.
 */
class Turtle {
public:
    /**
     * @brief Constructor: creates a turtle at a given position and orientation.
     * @param nh ROS NodeHandle reference.
     * @param name The name of the turtle (e.g., "turtle1").
     * @param x X-coordinate for spawning the turtle.
     * @param y Y-coordinate for spawning the turtle.
     * @param theta Orientation angle in radians.
     */
    Turtle(ros::NodeHandle& nh, const std::string& name, float x, float y, float theta);

    /**
     * @brief Move the turtle to the specified position using an action client.
     * @param position The target position to reach.
     * @return True if movement was successful.
     */
    bool goto_position(turtlesim::Pose position);

    /**
     * @brief Rotate the turtle to face the desired direction.
     * @param position The target position to align to.
     * @return True if alignment was successful.
     */
    bool align(turtlesim::Pose position);

    /**
     * @brief Move the turtle in a straight line toward the given position.
     * @param position The target position to reach.
     * @return True if movement was successful.
     */
    bool go_straight(turtlesim::Pose position);

    /**
     * @brief Set or disable the turtle's pen.
     * @param r Red value (0-255).
     * @param g Green value (0-255).
     * @param b Blue value (0-255).
     * @param width Pen width.
     * @param off Set true to disable the pen, false to enable.
     */
    void setPen(uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off);

    /**
     * @brief Get the current pose of the turtle.
     * @return A turtlesim::Pose representing the current position and orientation.
     */
    turtlesim::Pose getPose() const;

private:
    /**
     * @brief Callback to update the current pose of the turtle.
     * @param msg The received pose message.
     */
    void poseCallback(const turtlesim::Pose::ConstPtr& msg);

    ros::NodeHandle nh_;                ///< ROS NodeHandle
    std::string name_;                  ///< Name of the turtle (e.g., "turtle1")
    ros::Publisher vel_pub_;            ///< Publisher to command turtle velocity
    ros::Subscriber pose_sub_;          ///< Subscriber to receive turtle pose
    ros::ServiceClient spawn_client_;   ///< Client to call the spawn service
    ros::ServiceClient pen_client_;     ///< Client to call the set_pen service
    turtlesim::Pose pose_;              ///< Current pose of the turtle
};

#endif
