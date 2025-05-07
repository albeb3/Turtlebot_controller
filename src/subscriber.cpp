#include "ros/ros.h"
#include "turtlebot_controller/turtle.h"
#include <turtlesim/Kill.h>
#include <actionlib/client/simple_action_client.h>
#include <turtlebot_controller/MoveAction.h>
/*
class Draw{
	private:
		Turtle& turtle_;
		
	public:
		Draw(Turtle& turtle):turtle_(turtle){}
		
		
		
    	void go_straight(){
			my_vel_.linear.y=0.0;
			if (std::abs(getAngleError()) < th_ang_)
				my_vel.linear.x = 1.0;
			else
				my_vel.linear.x = 0.0;
		}
		void alignment(double desired_theta){
			double e_theta= normalize_angle(desired_theta - g_pose.theta);
			if (e_theta > 0)  my_vel.angular.z = Kp * e_theta;
			else my_vel.angular.z = -Kp * e_theta;
			go_straight();
			
		}
		bool goto_target(float x, float y) {
			ROS_INFO("Debugging: goto_target@[%f,%f]", x,y);
			
			if (g_pose.x>x+th_lin) { alignment(M_PI);return false; }
			else if (g_pose.x<x-th_lin) {alignment(0);return false;}
			else if (g_pose.y>y+th_lin) {alignment(-M_PI/2);return false;}
			else if (g_pose.y<y-th_lin) {alignment(M_PI/2);return false;}
			else {
				my_vel_ = geometry_msgs::Twist();
				return true;
			}	
		}
		void moveLogic(){
			switch (count){
				case 1:
					if(goto_target(1,1))count++;
					break;
				case 2:
					if(goto_target(10,1))count++;
					break;
				case 3: 
					if(goto_target(10,10))count++;
					break;
				case 4:
					if(goto_target(1,10))count++;
					break;
				case 5:
					if(goto_target(1,5))count++;
					break;
				default:
					my_vel_ = geometry_msgs::Twist();
					break;
			}
		}
		void updatePen() {
			if ((std::abs(g_pose.x-1)<= th_lin && (std::abs(g_pose.y)<= 10 + 2*th_lin || std::abs(g_pose.y)>= 1 - 2*th_lin) ) ||
				 (std::abs(g_pose.x-10)<= th_lin && (std::abs(g_pose.y)<= 10 + 2*th_lin || std::abs(g_pose.y)>= 1 - 2*th_lin))) {
				setPen(0,255,0,0,10);
			}
			else{
				setPen(1);
			}
		}
	

		double normalize_angle(double angle){
			while (angle > PI) angle -= 2 * PI;
			while (angle < -PI) angle += 2 * PI;
			return angle;
		}

}

*/


int main(int argc, char **argv)
{
 
	ros::init(argc,argv,"turtlebot_subscriber");
	ros::NodeHandle nh;
	ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("kill");
	turtlesim::Kill kill_srv;
	kill_srv.request.name= "turtle1";
	kill_client.call(kill_srv);
	
	
	Turtle t1(nh,"turtle2",5,5,0);
	t1.setPen(255, 0, 0, 5, false);
	/*
	actionlib::SimpleActionClient<turtlebot_controller::MoveAction> ac("move", true);
    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer(); // blocca finché il server non è pronto
    ROS_INFO("Action server started, sending goal...");
	*/
    
    
    
    /*
    turtlebot_controller::MoveGoal goal;
    goal.topic = "turtle2";
    goal.x = 11.0;   // imposta qui le coordinate desiderate
    goal.y = 11.0;
    
    ac.sendGoal(goal);
    */
    turtlesim::Pose goal2;
    goal2.x = 1;
    goal2.y = 1;
    
    
    ros::Rate loop_rate(10);
	while (ros::ok()) {
	//t1.goto_position(goal2);
	if(t1.align(goal2)){
	t1.go_straight(goal2);
	}
	ros::spinOnce();
	loop_rate.sleep();
	}
    
    /*
    ros::Rate loop_rate(10);
	while (ros::ok()) {
	
	t1.go_straight(10,5);
	ros::spinOnce();
	loop_rate.sleep();
	}
	*/
	
	
	//Draw draw;
	ros::spin();
	return 0;

}
