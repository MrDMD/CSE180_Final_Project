#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>   
#include <ctime>

 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

nav_msgs::OccupancyGrid map_data;
std_msgs::Bool obj_Found;
void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg){
 map_data = *msg;
 return;

}

void obj_foundCB(const std_msgs::Bool::ConstPtr &msg){
 obj_Found = *msg;
 return;

}


int main(int argc, char**argv){
ros::init(argc, argv, "moverobo");
ros::NodeHandle nh;

ros::Subscriber map_sub = nh.subscribe("/map", 1000, &mapCB);
ros::Subscriber found_obj = nh.subscribe("obj_found", 1000, &obj_foundCB);
ros::Publisher reached_pose = nh.advertise<std_msgs::Bool>("reachpose", 1000);
 //tell the action client that we want to spin a thread by default
     MoveBaseClient ac("move_base", true);
   
     //wait for the action server to come up
     while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }
   
     move_base_msgs::MoveBaseGoal goal;
   
     //we'll send a goal to the robot to move 1 meter forward
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.header.stamp = ros::Time::now();
   
while (ros::ok()){
	ros::spinOnce();


	if(obj_Found.data == true) {
	//match the pose!	
	
	
	
	}

	for(int i = 0; i < map_data.data.size(); i++){
			
	
	}

	goal.target_pose.pose.position.x = 5;
	goal.target_pose.pose.position.y = 0;

     goal.target_pose.pose.position.x = rand() % 17 - 8;
     goal.target_pose.pose.position.y = rand() % 17 - 8;
     goal.target_pose.pose.orientation.w = 0.707;
     goal.target_pose.pose.orientation.z = 0.707;
     ROS_INFO("Sending goal");
     ac.sendGoal(goal);
   
     ac.waitForResult();
   
	}
    


}

