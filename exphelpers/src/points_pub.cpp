#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
#include <cmath>

ros::Publisher points_pub;

int main(int argc, char **argv) {

	ros::init(argc, argv, "points_pub");

	ros::NodeHandle nh( ros::this_node::getName() );
	points_pub = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird/command/pose", 1000);

	std::ifstream wp_file("/home/vkpankov/first_test.txt"); 

	if (wp_file.is_open()) {

	double t, x, y, z, rx,ry,rz,rw;
	double prev_x=0,prev_y=0,prev_z=0,prev_rx=0,prev_ry=0,prev_rz=0, prev_rw;
	
	double desired_vel = 1;

	while (nh.ok() && wp_file >> t >> x >> y >> z >> rx >> ry >> rz >> rw) {

		geometry_msgs::PoseStamped pose;

		pose.header.frame_id="/hummingbird/base_link";
        	pose.header.stamp = ros::Time::now ();
		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.position.z = z;

		double dist =  std::sqrt(std::pow(x-prev_x,2) +
		 std::pow(y-prev_y,2) + std::pow(z-prev_z,2) +
		std::pow(rx-prev_rx,2)*2 + std::pow(ry-prev_ry,2)*2+ std::pow(rz-prev_rz,2)*2 + std::pow(rw-prev_rw,2)*2);
		

		prev_x = x; 
		prev_y = y;
		prev_z = z;

		prev_rx = rx; 
		prev_ry = ry;
		prev_rz = rz;
		prev_rw = rw;



		pose.pose.orientation.x = rx;
		pose.pose.orientation.y = ry;
		pose.pose.orientation.z = rz;
		pose.pose.orientation.w = rw;

		points_pub.publish(pose);

        	ros::spinOnce();
            	ros::Duration(((dist+0.05)/desired_vel)).sleep();
                

		
	}



	wp_file.close();
}

	ros::spin();


	return 0;
}
