#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <iostream>

#include <fstream>


class Controller{
private:
	typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	Controller(ros::NodeHandle &n) :
		node_(n),
		action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&Controller::goalCB, this, _1),
				boost::bind(&Controller::cancelCB, this, _1),
				false),
				has_active_goal_(false)
{
		creato=0;
		empty.linear.x=0;
		empty.linear.y=0;
		empty.linear.z=0;
		empty.angular.z=0;
		empty.angular.y=0;
		empty.angular.x=0;
		pub_topic = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		pub_topic2 = node_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/command/trajectory", 1);
		action_server_.start();
		ROS_INFO_STREAM("Node ready!");
}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Publisher pub_topic;
	ros::Publisher pub_topic2;
	geometry_msgs::Twist empty;
	geometry_msgs::Transform_<std::allocator<void> > lastPosition;
	geometry_msgs::Twist cmd;
	pthread_t trajectoryExecutor;
	int creato;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(creato){
				ROS_INFO_STREAM("Stop thread");
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(creato){
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;

		//controllore solo per il giunto virtuale Base
		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			creato=1;
			ROS_INFO_STREAM("Thread for trajectory execution created");
		} else {
			ROS_INFO_STREAM("Thread creation failed!");
		}

	}

	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void executeTrajectory(){
		if(toExecute.joint_names[0]=="virtual_joint" && toExecute.points.size()>0){

			std::ofstream log("/home/vkpankov/first_test.txt", std::ios_base::app | std::ios_base::out);

			for(int k=0; k<toExecute.points.size(); k++){
				geometry_msgs::Transform_<std::allocator<void> > point=toExecute.points[k].transforms[0];
				double des_roll, des_pitch, des_yaw;
				tf::Quaternion q;
  				tf::Matrix3x3(q).getRPY(des_roll, des_pitch, des_yaw);
				log << "0.5" << " " <<  point.translation.x << " " << point.translation.y << " " << point.translation.z
				<< " " << point.rotation.x << " " << point.rotation.y << " " << point.rotation.z << " " << point.rotation.w << std::endl << std::flush;
			}
			log.close();



                        pub_topic2.publish(toExecute);
                        ros::Duration(1.0).sleep();
 
		}
		active_goal_.setSucceeded();
		has_active_goal_=false;
		creato=0;

	}
	bool publishTranslationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool anyway){
		//creazione comando di traslazione
		cmd.linear.x=punto.translation.x-lastPosition.translation.x;
		cmd.linear.y=punto.translation.y-lastPosition.translation.y;
		cmd.linear.z=punto.translation.z-lastPosition.translation.z;
		cmd.angular.x=cmd.angular.y=cmd.angular.z=0;

		if(anyway || cmd.linear.x>=0.5 || cmd.linear.y>=0.5 || cmd.linear.z>=0.5){
			printPositionInfo();
			printCmdInfo();
			pub_topic.publish(cmd);
			//tempo d'esecuzione
			ros::Duration(1.0).sleep();
			return true;
		}
		return false;
	}

	void publishRotationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool start){
		//comando di allineamento, permesse solo rotazioni sull'asse z
		cmd.linear.x=cmd.linear.y=cmd.linear.z=cmd.angular.x=cmd.angular.y=0;
		//start = true --> devo tornare nell'orientazione 0
		//start = false --> devo arrivare al'orientazione punto.rotation.z
		cmd.angular.z=(start?0-punto.rotation.z:punto.rotation.z);

		printCmdInfo();

		double sleep=cmd.angular.z*3.0; //tempo necessario a tornare nella giusta orientazione
		if(sleep<0) sleep=-sleep;
		pub_topic.publish(cmd);
		ros::Duration(sleep).sleep();
		cmd.angular.z=0;
	}

	void printPositionInfo(){
		ROS_INFO_STREAM("Start Position: ["<<lastPosition.translation.x<<
				", "<<lastPosition.translation.y<<
				", "<<lastPosition.translation.z<<"] "<<
				"[ "<<lastPosition.rotation.x<<
				", "<<lastPosition.rotation.y<<
				", "<<lastPosition.rotation.z<<" ]");
	}

	void printCmdInfo(){
		ROS_INFO_STREAM("cmd to execute: "<<"x:"<<cmd.linear.x
				<<" y: " << cmd.linear.y
				<<" z: " << cmd.linear.z
				<<" rX: " << cmd.angular.x
				<<" rY: " << cmd.angular.y
				<<" rZ: " << cmd.angular.z);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_controller_node");
	ros::NodeHandle node;//("~");
	Controller control(node);

	ros::spin();

	return 0;
}
