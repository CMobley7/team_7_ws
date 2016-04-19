#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
# define M_PI       3.14159265358979323846  /* pi */
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;
int main(int argc, char **argv)
{

	  ros::init(argc, argv, "model_pose_controller");
	  ros::NodeHandle n;
	  ros::Publisher model_pose_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
	  int rate=10;
	  ros::Rate loop_rate(rate);
	  int t=0;
	  int t1=360*rate;
	  int t2=360*rate;
	  int t3=480*rate;
	  float v=4.16667;
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0;
		twist.angular.y = 0.0;
		twist.angular.z = 0;
		int stage=0;
		float percentage=0.5;
		float R=20;
	  while (ros::ok())
	  {
		  if(t%t3==t1) v=2.77778;
		  if(t%t3==t2) v=1.38889;
		  if(t%t3==0) v=4.16667;

		geometry_msgs::Pose pose;

		pose.position.z = 0;
		pose.orientation.x = 0;
		pose.orientation.y = 0;
		float theta_degree=0;

		if(percentage<1)
		{
			if((stage==0)||(stage==2))	percentage+=(float)v/rate/(2*R);
			if((stage==1)||(stage==3))	percentage+=(float)v/rate/(1.5*M_PI*R);
		}
		else
		{
			if((stage==0)||(stage==2))	percentage=(float)v/rate/(2*R);
			if((stage==1)||(stage==3))	percentage=(float)v/rate/(1.5*M_PI*R);

			stage=(stage+1)%4;
//			cout<<"t="<<t<<",stage="<<stage<<",percentage="<<percentage<<",theta="<<theta_degree<<",x="<<pose.position.x<<",y="<<pose.position.y<<endl;

		}


		if(stage==0)
		{
			pose.position.x =(percentage-0.5)/sqrt(2)*2*R;
			pose.position.y =(percentage-0.5)/sqrt(2)*2*R;
			theta_degree=45;
		}
		if(stage==1)
		{
			theta_degree=45-percentage*270;
			pose.position.x =(sqrt(2)-sin(theta_degree/180*M_PI))*R;
			pose.position.y =cos(theta_degree/180*M_PI)*R;
		}
		if(stage==2)
		{
			pose.position.x =-(percentage-0.5)/sqrt(2)*2*R;
			pose.position.y =(percentage-0.5)/sqrt(2)*2*R;
			theta_degree=45-270;
		}
		if(stage==3)
		{
			theta_degree=45-(1-percentage)*270;
			pose.position.x =-(sqrt(2)-sin(theta_degree/180*M_PI))*R;
			pose.position.y =-cos(theta_degree/180*M_PI)*R;
		}

		pose.position.z = 0.3;
		float theta=(float)theta_degree/2/180*M_PI;
		pose.orientation.z = sin(theta);
		pose.orientation.w=cos(theta);

		gazebo_msgs::ModelState modelstate;
		modelstate.model_name ="mobile_base";
		modelstate.pose = pose;
		modelstate.twist = twist;
		modelstate.reference_frame="world";

		model_pose_pub.publish(modelstate);




		cout<<"t="<<t<<",stage="<<stage<<",percentage="<<percentage<<",theta="<<theta_degree<<",x="<<pose.position.x<<",y="<<pose.position.y<<"v="<<v<<endl;

		ros::spinOnce();

		loop_rate.sleep();
		t++;


	  }
	  return 0;
}
 
