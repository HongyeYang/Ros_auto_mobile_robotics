//Hongye Yang
//publish odom_reset message & motion message
//subscribe prediction & goal message
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "ros_navigator_cmdline.h"
#include <math.h>
#include <iostream>
#include <fstream>

using namespace ros;
using namespace std;


ros::Time time_re;


void prediction2motion(const geometry_msgs::PoseWithCovariance::ConstPtr& msg1);
void goal_storing(const geometry_msgs::Pose::ConstPtr& msg2);

std_msgs::Empty ll;
geometry_msgs::Twist motion1;


double position[3]={0,0,0},goal[2]={0.35,0.25},v=0.1,mark=0,setang=0.2;

int
main(int argc, char** argv)
{
	cout<<"begin"<<endl;
	init(argc, argv,"talker");
	gengetopt_args_info args;
	cmdline_parser(argc,argv,&args);
	double v=args.linearVelocity_arg;
	
	motion1.angular.z=0.1644;
	NodeHandle n;
	Publisher pub=n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);

	NodeHandle m;
	Publisher pub2=m.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1000);

	pub2.publish(ll);
	ros::spinOnce();



	NodeHandle s;
	Subscriber predict=s.subscribe("/prediction",1000,prediction2motion);

	


	NodeHandle k;
	Subscriber sub=k.subscribe("/goal",1000,goal_storing);

		
	ros::Rate loop_rate(10);
	while(ros::ok() )
	{
	
		
		cout<<motion1.angular.z<<endl;
		//motion1.angular.z=0.1644;
		pub.publish(motion1);
		ros::spinOnce();
		
		loop_rate.sleep();
				
	}

	
	return 0;
} 


void prediction2motion(const geometry_msgs::PoseWithCovariance::ConstPtr& msg1)
{
//ros::Time time_ex=ros::Time::now();
//double tmdi=(time_ex-time_re).toSec();
if(goal[0]==100 &&goal[1]==100)
{
motion1.linear.x=0;
motion1.linear.y=0;
motion1.linear.z=0;

motion1.angular.x=0;
motion1.angular.y=0;
motion1.angular.z=0;
}
else
{
double z,yaw,w;
position[0]=msg1->pose.position.x;
position[1]=msg1->pose.position.y;
z=msg1->pose.orientation.z;
w=msg1->pose.orientation.w;
yaw=atan2(2*w*z,(1-2*z*z));

cout<<mark<<endl;
if(mark==0)
{
motion1.linear.x=v;
motion1.linear.y=0;
motion1.linear.z=0;

motion1.angular.x=0;
motion1.angular.y=0;

double dis;
dis=sqrt((position[0]-goal[0])*(position[0]-goal[0])+(position[1]-goal[1])*(position[1]-goal[1]));
double op,dp,gp;
op=atan2((goal[1]-position[1]),(goal[0]-position[0]));
dp=op-yaw;
gp=1.5708-dp;
motion1.angular.z=v*2*cos(gp)/dis;
}
else if(mark==1)
{
motion1.linear.x=0.00001;
motion1.angular.z=-setang;
}
else if(mark==2)
{
motion1.linear.x=0.00001;
motion1.angular.z=setang;
}
else if(mark==3)
{
motion1.linear.x=0.00001;
motion1.angular.z=setang;
}
else if(mark==4)
{
motion1.linear.x=0.00001;
motion1.angular.z=-setang;
}
}

//cout<<"dis="<<dis<<" op="<<op<<" dp="<<dp<<" gp="<<gp<<
//cout<<"x="<<position[0]<<"; y= "<<position[1]<<"; "<<" yaw="<<yaw<<" w="<<motion1.angular.z<<" goalx="<<goal[0]<<" goaly="<<goal[1]<<endl;//" "<<goal[0]<<" "<<goal[1]<<endl;

//cout<<v<<" "<<motion1.angular.z<<endl;
return;

}


void goal_storing(const geometry_msgs::Pose::ConstPtr& msg2)
{
goal[0]=msg2->position.x;
goal[1]=msg2->position.y;
mark=msg2->position.z;


return;
}







