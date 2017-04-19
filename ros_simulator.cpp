//Hongye Yang
//publish position message & scan message
//receive odom_reset message & motion message
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "ros_simulator_cmdline.h"
#include <iostream>
#include <cstdlib>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <fstream>




using namespace std;
using namespace ros;

ofstream labdata2;

nav_msgs::Odometry pose1;
ros::Time t1,t2;
double tmdi,yaw,yaw1,angle_max,angle_min,reslt,bear1,bear2,bear3,bear4,bear5,bear6,margin1,margin2,margin3,margin4,margin5,margin6,prerange,ff,ww,var;
sensor_msgs::LaserScan measure1;
int ii,i,r;

void resetCallback(const std_msgs::Empty& msg1);

void poseCallback(const geometry_msgs::Twist::ConstPtr& msg);

void measuredo();

int sgn(double);
double errorv=1,errorw=1;



int main(int argc, char** argv)
	{
	cout<<"begin"<<endl;
//initial value
	yaw=0;
	pose1.pose.pose.position.x=0;
	pose1.pose.pose.position.y=0;
	pose1.pose.pose.position.z=0;
	pose1.pose.pose.orientation.x=0;
	pose1.pose.pose.orientation.y=0;
	pose1.pose.pose.orientation.z=0;
	pose1.pose.pose.orientation.w=1;

	init(argc, argv,"listener");

	gengetopt_args_info args;
	cmdline_parser(argc,argv,&args);
	
	labdata2.open("lab56_actual_pose.txt");
	
	double amax=args.angle_max_arg;
	double amin=args.angle_min_arg;
	double t=args.time_arg;
	r=args.resolution_arg;
        var=args.variance_arg;
	double tstep=args.time_step_arg;
	measure1.angle_min=amin;
	measure1.angle_max=amax;
	measure1.time_increment=tstep;
	measure1.scan_time=t;
	measure1.ranges[r];
	measure1.angle_increment=(amax-amin)/r;
	


	
	NodeHandle n;
	Subscriber subreset=n.subscribe("/mobile_base/commands/reset_odometry",1000,resetCallback);

	
	NodeHandle m;
        t1=ros::Time::now();
	t2=ros::Time::now();

	Subscriber sub=m.subscribe("/cmd_vel_mux/input/navi",1000,poseCallback);
	

	 NodeHandle k;
	 Publisher pub=k.advertise<nav_msgs::Odometry>("/odom",1000);
	
	NodeHandle z;
	Publisher pubmeasure=z.advertise<sensor_msgs::LaserScan>("/scan",1000);
        
	ros::Rate loop_rate(10);
	while(ros::ok() )
	{
	  
		  pub.publish(pose1);
		  ros::spinOnce();
		measuredo();
		pubmeasure.publish(measure1);
		
	ros::spinOnce();

		
	loop_rate.sleep();
	}
	return 0;
	}

void resetCallback(const std_msgs::Empty& msg1)
{
	pose1.pose.pose.position.x=0;
	pose1.pose.pose.position.y=0;
	pose1.pose.pose.position.z=0;
	pose1.pose.pose.orientation.x=0;
	pose1.pose.pose.orientation.y=0;
	pose1.pose.pose.orientation.z=0;
	pose1.pose.pose.orientation.w=1;

       return;
};

void poseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	t1=ros::Time::now();
	tmdi=(t1-t2).toSec();
        if(tmdi>0.5)tmdi=0.1;
	t2=t1;
	  double v,w,v1,w1,b,s1=0,s2=0,s3=0,s4=0,a1=0.01,a2=0.01,a3=0.01,a4=0.01,a5=0.01,a6=0.01;

	v=msg->linear.x;
	w=msg->angular.z;

	double xx=pose1.pose.pose.position.x;
	double yy=pose1.pose.pose.position.y;

	b=a1*fabs(v)+a2*fabs(w);
	for(i=0;i<13;i++)
		{
		s1=s1+(rand()/(RAND_MAX + 1.0))*b*2-b;
		}
	s1=s1/2;
	b=a3*fabs(v)+a4*fabs(w);
	for(i=0;i<13;i++)
		{
		s2=s2+(rand()/(RAND_MAX + 1.0))*b*2-b;
		}
	s2=s2/2;
	b=a5*fabs(v)+a6*fabs(w);
	for(i=0;i<13;i++)
		{
		s3=s3+(rand()/(RAND_MAX + 1.0))*b*2-b;
		}
	s3=s3/2;

	v1=v+s1*errorv;
	w1=w+s2*errorw;
/*cout<<"b"<<" "<<pose1.pose.pose.position.x<<" "<<pose1.pose.pose.position.y<<endl;
cout<<"xx="<<xx<<" "<<"v1="<<v1<<" "<<"w1="<<w1<<" "<<"yaw="<<yaw<<"tmdi="<<tmdi<<endl;*/
	pose1.pose.pose.position.x=xx-v1/w1*sin(yaw)+v1/w1*sin(yaw+w1*tmdi);
	pose1.pose.pose.position.y=yy+v1/w1*cos(yaw)-v1/w1*cos(yaw+w1*tmdi);
	
	yaw=yaw+w1*tmdi+s3*tmdi;
	pose1.pose.pose.orientation.x=0;
	pose1.pose.pose.orientation.y=0;
	pose1.pose.pose.orientation.z=sin(0.5*yaw);
	pose1.pose.pose.orientation.w=cos(0.5*yaw);
     
	labdata2<<pose1.pose.pose.position.x<<", "<<pose1.pose.pose.position.y<<","<<yaw<<";"<<endl;

	return;
}

int sgn(double x)
{
	if(x>0)
	return 1;
	else if(x<0)
	return -1;
	else
	return 0;
}



void measuredo(void)
{
double s4=0;
vector<float>ranges(640, 0);
	
	for(ii=0;ii<=640;ii++)
		{
			

		yaw1=atan2((2*pose1.pose.pose.orientation.w*pose1.pose.pose.orientation.z),(1-2*pose1.pose.pose.orientation.z*pose1.pose.pose.orientation.z));

	bear1 = atan2( (0 -pose1.pose.pose. position.y),(1 - pose1.pose.pose.position.x)) - yaw1;
	bear2 = atan2( (0 -pose1.pose.pose. position.y),(2 - pose1.pose.pose.position.x)) - yaw1;
	bear3 = atan2( (0 -pose1.pose.pose. position.y),(3 - pose1.pose.pose.position.x)) - yaw1;
	bear4 = atan2( (0 -pose1.pose.pose. position.y),(5 - pose1.pose.pose.position.x)) - yaw1;
	bear5 = atan2( (0 -pose1.pose.pose. position.y),(6 - pose1.pose.pose.position.x)) - yaw1;
	bear6 = atan2( (0 -pose1.pose.pose. position.y),(7 - pose1.pose.pose.position.x)) - yaw1;	
//if(ii==32)cout<<bear1<<endl;
	if (bear1 < -3.14159)bear1= bear1 + 3.14159*2;
	if (bear2 < -3.14159)bear2= bear2 + 3.14159*2;
	if (bear3 < -3.14159)bear3= bear3 + 3.14159*2;
	if (bear4 < -3.14159)bear4= bear4 + 3.14159*2;
	if (bear5 < -3.14159)bear5= bear5 + 3.14159*2;
	if (bear6 < -3.14159)bear6= bear6 + 3.14159*2;

	if (bear1 >  3.14159)bear1= bear1 - 3.14159*2;
	if (bear2 >  3.14159)bear2= bear2 - 3.14159*2;
	if (bear3 >  3.14159)bear3= bear3 - 3.14159*2;
	if (bear4 >  3.14159)bear4= bear4 - 3.14159*2;
	if (bear5 >  3.14159)bear5= bear5 - 3.14159*2;
	if (bear6 >  3.14159)bear6= bear6 - 3.14159*2;

	if (bear1 < -9.4247778)bear1= bear1 + 3.14159*4;
	if (bear2 < -9.4247778)bear2= bear2 + 3.14159*4;
	if (bear3 < -9.4247778)bear3= bear3 + 3.14159*4;
	if (bear4 < -9.4247778)bear4= bear4 + 3.14159*4;
	if (bear5 < -9.4247778)bear5= bear5 + 3.14159*4;
	if (bear6 < -9.4247778)bear6= bear6 + 3.14159*4;


	margin1=asin(0.057/sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((1-pose1.pose.pose.position.x),2)));
	margin2=asin(0.057/sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((2-pose1.pose.pose.position.x),2)));
	margin3=asin(0.057/sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((3-pose1.pose.pose.position.x),2)));
	margin4=asin(0.057/sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((5-pose1.pose.pose.position.x),2)));
	margin5=asin(0.057/sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((6-pose1.pose.pose.position.x),2)));
	margin6=asin(0.057/sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((7-pose1.pose.pose.position.x),2)));
double rand_n;
	if(ii*measure1.angle_increment+measure1.angle_min>bear1-margin1 && ii*measure1.angle_increment+measure1.angle_min<bear1+margin1)	
		{
			if(ii*measure1.angle_increment+measure1.angle_min>bear1-margin1*0.65 && ii*measure1.angle_increment+measure1.angle_min<bear1+margin1*0.65)
			{
				
				rand_n=rand()/(RAND_MAX + 1.0);
				if(rand_n<0.8)
				{
					ff=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((1-pose1.pose.pose.position.x),2))*cos(measure1.angle_min+ii*measure1.angle_increment-bear1);
					ww=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((1-pose1.pose.pose.position.x),2))*sin(measure1.angle_min+ii*measure1.angle_increment-bear1);
					prerange=abs(ff-(sqrt(0.057*0.057-pow(ww,2))));
				}
				else
					prerange=sqrt(-1);
			}
			else
			prerange=sqrt(-1);
		}
	else if(ii*measure1.angle_increment+measure1.angle_min>bear2-margin2 && ii*measure1.angle_increment+measure1.angle_min<bear2+margin2)
	{
		if(ii*measure1.angle_increment+measure1.angle_min>bear2-margin2*0.65 && ii*measure1.angle_increment+measure1.angle_min<bear2+margin2*0.65)
			{
				
				rand_n=rand()/(RAND_MAX + 1.0);
				if(rand_n<0.8)
				{		
		ff=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((2-pose1.pose.pose.position.x),2))*cos(measure1.angle_min+ii*measure1.angle_increment-bear2);
		ww=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((2-pose1.pose.pose.position.x),2))*sin(measure1.angle_min+ii*measure1.angle_increment-bear2);
		prerange=abs(ff-(sqrt(0.057*0.057-pow(ww,2))));	
				}
				else
					prerange=sqrt(-1);
			}
			else
			prerange=sqrt(-1);
	}

	else if(ii*measure1.angle_increment+measure1.angle_min>bear3-margin3 && ii*measure1.angle_increment+measure1.angle_min<bear3+margin3)
	{
		if(ii*measure1.angle_increment+measure1.angle_min>bear3-margin3*0.65 && ii*measure1.angle_increment+measure1.angle_min<bear3+margin3*0.65)
			{
				
				rand_n=rand()/(RAND_MAX + 1.0);
				if(rand_n<0.8)
				{
		ff=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((3-pose1.pose.pose.position.x),2))*cos(measure1.angle_min+ii*measure1.angle_increment-bear3);
		ww=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((3-pose1.pose.pose.position.x),2))*sin(measure1.angle_min+ii*measure1.angle_increment-bear3);
		prerange=abs(ff-(sqrt(0.057*0.057-pow(ww,2))));
		}
				else
					prerange=sqrt(-1);
			}
			else
			prerange=sqrt(-1);
	}

	else if(ii*measure1.angle_increment+measure1.angle_min>bear4-margin4 && ii*measure1.angle_increment+measure1.angle_min<bear4+margin4)
	{
		if(ii*measure1.angle_increment+measure1.angle_min>bear4-margin4*0.65 && ii*measure1.angle_increment+measure1.angle_min<bear4+margin4*0.65)
			{
				
				rand_n=rand()/(RAND_MAX + 1.0);
				if(rand_n<0.8)
				{
		ff=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((5-pose1.pose.pose.position.x),2))*cos(measure1.angle_min+ii*measure1.angle_increment-bear4);
		ww=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((5-pose1.pose.pose.position.x),2))*sin(measure1.angle_min+ii*measure1.angle_increment-bear4);
		prerange=abs(ff-(sqrt(0.057*0.057-pow(ww,2))));
}
				else
					prerange=sqrt(-1);
			}
			else
			prerange=sqrt(-1);
	}

	else if(ii*measure1.angle_increment+measure1.angle_min>bear5-margin5 && ii*measure1.angle_increment+measure1.angle_min<bear5+margin5)
	{
		if(ii*measure1.angle_increment+measure1.angle_min>bear5-margin5*0.65 && ii*measure1.angle_increment+measure1.angle_min<bear5+margin5*0.65)
			{
				
				rand_n=rand()/(RAND_MAX + 1.0);
				if(rand_n<0.8)
				{
		ff=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((6-pose1.pose.pose.position.x),2))*cos(measure1.angle_min+ii*measure1.angle_increment-bear5);
		ww=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((6-pose1.pose.pose.position.x),2))*sin(measure1.angle_min+ii*measure1.angle_increment-bear5);
		prerange=abs(ff-(sqrt(0.057*0.057-pow(ww,2))));
}
				else
					prerange=sqrt(-1);
			}
			else
			prerange=sqrt(-1);	
		}

	else if(ii*measure1.angle_increment+measure1.angle_min>bear6-margin6 && ii*measure1.angle_increment+measure1.angle_min<bear6+margin6)
	{
		if(ii*measure1.angle_increment+measure1.angle_min>bear6-margin6*0.65 && ii*measure1.angle_increment+measure1.angle_min<bear6+margin6*0.65)
			{
				
				rand_n=rand()/(RAND_MAX + 1.0);
				if(rand_n<0.8)
				{
		ff=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((7-pose1.pose.pose.position.x),2))*cos(measure1.angle_min+ii*measure1.angle_increment-bear6);
		ww=sqrt(pow((0-pose1.pose.pose.position.y),2)+pow((7-pose1.pose.pose.position.x),2))*sin(measure1.angle_min+ii*measure1.angle_increment-bear6);
		prerange=abs(ff-(sqrt(0.057*0.057-pow(ww,2))));
}
				else
					prerange=sqrt(-1);
			}
			else
			prerange=sqrt(-1);		
}
		else
		  {
double rang_ang=measure1.angle_min+ii*measure1.angle_increment+yaw1;
if(rang_ang>3.14159)rang_ang=rang_ang-3.14159*2;
if(rang_ang<-3.14159)rang_ang=rang_ang+3.14159*2;
if(rang_ang<-9.42477)rang_ang=rang_ang+3.14159*4;
	if(rang_ang>0)		    
	prerange=abs(0.7-pose1.pose.pose.position.y)/sin(rang_ang);
else 
prerange=abs(-0.7-pose1.pose.pose.position.y)/sin(abs(rang_ang));

		  }

	for(i=0;i<13;i++)
			  {
		    s4=s4+(rand()/(RAND_MAX + 1.0))*var*2-var;
			  }
		s4=s4/2;
		ranges[ii]=prerange+s4;
		if(ranges[ii]>3)ranges[ii]=sqrt(-1);

	}

	  
measure1.ranges=ranges;

return;
}


