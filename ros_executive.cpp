#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "ros_executive_cmdline.h"
#include <cmath>

using namespace std;
using namespace ros;

void local2goal(const geometry_msgs::PoseWithCovariance::ConstPtr& msg);

geometry_msgs::Pose goal1;

double e_predict[2]={0,0};
double path=0,reset=1,rsum=0,mark=0,stm=40.0,tscale=0.8;


int main(int argc, char** argv)
	{
	cout<<"begin"<<endl;
	goal1.position.x=0.35;
	goal1.position.y=0.2;
	init(argc, argv,"executive");

	gengetopt_args_info args;
	cmdline_parser(argc,argv,&args);
	
	NodeHandle n;
	Subscriber localization=n.subscribe("/prediction",1000,local2goal);

	NodeHandle p;
	Publisher goal=p.advertise<geometry_msgs::Pose>("/goal",1000);

	ros::Rate loop_rate(10);
		while(ros::ok() )
		{
		goal1.position.z=mark;
		cout<<goal1.position.z<<endl;  
		goal.publish(goal1);
//cout<<goal1.position.x<<" "<<goal1.position.y<<endl;
		ros::spinOnce();
		
		loop_rate.sleep();
		}
		return 0;

		}
	

void local2goal(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)
{
double jug,dia=0.1,rax=0.3,ray=0.4;
e_predict[0]=msg->pose.position.x;
e_predict[1]=msg->pose.position.y;


if(goal1.position.x==0.5*(1-rax) &&goal1.position.y==0.5*ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=1-rax;
goal1.position.y=ray;
}
/*else if(((e_predict[0]>1-dia &&e_predict[0]<1+dia)||mark!=0) && reset==1)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=1;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=2;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=2;
}
}

else if(((e_predict[0]>2-dia &&e_predict[0]<2+dia)||mark!=0) && reset==2)
{
cout<<rsum<<endl;
if(rsum<(tscale*stm))
{mark=3;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=4;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=3;
}
}

else if(((e_predict[0]>3-dia &&e_predict[0]<3+dia)||mark!=0) && reset==3)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=1;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=2;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=5;
}
}

else if(((e_predict[0]>5-dia &&e_predict[0]<5+dia)||mark!=0) && reset==5)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=3;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=4;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=6;
}
}

else if(((e_predict[0]>6-dia &&e_predict[0]<6+dia)||mark!=0) && reset==6)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=1;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=2;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=7;
}
}

else if(((e_predict[0]>7-dia &&e_predict[0]<7+dia)||mark!=0) && reset==7)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=3;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=4;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=-7;
}
}

else if(((e_predict[0]>7-dia &&e_predict[0]<7+dia)||mark!=0) && reset==-7)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=3;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=4;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=-6;
}
}

else if(((e_predict[0]>6-dia &&e_predict[0]<6+dia)||mark!=0) && reset==-6)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=1;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=2;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=-5;
}
}

else if(((e_predict[0]>5-dia &&e_predict[0]<5+dia)||mark!=0) && reset==-5)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=3;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=4;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=-3;
}
}

else if(((e_predict[0]>3-dia &&e_predict[0]<3+dia)||mark!=0) && reset==-3)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=1;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=2;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=-2;
}
}


else if(((e_predict[0]>2-dia &&e_predict[0]<2+dia)||mark!=0) && reset==-2)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=3;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=4;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=-1;
}
}

else if(((e_predict[0]>1-dia &&e_predict[0]<1+dia)||mark!=0) && reset==-1)
{
cout<<mark<<endl;
if(rsum<(tscale*stm))
{mark=1;
rsum=rsum+1;}
else if(rsum>=(tscale*stm) && rsum<stm)
{
mark=2;
rsum=rsum+1;
}
else if(rsum>=stm)
{
mark=0;
reset=0;
}
}
*/
else if(goal1.position.x==1-rax &&goal1.position.y==ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=1+rax;
goal1.position.y=ray;
}
else if(goal1.position.x==1+rax &&goal1.position.y==ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=1.5;
goal1.position.y=0;
rsum=0;
}
else if(goal1.position.x==1.5 &&goal1.position.y==0  &&path==0 && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=2-rax;
goal1.position.y=-ray;
}
else if(goal1.position.x==2-rax &&goal1.position.y==-ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=2+rax;
goal1.position.y=-ray;
}
else if(goal1.position.x==2+rax &&goal1.position.y==-ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=2.5;
goal1.position.y=0;
rsum=0;
}
else if(goal1.position.x==2.5 &&goal1.position.y==0 &&path==0 && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=3-rax;
goal1.position.y=ray;
}
else if(goal1.position.x==3-rax &&goal1.position.y==ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=3+rax;
goal1.position.y=ray;
}
else if(goal1.position.x==3+rax &&goal1.position.y==ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=4;
goal1.position.y=0;
rsum=0;
}
else if(goal1.position.x==4 &&goal1.position.y==0 &&path==0 && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=5-rax;
goal1.position.y=-ray;
}
else if(goal1.position.x==5-rax &&goal1.position.y==-ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=5+rax;
goal1.position.y=-ray;
}
else if(goal1.position.x==5+rax &&goal1.position.y==-ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=5.5;
goal1.position.y=0;
rsum=0;
}
else if(goal1.position.x==5.5 &&goal1.position.y==0 &&path==0 && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=6-rax;
goal1.position.y=ray;
}
else if(goal1.position.x==6-rax &&goal1.position.y==ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=6+rax;
goal1.position.y=ray;
}
else if(goal1.position.x==6+rax &&goal1.position.y==ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=6.5;
goal1.position.y=0;
rsum=0;
}
else if(goal1.position.x==6.5 &&goal1.position.y==0  &&path==0 && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=7-rax;
goal1.position.y=-ray;
}
else if(goal1.position.x==7-rax &&goal1.position.y==-ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=7+rax;
goal1.position.y=-ray;
}
else if(goal1.position.x==7+rax &&goal1.position.y==-ray && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=7.5;
goal1.position.y=0;

}
else if(goal1.position.x==7.5 &&goal1.position.y==0  &&path==0 && e_predict[0]>(goal1.position.x-dia) && e_predict[0]<(goal1.position.x+dia) && e_predict[1]>(goal1.position.y-dia) && e_predict[1]<(goal1.position.y+dia))
{
goal1.position.x=100;
goal1.position.y=100;
path=1;
rsum=0;
}

//cout<<goal1.position.x<<" "<<goal1.position.y<<endl;
return;
}
