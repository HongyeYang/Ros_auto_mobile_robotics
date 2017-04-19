#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "ros_localization_cmdline.h"
#include <vector>
#include <cmath>
#include <fstream>


using namespace std;
using namespace ros;

ofstream labdata1;

void motion_dealing(const geometry_msgs::Twist::ConstPtr& msg1);
void scan_dealing(const sensor_msgs::LaserScan::ConstPtr& msg);
void zit_loop();

void matrix_multiply(double *matrix1, int m1, int n1, double *matrix2, int m2, int n2, double *matrix3);
void matrix_plus(double *matrix1, int m, int n, double *matrix2, double *matrix3);
void matrix_minus(double *matrix1, int m, int n, double *matrix2, double *matrix3);
void matrix_multiply3(double *matrix1, int m1, int n1, double *matrix2, int m2, int n2, double *matrix3, int m3, int n3, double *matrix4);
void matrix_transpose(double *matrix1, int n, int m, double *matrix2);
void matrix_inv(double *matrix1, int m, int n, double *matrix2);
void matrix_equal(double *matrix1, int m, int n, double *matrix2);


geometry_msgs::PoseWithCovariance Prediction1;
sensor_msgs::LaserScan scan1;

double predict[3][1]={0.0,0.0,0.0};
double predict_bar[3][1]={0.0,0.0,0.0};
double yaw,v,w,ra=0.057,q,dt=0.1,mx,my,fallr=0.3,falla=0.3,ravg=0.1,rdi=0.08,rpdx=1.5,rpd=1.5;
double Gt[3][3],Vt[3][2],Mt[2][2],cv[3][3]={{0,0,0},{0,0,0},{0,0,0}}/*{{0.1,0.1,0.1},{0.1,0.1,0.1},{0.1,0.1,0.1}}*/,cv_bar[3][3]={{0,0,0},{0,0,0},{0,0,0}}/*{{0.1,0.1,0.1},{0.1,0.1,0.1},{0.1,0.1,0.1}}*/,Qt[3][3]={{0.0001,0,0},{0,0.0001,0},{0,0,0.0001}},Zit[3][1],Zit_bar[3][1],Hit[3][3],Sit[3][3],Kit[3][3],a[6]={0.1,0.1,0.3,0.3,0.1,0.1},vv[3][1],I[3][3]={{1,0,0},{0,1,0},{0,0,1}};
int rsize;

double thres=10;



int main(int argc, char** argv)
	{
	cout<<"begin"<<endl;

	
	init(argc, argv,"locationer");

	gengetopt_args_info args;
	cmdline_parser(argc,argv,&args);
labdata1.open("lab56_prediction_pose_mean.txt");

NodeHandle n;
Subscriber Motion=n.subscribe("/cmd_vel_mux/input/navi",1000,motion_dealing);

NodeHandle k;
Subscriber Laser=k.subscribe("/scan",1000,scan_dealing);

NodeHandle p;
Publisher prediction=p.advertise<geometry_msgs::PoseWithCovariance>("/prediction",1000);

ros::Rate loop_rate(10);
	while(ros::ok() )
	{
	  
		  prediction.publish(Prediction1);
		  ros::spinOnce();
		
	loop_rate.sleep();
	}
	return 0;

	}

void motion_dealing(const geometry_msgs::Twist::ConstPtr& msg1)
{
//store motion msg

v=msg1->linear.x;

w=msg1->angular.z;

yaw=predict[2][0];



//define Gt
Gt[0][0]=1;
Gt[0][1]=0;
Gt[0][2]=-v/w*cos(yaw)+v/w*cos(yaw+w*dt);
Gt[1][0]=0;
Gt[1][1]=1;
Gt[1][2]=-v/w*sin(yaw)+v/w*sin(yaw+w*dt);
Gt[2][0]=0;
Gt[2][1]=0;
Gt[2][2]=1;

//define Vt
Vt[0][0]=-(sin(yaw)+sin(yaw+w*dt))/w;
Vt[0][1]=v*(sin(yaw)-sin(yaw+w*dt))/(w*w)+v*cos(yaw+w*dt)*dt/w;
Vt[1][0]=(cos(yaw)-cos(yaw+w*dt))/w;
Vt[1][1]=-v*(cos(yaw)-cos(yaw+w*dt))/(w*w)+v*sin(yaw+w*dt)*dt/w;
Vt[2][0]=0;
Vt[2][1]=dt;

//define Mt
Mt[0][0]=(a[0]*abs(v)+a[1]*abs(w))*(a[0]*abs(v)+a[1]*abs(w));
Mt[0][1]=0;
Mt[1][0]=0;
Mt[1][1]=(a[2]*abs(v)+a[3]*abs(w))*(a[2]*abs(v)+a[3]*abs(w));

//prediction_bar
vv[0][0]=-v/w*sin(yaw)+v/w*sin(yaw+w*dt);
vv[1][0]=v/w*cos(yaw)-v/w*cos(yaw+w*dt);
vv[2][0]=w*dt;
matrix_plus(&predict[0][0],3,1,&vv[0][0],&predict_bar[0][0]);

//cout<<"predict_bar="<<predict_bar[0][0]<<","<<predict_bar[1][0]<<endl;
//calculate covariance

double Gt_t[3][3], cv1[3][3], cv2[3][3],Vt_t[3][3];
matrix_transpose(&Gt[0][0], 3, 3,&Gt_t[0][0]);
matrix_multiply3(&Gt[0][0], 3, 3, &cv[0][0], 3, 3, &Gt_t[0][0],3,3,&cv1[0][0]);
matrix_transpose(&Vt[0][0], 3, 2,&Vt_t[0][0]);
matrix_multiply3(&Vt[0][0], 3, 2, &Mt[0][0], 2, 2, &Vt_t[0][0],2,3,&cv2[0][0]);
matrix_plus(&cv1[0][0],3,3,&cv2[0][0],&cv_bar[0][0]);




return;
}


void scan_dealing(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//store the scan msg
rsize=msg->ranges.size();
scan1.ranges.resize(rsize);
scan1.angle_min=msg->angle_min;
scan1.angle_max=msg->angle_max;
scan1.angle_increment=msg->angle_increment;
scan1.time_increment=msg->time_increment;
scan1.scan_time=msg->scan_time;
scan1.range_min=msg->range_min;
scan1.range_max=msg->range_max;
scan1.ranges=msg->ranges;

//cout<<"min_ang= "<<scan1.angle_min<<" angle_inc="<<scan1.angle_increment<<endl;

//find position of the cone
double sumn_1=0,sumn_2=0,sumn_3=0,sumn_5=0,sumn_6=0,sumn_7=0;
double sumx_1=0,sumx_2=0,sumx_3=0,sumx_5=0,sumx_6=0,sumx_7=0;
double sumy_1=0,sumy_2=0,sumy_3=0,sumy_5=0,sumy_6=0,sumy_7=0;
double sumx2_1=0,sumx2_2=0,sumx2_3=0,sumx2_5=0,sumx2_6=0,sumx2_7=0;
double sumy2_1=0,sumy2_2=0,sumy2_3=0,sumy2_5=0,sumy2_6=0,sumy2_7=0;
double sumxy_1=0,sumxy_2=0,sumxy_3=0,sumxy_5=0,sumxy_6=0,sumxy_7=0;
double sumx2py2_1=0,sumx2py2_2=0,sumx2py2_3=0,sumx2py2_5=0,sumx2py2_6=0,sumx2py2_7=0;
double sumx3_1=0,sumx3_2=0,sumx3_3=0,sumx3_5=0,sumx3_6=0,sumx3_7=0;
double sumy3_1=0,sumy3_2=0,sumy3_3=0,sumy3_5=0,sumy3_6=0,sumy3_7=0;
double sumx2y_1=0,sumx2y_2=0,sumx2y_3=0,sumx2y_5=0,sumx2y_6=0,sumx2y_7=0;
double sumxy2_1=0,sumxy2_2=0,sumxy2_3=0,sumxy2_5=0,sumxy2_6=0,sumxy2_7=0;
double ragx,ragy,cc,dd,ee,gg,hh,aa,bb,yuanx,yuany,ccc,rri,bbi,RG1,RG2,RG3,RG5,RG6,RG7,AG1,AG2,AG3,AG5,AG6,AG7;
RG1=sqrt((1-predict[0][0])*(1-predict[0][0])+(0-predict[1][0])*(0-predict[1][0]));
	AG1=atan2((0-predict[1][0]),(1-predict[0][0]))-yaw;
if(AG1<-3.14159)AG1=AG1+2*3.14159;
if(AG1<-3*3.14159)AG1=AG1+4*3.14159;
if(AG1>3.14159)AG1=AG1-2*3.14159;

RG2=sqrt((2-predict[0][0])*(2-predict[0][0])+(0-predict[1][0])*(0-predict[1][0]));
	AG2=atan2((0-predict[1][0]),(2-predict[0][0]))-yaw;

if(AG2<-3.14159)AG2=AG2+2*3.14159;
if(AG2<-3*3.14159)AG2=AG2+4*3.14159;
if(AG2>3.14159)AG2=AG2-2*3.14159;

	RG3=sqrt((3-predict[0][0])*(3-predict[0][0])+(0-predict[1][0])*(0-predict[1][0]));
	AG3=atan2((0-predict[1][0]),(3-predict[0][0]))-yaw;
if(AG3<-3.14159)AG3=AG3+2*3.14159;
if(AG3<-3*3.14159)AG3=AG3+4*3.14159;
if(AG3>3.14159)AG3=AG3-2*3.14159;

RG5=sqrt((5-predict[0][0])*(5-predict[0][0])+(0-predict[1][0])*(0-predict[1][0]));
	AG5=atan2((0-predict[1][0]),(5-predict[0][0]))-yaw;
if(AG5<-3.14159)AG5=AG5+2*3.14159;
if(AG5<-3*3.14159)AG5=AG5+4*3.14159;
if(AG5>3.14159)AG5=AG5-2*3.14159;

RG6=sqrt((6-predict[0][0])*(6-predict[0][0])+(0-predict[1][0])*(0-predict[1][0]));
	AG6=atan2((0-predict[1][0]),(6-predict[0][0]))-yaw;
if(AG6<-3.14159)AG6=AG6+2*3.14159;
if(AG6<-3*3.14159)AG6=AG6+4*3.14159;
if(AG6>3.14159)AG6=AG6-2*3.14159;

RG7=sqrt((7-predict[0][0])*(7-predict[0][0])+(0-predict[1][0])*(0-predict[1][0]));
	AG7=atan2((0-predict[1][0]),(7-predict[0][0]))-yaw;
if(AG7<-3.14159)AG7=AG7+2*3.14159;
if(AG7<-3*3.14159)AG7=AG7+4*3.14159;
if(AG7>3.14159)AG7=AG7-2*3.14159;


//cout<<"G1= "<<RG1<<" "<<AG1<<" G2= "<<RG2<<" "<<AG2<<" G3= "<<RG3<<" "<<AG3<<" G5="<<RG5<<" "<<AG5<<" G6= "<<RG6<<" "<<AG6<<" G7="<<RG7<<" "<<AG7<<endl;
for(int i=0;i<rsize;i++)
{
	rri=scan1.ranges[i];
	bbi=i*scan1.angle_increment+scan1.angle_min;

	
ragx=scan1.ranges[i]*cos(i*scan1.angle_increment+scan1.angle_min+yaw)+predict_bar[0][0];
	ragy=scan1.ranges[i]*sin(i*scan1.angle_increment+scan1.angle_min+yaw)+predict_bar[1][0];
	//cone(1,0)
	if(rri<RG1+fallr && rri>RG1-fallr && bbi>AG1-falla/rri &&bbi<AG1+falla/rri)
	{
	sumn_1=sumn_1+1;
	sumx_1=sumx_1+ragx;
	sumy_1=sumy_1+ragy;
	sumx2_1=sumx2_1+ragx*ragx;
	sumy2_1=sumy2_1+ragy*ragy;
	sumxy_1=sumxy_1+ragx*ragy;
	sumx2py2_1=sumx2py2_1+ragx*ragx+ragy*ragy;
	sumx3_1=sumx3_1+ragx*ragx*ragx;
	sumy3_1=sumy3_1+ragy*ragy*ragy;
	sumx2y_1=sumx2y_1+ragx*ragx*ragy;
	sumxy2_1=sumxy2_1+ragx*ragy*ragy;
	}
	
	if(rri<RG2+fallr && rri>RG2-fallr && bbi>AG2-falla/rri &&bbi<AG2+falla/rri)
	{
	sumn_2=sumn_2+1;
	sumx_2=sumx_2+ragx;
	sumy_2=sumy_2+ragy;
	sumx2_2=sumx2_2+ragx*ragx;
	sumy2_2=sumy2_2+ragy*ragy;
	sumxy_2=sumxy_2+ragx*ragy;
	sumx2py2_2=sumx2py2_2+ragx*ragx+ragy*ragy;
	sumx3_2=sumx3_2+ragx*ragx*ragx;
	sumy3_2=sumy3_2+ragy*ragy*ragy;
	sumx2y_2=sumx2y_2+ragx*ragx*ragy;
	sumxy2_2=sumxy2_2+ragx*ragy*ragy;
	}

	if(rri<RG3+fallr && rri>RG3-fallr && bbi>AG3-falla/rri &&bbi<AG3+falla/rri)
	{
	sumn_3=sumn_3+1;
	sumx_3=sumx_3+ragx;
	sumy_3=sumy_3+ragy;
	sumx2_3=sumx2_3+ragx*ragx;
	sumy2_3=sumy2_3+ragy*ragy;
	sumxy_3=sumxy_3+ragx*ragy;
	sumx2py2_3=sumx2py2_3+ragx*ragx+ragy*ragy;
	sumx3_3=sumx3_3+ragx*ragx*ragx;
	sumy3_3=sumy3_3+ragy*ragy*ragy;
	sumx2y_3=sumx2y_3+ragx*ragx*ragy;
	sumxy2_3=sumxy2_3+ragx*ragy*ragy;
	}

	if(rri<RG5+fallr && rri>RG5-fallr && bbi>AG5-falla/rri &&bbi<AG5+falla/rri)
	{
	sumn_5=sumn_5+1;
	sumx_5=sumx_5+ragx;
	sumy_5=sumy_5+ragy;
	sumx2_5=sumx2_5+ragx*ragx;
	sumy2_5=sumy2_5+ragy*ragy;
	sumxy_5=sumxy_5+ragx*ragy;
	sumx2py2_5=sumx2py2_5+ragx*ragx+ragy*ragy;
	sumx3_5=sumx3_5+ragx*ragx*ragx;
	sumy3_5=sumy3_5+ragy*ragy*ragy;
	sumx2y_5=sumx2y_5+ragx*ragx*ragy;
	sumxy2_5=sumxy2_5+ragx*ragy*ragy;
	}

	if(rri<RG6+fallr && rri>RG6-fallr && bbi>AG6-falla/rri &&bbi<AG6+falla/rri)
	{
	sumn_6=sumn_6+1;
	sumx_6=sumx_6+ragx;
	sumy_6=sumy_6+ragy;
	sumx2_6=sumx2_6+ragx*ragx;
	sumy2_6=sumy2_6+ragy*ragy;
	sumxy_6=sumxy_6+ragx*ragy;
	sumx2py2_6=sumx2py2_6+ragx*ragx+ragy*ragy;
	sumx3_6=sumx3_6+ragx*ragx*ragx;
	sumy3_6=sumy3_6+ragy*ragy*ragy;
	sumx2y_6=sumx2y_6+ragx*ragx*ragy;
	sumxy2_6=sumxy2_6+ragx*ragy*ragy;
	}

	if(rri<RG7+fallr && rri>RG7-fallr && bbi>AG7-falla/rri &&bbi<AG7+falla/rri)
	{
	sumn_7=sumn_7+1;
	sumx_7=sumx_7+ragx;
	sumy_7=sumy_7+ragy;
	sumx2_7=sumx2_7+ragx*ragx;
	sumy2_7=sumy2_7+ragy*ragy;
	sumxy_7=sumxy_7+ragx*ragy;
	sumx2py2_7=sumx2py2_7+ragx*ragx+ragy*ragy;
	sumx3_7=sumx3_7+ragx*ragx*ragx;
	sumy3_7=sumy3_7+ragy*ragy*ragy;
	sumx2y_7=sumx2y_7+ragx*ragx*ragy;
	sumxy2_7=sumxy2_7+ragx*ragy*ragy;
	}
}

//calculate cone(1,0)
if(sumn_1>=thres)
{
	cc=sumn_1*sumx2_1-sumx_1*sumx_1;
	dd=sumn_1*sumxy_1-sumx_1*sumy_1;
	ee=sumn_1*sumx3_1+sumn_1*sumxy2_1-sumx2py2_1*sumx_1;
	gg=sumn_1*sumy2_1-sumy_1*sumy_1;
	hh=sumn_1*sumx2y_1+sumn_1*sumy3_1-sumx2py2_1*sumy_1;

	aa=(hh*dd-ee*gg)/(cc*gg-dd*dd);
	bb=(hh*cc-ee*dd)/(dd*dd-gg*cc);
	ccc=-(sumx2py2_1+aa*sumx_1+bb*sumy_1)/sumn_1;
	
	
	yuanx=aa/(-2);
	yuany=bb/(-2);
	ra=0.5*sqrt(aa*aa+bb*bb-4*ccc);


	Zit[0][0]=sqrt((yuanx-predict[0][0])*(yuanx-predict[0][0])+(yuany-predict[1][0])*(yuany-predict[1][0]));
	Zit[1][0]=atan2(yuany-predict[1][0],yuanx-predict[0][0])-predict[2][0];
	if(predict[0][0]>3 && predict[0][0]<5 &&predict[1][0]<0)rpd=2.0;
	if(predict[0][0]<3 ||predict[0][0]>5 ||predict[1][0]<0)rpd=rpdx;
	if(Zit[0][0]<rpd  &&ra>ravg-rdi &&ra<ravg+rdi)//&& yuanx>1-scandiff &&yuanx<1+scandiff &&yuany<0+scandiff &&yuany>0-scandiff)
	Zit[2][0]=1;
	else
	Zit[2][0]=0;
	mx=1;
	my=0;
cout<<"circle1 "<<yuanx<<"  "<<yuany<<" ra="<<ra<<endl;
	zit_loop();
	
	
	
}


	if(sumn_2>=thres)
{
	cc=sumn_2*sumx2_2-sumx_2*sumx_2;
	dd=sumn_2*sumxy_2-sumx_2*sumy_2;
	ee=sumn_2*sumx3_2+sumn_2*sumxy2_2-sumx2py2_2*sumx_2;
	gg=sumn_2*sumy2_2-sumy_2*sumy_2;
	hh=sumn_2*sumx2y_2+sumn_2*sumy3_2-sumx2py2_2*sumy_2;

	aa=(hh*dd-ee*gg)/(cc*gg-dd*dd);
	bb=(hh*cc-ee*dd)/(dd*dd-gg*cc);
	ccc=-(sumx2py2_2+aa*sumx_2+bb*sumy_2)/sumn_2;
	
	
	yuanx=aa/(-2);
	yuany=bb/(-2);
	ra=0.5*sqrt(aa*aa+bb*bb-4*ccc);

	
	Zit[0][0]=sqrt((yuanx-predict[0][0])*(yuanx-predict[0][0])+(yuany-predict[1][0])*(yuany-predict[1][0]));
	Zit[1][0]=atan2(yuany-predict[1][0],yuanx-predict[0][0])-predict[2][0];

	if(Zit[0][0]<rpd  &&ra>ravg-rdi &&ra<ravg+rdi)// && yuanx>2-scandiff &&yuanx<2+scandiff &&yuany<0+scandiff &&yuany>0-scandiff)
	Zit[2][0]=1;
	else
	Zit[2][0]=0;
cout<<"circle2 "<<yuanx<<" "<<yuany<<endl;
	mx=2;
	my=0;

	zit_loop();
	
	
	
}
	


if(sumn_3>=thres)
{
	cc=sumn_3*sumx2_3-sumx_3*sumx_3;
	dd=sumn_3*sumxy_3-sumx_3*sumy_3;
	ee=sumn_3*sumx3_3+sumn_3*sumxy2_3-sumx2py2_3*sumx_3;
	gg=sumn_3*sumy2_3-sumy_3*sumy_3;
	hh=sumn_3*sumx2y_3+sumn_3*sumy3_3-sumx2py2_3*sumy_3;

	aa=(hh*dd-ee*gg)/(cc*gg-dd*dd);
	bb=(hh*cc-ee*dd)/(dd*dd-gg*cc);
	ccc=-(sumx2py2_3+aa*sumx_3+bb*sumy_3)/sumn_3;
	
	
	yuanx=aa/(-2);
	yuany=bb/(-2);
	ra=0.5*sqrt(aa*aa+bb*bb-4*ccc);



	Zit[0][0]=sqrt((yuanx-predict[0][0])*(yuanx-predict[0][0])+(yuany-predict[1][0])*(yuany-predict[1][0]));
	Zit[1][0]=atan2(yuany-predict[1][0],yuanx-predict[0][0])-predict[2][0];
	if(Zit[0][0]<rpd  &&ra>ravg-rdi &&ra<ravg+rdi)
	Zit[2][0]=1;
	else
	Zit[2][0]=0;
cout<<"circle3 "<<yuanx<<" "<<yuany<<endl;
	mx=3;
	my=0;

	zit_loop();
	
	
	
}

	if(sumn_5>=thres)
{
	cc=sumn_5*sumx2_5-sumx_5*sumx_5;
	dd=sumn_5*sumxy_5-sumx_5*sumy_5;
	ee=sumn_5*sumx3_5+sumn_5*sumxy2_5-sumx2py2_5*sumx_5;
	gg=sumn_5*sumy2_5-sumy_5*sumy_5;
	hh=sumn_5*sumx2y_5+sumn_5*sumy3_5-sumx2py2_5*sumy_5;

	aa=(hh*dd-ee*gg)/(cc*gg-dd*dd);
	bb=(hh*cc-ee*dd)/(dd*dd-gg*cc);
	ccc=-(sumx2py2_5+aa*sumx_5+bb*sumy_5)/sumn_5;
	
	
	yuanx=aa/(-2);
	yuany=bb/(-2);
	ra=0.5*sqrt(aa*aa+bb*bb-4*ccc);

cout<<"(5,0) "<<yuanx<<" "<<yuany<<endl;

	Zit[0][0]=sqrt((yuanx-predict[0][0])*(yuanx-predict[0][0])+(yuany-predict[1][0])*(yuany-predict[1][0]));
	Zit[1][0]=atan2(yuany-predict[1][0],yuanx-predict[0][0])-predict[2][0];
	if(Zit[0][0]<rpd  &&ra>ravg-rdi &&ra<ravg+rdi)
	Zit[2][0]=1;
	else
	Zit[2][0]=0;
	mx=5;
	my=0;

	zit_loop();
	
	
	
}
	if(sumn_6>=thres)
{
	cc=sumn_6*sumx2_6-sumx_6*sumx_6;
	dd=sumn_6*sumxy_6-sumx_6*sumy_6;
	ee=sumn_6*sumx3_6+sumn_6*sumxy2_6-sumx2py2_6*sumx_6;
	gg=sumn_6*sumy2_6-sumy_6*sumy_6;
	hh=sumn_6*sumx2y_6+sumn_6*sumy3_6-sumx2py2_6*sumy_6;

	aa=(hh*dd-ee*gg)/(cc*gg-dd*dd);
	bb=(hh*cc-ee*dd)/(dd*dd-gg*cc);
	ccc=-(sumx2py2_6+aa*sumx_6+bb*sumy_6)/sumn_6;
	
	
	yuanx=aa/(-2);
	yuany=bb/(-2);
	ra=0.5*sqrt(aa*aa+bb*bb-4*ccc);

cout<<"(6,0) "<<yuanx<<" "<<yuany<<endl;

	Zit[0][0]=sqrt((yuanx-predict[0][0])*(yuanx-predict[0][0])+(yuany-predict[1][0])*(yuany-predict[1][0]));
	Zit[1][0]=atan2(yuany-predict[1][0],yuanx-predict[0][0])-predict[2][0];
	if(Zit[0][0]<rpd  &&ra>ravg-rdi &&ra<ravg+rdi)
	Zit[2][0]=1;
	else
	Zit[2][0]=0;
	mx=6;
	my=0;

	zit_loop();
	
	
	
}
	if(sumn_7>=thres)
{
	cc=sumn_7*sumx2_7-sumx_7*sumx_7;
	dd=sumn_7*sumxy_7-sumx_7*sumy_7;
	ee=sumn_7*sumx3_7+sumn_7*sumxy2_7-sumx2py2_7*sumx_7;
	gg=sumn_7*sumy2_7-sumy_7*sumy_7;
	hh=sumn_7*sumx2y_7+sumn_7*sumy3_7-sumx2py2_7*sumy_7;

	aa=(hh*dd-ee*gg)/(cc*gg-dd*dd);
	bb=(hh*cc-ee*dd)/(dd*dd-gg*cc);
	ccc=-(sumx2py2_7+aa*sumx_7+bb*sumy_7)/sumn_7;
	
	
	yuanx=aa/(-2);
	yuany=bb/(-2);
	ra=0.5*sqrt(aa*aa+bb*bb-4*ccc);

cout<<"(7,0) "<<yuanx<<" "<<yuany<<endl;

	Zit[0][0]=sqrt((yuanx-predict[0][0])*(yuanx-predict[0][0])+(yuany-predict[1][0])*(yuany-predict[1][0]));
	Zit[1][0]=atan2(yuany-predict[1][0],yuanx-predict[0][0])-predict[2][0];
	if(Zit[0][0]<rpd  &&ra>ravg-rdi &&ra<ravg+rdi)
	Zit[2][0]=1;
	else
	Zit[2][0]=0;
	mx=7;
	my=0;

	zit_loop();
	
	
	
}

	if(sumn_1<thres &&sumn_2<thres &&sumn_3<thres &&sumn_5<thres &&sumn_6<thres &&sumn_7<thres)
{
	Zit[0][0]=0;
	Zit[1][0]=0;
	Zit[2][0]=0;

	mx=0;
	my=0;
	//cout<<"mx="<<mx<<" my="<<my<<endl;
}	


//update

matrix_equal(&predict_bar[0][0], 3, 1,&predict[0][0]);
matrix_equal(&cv_bar[0][0], 3, 3,&cv[0][0]);

Prediction1.pose.position.x=predict[0][0];
Prediction1.pose.position.y=predict[1][0];
Prediction1.pose.position.z=0;

Prediction1.pose.orientation.x=0;
Prediction1.pose.orientation.y=0;
Prediction1.pose.orientation.z=sin(0.5*predict[2][0]);
Prediction1.pose.orientation.w=cos(0.5*predict[2][0]);


Prediction1.covariance[0]=cv[0][0];
Prediction1.covariance[1]=cv[1][1];
Prediction1.covariance[2]=cv[2][2];


labdata1<<predict[0][0]<<","<<predict[1][0]<<","<<predict[2][0]<<";"<<endl;


for(int i=3;i<36;i++)
Prediction1.covariance[i]=0;

//cout<<Prediction1.pose.position.x<<"  "<<Prediction1.pose.position.y<<"  "<<predict[2][0]<<endl;
//cout<<Prediction1.covariance[0]<<"  "<<Prediction1.covariance[1]<<endl;
return;
}


void zit_loop(void)
{
if(Zit[2][0]==1)
	//prediction
	//define q
{
	q=(mx-predict_bar[0][0])*(mx-predict_bar[0][0])+(my-predict_bar[1][0])*(my-predict_bar[1][0]);
	//define Zit_bar

	Zit_bar[0][0]=sqrt(q);
	Zit_bar[1][0]=atan2((my-predict_bar[1][0]),(mx-predict_bar[0][0]))-predict_bar[2][0];
	if(Zit_bar[1][0]<-3.14159)Zit_bar[1][0]=Zit_bar[1][0]+2*3.14159;
	if(Zit_bar[1][0]>3.14159)Zit_bar[1][0]=Zit_bar[1][0]-2*3.14159;
	if(Zit_bar[1][0]<-9.42478)Zit_bar[1][0]=Zit_bar[1][0]+4*3.14159;

	if(Zit[1][0]<-3.14159)Zit[1][0]=Zit[1][0]+2*3.14159;
	if(Zit[1][0]>3.14159)Zit[1][0]=Zit[1][0]-2*3.14159;
	if(Zit[1][0]<-9.42478)Zit[1][0]=Zit[1][0]+4*3.14159;
	Zit_bar[2][0]=1;

	//define Hit
	Hit[0][0]=-(mx-predict_bar[0][0])/sqrt(q);
	Hit[0][1]=-(my-predict_bar[1][0])/sqrt(q);
	Hit[0][2]=0;
	Hit[1][0]=(my-predict_bar[1][0])/q;
	Hit[1][1]=-(mx-predict_bar[0][0])/q;
	Hit[1][2]=-1;
	Hit[2][0]=0;
	Hit[2][1]=0;
	Hit[2][2]=0;


	//define Sit
	double Hit_t[3][3],sit1[3][3];
	matrix_transpose(&Hit[0][0], 3, 3,&Hit_t[0][0]);


	matrix_multiply3(&Hit[0][0], 3, 3, &cv_bar[0][0], 3, 3, &Hit_t[0][0],3,3,&sit1[0][0]);
	matrix_plus(&sit1[0][0],3,3,&Qt[0][0],&Sit[0][0]);


	//define Kit
	double Sit_inv[3][3];
	matrix_inv(&Sit[0][0], 3, 3,&Sit_inv[0][0]);
	matrix_multiply3(&cv_bar[0][0], 3, 3, &Hit_t[0][0], 3, 3, &Sit_inv[0][0],3,3,&Kit[0][0]);
cout<<mx<<endl;
}
else
{
	
	Kit[0][0]=0;
	Kit[0][1]=0;
	Kit[0][2]=0;
	Kit[1][0]=0;
	Kit[1][1]=0;
	Kit[1][2]=0;
	Kit[2][0]=0;
	Kit[2][1]=0;
	Kit[2][2]=0;
}

//calculate final predict_bar and cv_bar
double z_diff[3][1],kz[3][1],predict_bar_gd[3][1],cv_bar_gd[3][3],kh[3][3],ikh[3][3];
matrix_minus(&Zit[0][0],3,1,&Zit_bar[0][0],&z_diff[0][0]);
matrix_multiply(&Kit[0][0], 3, 3, &z_diff[0][0], 3, 1, &kz[0][0]);

matrix_plus(&predict_bar[0][0],3,1,&kz[0][0],&predict_bar_gd[0][0]);
matrix_equal(&predict_bar_gd[0][0], 3, 1,&predict_bar[0][0]);

matrix_multiply(&Kit[0][0], 3, 3, &Hit[0][0], 3, 3, &kh[0][0]);
matrix_minus(&I[0][0],3,3,&kh[0][0],&ikh[0][0]);
matrix_multiply(&ikh[0][0], 3, 3, &cv_bar[0][0], 3, 3, &cv_bar_gd[0][0]);
matrix_equal(&cv_bar_gd[0][0], 3, 3,&cv_bar[0][0]);


return;
}




//*
void matrix_multiply(double *matrix1, int m1, int n1, double *matrix2, int m2, int n2, double *matrix3)
{
	for (int i = 0; i<m1; i++)
	{
		for (int j = 0; j<n2; j++)
		{
			double sum = 0;
			for (int ink = 0; ink<n1; ink++)
				sum = sum + (*(matrix1 + i*n1 + ink))*(*(matrix2 + ink*n2 + j));
			(*(matrix3 + i*n2 + j)) = sum;
		}
	}
	return;
}



//3*
void matrix_multiply3(double *matrix1, int m1, int n1, double *matrix2, int m2, int n2, double *matrix3, int m3, int n3, double *matrix4)
{
	if (n2 == 3)
	{
		double mgd1[3][3];
		matrix_multiply(matrix1, m1, n1, matrix2, m2, n2, &mgd1[0][0]);
		matrix_multiply(&mgd1[0][0], m1, n2, matrix3, m3, n3, matrix4);
	}
	else if (n2 == 2)
	{
		double mgd1[3][2];

		matrix_multiply(matrix1, m1, n1, matrix2, m2, n2, &mgd1[0][0]);
		matrix_multiply(&mgd1[0][0], m1, n2, matrix3, m3, n3, matrix4);
	}

	return;


}

//+
void matrix_plus(double *matrix1, int m, int n, double *matrix2, double *matrix3)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			(*(matrix3+i*n+j)) = (*(matrix1 + i*n + j)) + (*(matrix2 + i*n + j));
		}
	}

	return;
}


//minus
void matrix_minus(double *matrix1, int m, int n, double *matrix2, double *matrix3)
{
for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			(*(matrix3+i*n+j)) = (*(matrix1 + i*n + j)) - (*(matrix2 + i*n + j));
		}
	}


return;
}




//transpose
void matrix_transpose(double *matrix1, int n, int m, double *matrix2)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			(*(matrix2 + i*n+ j)) = (*(matrix1 + j*m + i));
		}
	}

	return;
}


//inv
void matrix_inv(double *matrix1, int m, int n, double *matrix2)
{
	double det, c11, c12, c13, c21, c22, c23, c31, c32, c33;
	if (m == 3)
	{

		c11 = (*(matrix1 + 0 * n + 0));
		c12 = (*(matrix1 + 0 * n + 1));
		c13 = (*(matrix1 + 0 * n + 2));
		c21 = (*(matrix1 + 1 * n + 0));
		c22 = (*(matrix1 + 1 * n + 1));
		c23 = (*(matrix1 + 1 * n + 2));
		c31 = (*(matrix1 + 2 * n + 0));
		c32 = (*(matrix1 + 2 * n + 1));
		c33 = (*(matrix1 + 2 * n + 2));;

		det = c11*c22*c33 - c11*c23*c32 - c12*c21*c33 + c12*c23*c31 + c13*c21*c32 - c13*c22*c31;
		(*(matrix2 + 0 * n + 0)) = (c22*c33 - c23*c32) / det;
		(*(matrix2 + 0 * n + 1)) = -(c12*c33 - c13*c32) / det;
		(*(matrix2 + 0 * n + 2)) = (c12*c23 - c13*c22) / det;

		(*(matrix2 + 1 * n + 0)) = -(c21*c33 - c23*c31) / det;
		(*(matrix2 + 1 * n + 1)) = (c11*c33 - c13*c31) / det;
		(*(matrix2 + 1 * n + 2)) = -(c11*c23 - c13*c21) / det;

		(*(matrix2 + 2 * n + 0)) = (c21*c32 - c22*c31) / det;
		(*(matrix2 + 2 * n + 1)) = -(c11*c32 - c12*c31) / det;
		(*(matrix2 + 2 * n + 2)) = (c11*c22 - c12*c21) / det;
	}
	else if (m == 2)
	{

		c11 = (*(matrix1 + 0 * n + 0));
		c12 = (*(matrix1 + 0 * n + 1));
		c21 = (*(matrix1 + 1 * n + 0));
		c22 = (*(matrix1 + 1 * n + 1));

		det = c11*c22 - c12*c21;
		(*(matrix2 + 0 * n + 0)) = c22 / det;
		(*(matrix2 + 0 * n + 1)) = -c12 / det;
		(*(matrix2 + 1 * n + 0)) = -c21 / det;
		(*(matrix2 + 1 * n + 1)) = c11 / det;
	}


	return;
}


//=
void matrix_equal(double *matrix1, int m, int n, double *matrix2)
{

for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			(*(matrix2 + i*n+ j)) = (*(matrix1 + i*n + j));
		}
	}

	return;
}





