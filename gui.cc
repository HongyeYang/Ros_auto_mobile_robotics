//hongye
#include "gui.h"
#include "iostream"
#include <fstream>
#include <assert.h>

using namespace std;
ofstream labdata2;

std::vector<float> ranges;
double prediction_position[3]={0,0,0};
double prediction_orientation[4]={0,0,0,1};
double covariance[3];
double gui_position[3]={0,0,0},gui_orientation[4]={0,0,0,0};
double goal[3]={0,0,0}, goal_orientation[4]={0,0,0,0};

double centroid[3]={0,0,0};
int rsize=64;


GUI::
GUI( QWidget * parent ) : QGLWidget( parent ), timer() {
setMinimumSize( 600, 600 );
timer = new QTimer( this );
connect( timer, SIGNAL( timeout() ), this, SLOT( timer_callback() ) );
timer->start( 100 ); // call timer_callback at 0.1 Hz (period is 100 ms)
labdata2.open("lab56_actual_position.txt");
 ranges.resize(64);

}

GUI::
~GUI() {
}

void
GUI::
handle_laserscan( const sensor_msgs::LaserScan::ConstPtr& msg ){
// implement storing of laserscan message here
rsize=msg->ranges.size();

ranges.resize(rsize);
ranges=msg->ranges;
angle_min=msg->angle_min;
angle_max=msg->angle_max;
angle_increment=msg->angle_increment;
 
//labdata1<<endl;
//labdata1<<msg->scan_time;


/*for(int i=0;i<rsize;i++)
labdata1<<","<<ranges[i];
labdata1<<";";
*/
update();
return;
}

void
GUI::
handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
// implement storing of robot pose here
gui_position[0]=msg->pose.pose.position.x;
gui_position[1]=msg->pose.pose.position.y;
gui_position[2]=msg->pose.pose.position.z;

gui_orientation[0]=msg->pose.pose.orientation.x;
gui_orientation[1]=msg->pose.pose.orientation.y;
gui_orientation[2]=msg->pose.pose.orientation.z;
gui_orientation[3]=msg->pose.pose.orientation.w;

labdata2<<gui_position[0]<<","<<gui_position[1]<<";"<<endl;


 
return;
}

void GUI::handle_prediction( const geometry_msgs::PoseWithCovariance::ConstPtr& msg ){
//implement storing of prediction
prediction_position[0]=msg->pose.position.x;
prediction_position[1]=msg->pose.position.y;
prediction_position[2]=msg->pose.position.z;

prediction_orientation[0]=msg->pose.orientation.x;
prediction_orientation[1]=msg->pose.orientation.y;
prediction_orientation[2]=msg->pose.orientation.z;
prediction_orientation[3]=msg->pose.orientation.w;

covariance[0]=msg->covariance[0];
covariance[1]=msg->covariance[1];
covariance[2]=msg->covariance[2];


return;
}

void GUI::handle_goal( const geometry_msgs::Pose::ConstPtr& msg )
{
goal[0]=msg->position.x;
goal[1]=msg->position.y;
goal[2]=msg->position.z;

goal_orientation[0]=msg->orientation.x;
goal_orientation[1]=msg->orientation.y;
goal_orientation[2]=msg->orientation.z;
goal_orientation[3]=msg->orientation.w;
}

void
GUI::
timer_callback( void ){
ros::spinOnce(); // Process the messages in here

return;
}

void
GUI::
initializeGL(){
glClearColor( 1.0, 1.0, 1.0, 1.0 );
glMatrixMode( GL_PROJECTION );
gluOrtho2D(-9,9,-8,10);


glMatrixMode( GL_MODELVIEW ); 


return;
}

void
GUI::
paintGL(){
int i;
double yaw;
glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
glLoadIdentity();
// draw a coordinate system at the origin
glBegin( GL_LINES );
glColor4f( 1.0, 0.0, 0.0, 1.0 );
glVertex3f( 0.0, 0.0, 0.0 );
glVertex3f( 1.0, 0.0, 0.0 );
glColor4f( 0.0, 1.0, 0.0, 1.0 );
glVertex3f( 0.0, 0.0, 0.0 );
glVertex3f( 0.0, 1.0, 0.0 );
glColor4f( 0.0, 0.0, 1.0, 1.0 );
glVertex3f( 0.0, 0.0, 0.0 );
glVertex3f( 0.0, 0.0, 1.0 );
glEnd();

//implement drawing of wall
glBegin(GL_LINES);
glColor4f(0.0,0.0,0.0,1.0);
glVertex3f(1.0,0.0,0.0);
glVertex3f(1.0,10.0,0.0);
glEnd();
glBegin(GL_LINES);
glColor4f(0.0,0.0,0.0,1.0);
glVertex3f(-1.0,0.0,0.0);
glVertex3f(-1.0,10.0,0.0);
glEnd();

// implement drawing of laserscan and robot pose here
gui_position[2]=atan2(2*gui_orientation[2]*gui_orientation[3],1-2*gui_orientation[2]*gui_orientation[2]);
prediction_position[2]=atan2(2*prediction_orientation[2]*prediction_orientation[3],1-2*prediction_orientation[2]*prediction_orientation[2]);


//actual_pose

Circle( gui_position[0],gui_position[1],0.1,0.1,10,'r');
//actual orientation
glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-gui_position[1],gui_position[0],0.0);
glVertex3f(-gui_position[1]-0.5*sin(gui_position[2]),gui_position[0]+0.5*cos(gui_position[2]),0.0);
glEnd();
//draw 6 cone fall regions
//cone 1
double RG1,RG2,RG3,RG5,RG6,RG7, AG1,AG2,AG3,AG5,AG6,AG7,fallr=0.3,falla=0.3,ver1x,ver1y,ver2x,ver2y,ver3x,ver3y,ver4x,ver4y;
RG1=sqrt((1-prediction_position[0])*(1-prediction_position[0])+(0-prediction_position[1])*(0-prediction_position[1]));
	AG1=atan2((0-prediction_position[1]),(1-prediction_position[0]))-prediction_position[2];


RG2=sqrt((2-prediction_position[0])*(2-prediction_position[0])+(0-prediction_position[1])*(0-prediction_position[1]));
	AG2=atan2((0-prediction_position[1]),(2-prediction_position[0]))-prediction_position[2];

RG3=sqrt((3-prediction_position[0])*(3-prediction_position[0])+(0-prediction_position[1])*(0-prediction_position[1]));
	AG3=atan2((0-prediction_position[1]),(3-prediction_position[0]))-prediction_position[2];

RG5=sqrt((5-prediction_position[0])*(5-prediction_position[0])+(0-prediction_position[1])*(0-prediction_position[1]));
	AG5=atan2((0-prediction_position[1]),(5-prediction_position[0]))-prediction_position[2];

RG6=sqrt((6-prediction_position[0])*(6-prediction_position[0])+(0-prediction_position[1])*(0-prediction_position[1]));
	AG6=atan2((0-prediction_position[1]),(6-prediction_position[0]))-prediction_position[2];

RG7=sqrt((7-prediction_position[0])*(7-prediction_position[0])+(0-prediction_position[1])*(0-prediction_position[1]));
	AG7=atan2((0-prediction_position[1]),(7-prediction_position[0]))-prediction_position[2];





ver1x=gui_position[0]+(RG1+fallr)*cos(AG1+falla/RG1+gui_position[2]);
ver1y=gui_position[1]+(RG1+fallr)*sin(AG1+falla/RG1+gui_position[2]);


ver2x=gui_position[0]+(RG1+fallr)*cos(AG1-falla/RG1+gui_position[2]);
ver2y=gui_position[1]+(RG1+fallr)*sin(AG1-falla/RG1+gui_position[2]);

ver3x=gui_position[0]+(RG1-fallr)*cos(AG1+falla/RG1+gui_position[2]);
ver3y=gui_position[1]+(RG1-fallr)*sin(AG1+falla/RG1+gui_position[2]);

ver4x=gui_position[0]+(RG1-fallr)*cos(AG1-falla/RG1+gui_position[2]);
ver4y=gui_position[1]+(RG1-fallr)*sin(AG1-falla/RG1+gui_position[2]);

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver2y,ver2x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver2y,ver2x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver3y,ver3x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver3y,ver3x,0.0);
glEnd();

ver1x=gui_position[0]+(RG2+fallr)*cos(AG2+falla/RG2+gui_position[2]);
ver1y=gui_position[1]+(RG2+fallr)*sin(AG2+falla/RG2+gui_position[2]);


ver2x=gui_position[0]+(RG2+fallr)*cos(AG2-falla/RG2+gui_position[2]);
ver2y=gui_position[1]+(RG2+fallr)*sin(AG2-falla/RG2+gui_position[2]);

ver3x=gui_position[0]+(RG2-fallr)*cos(AG2+falla/RG2+gui_position[2]);
ver3y=gui_position[1]+(RG2-fallr)*sin(AG2+falla/RG2+gui_position[2]);

ver4x=gui_position[0]+(RG2-fallr)*cos(AG2-falla/RG2+gui_position[2]);
ver4y=gui_position[1]+(RG2-fallr)*sin(AG2-falla/RG2+gui_position[2]);

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver2y,ver2x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver2y,ver2x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver3y,ver3x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver3y,ver3x,0.0);
glEnd();

ver1x=gui_position[0]+(RG3+fallr)*cos(AG3+falla/RG3+gui_position[2]);
ver1y=gui_position[1]+(RG3+fallr)*sin(AG3+falla/RG3+gui_position[2]);


ver2x=gui_position[0]+(RG3+fallr)*cos(AG3-falla/RG3+gui_position[2]);
ver2y=gui_position[1]+(RG3+fallr)*sin(AG3-falla/RG3+gui_position[2]);

ver3x=gui_position[0]+(RG3-fallr)*cos(AG3+falla/RG3+gui_position[2]);
ver3y=gui_position[1]+(RG3-fallr)*sin(AG3+falla/RG3+gui_position[2]);

ver4x=gui_position[0]+(RG3-fallr)*cos(AG3-falla/RG3+gui_position[2]);
ver4y=gui_position[1]+(RG3-fallr)*sin(AG3-falla/RG3+gui_position[2]);

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver2y,ver2x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver2y,ver2x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver3y,ver3x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver3y,ver3x,0.0);
glEnd();

ver1x=gui_position[0]+(RG5+fallr)*cos(AG5+falla/RG5+gui_position[2]);
ver1y=gui_position[1]+(RG5+fallr)*sin(AG5+falla/RG5+gui_position[2]);


ver2x=gui_position[0]+(RG5+fallr)*cos(AG5-falla/RG5+gui_position[2]);
ver2y=gui_position[1]+(RG5+fallr)*sin(AG5-falla/RG5+gui_position[2]);

ver3x=gui_position[0]+(RG5-fallr)*cos(AG5+falla/RG5+gui_position[2]);
ver3y=gui_position[1]+(RG5-fallr)*sin(AG5+falla/RG5+gui_position[2]);

ver4x=gui_position[0]+(RG5-fallr)*cos(AG5-falla/RG5+gui_position[2]);
ver4y=gui_position[1]+(RG5-fallr)*sin(AG5-falla/RG5+gui_position[2]);

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver2y,ver2x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver2y,ver2x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver3y,ver3x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver3y,ver3x,0.0);
glEnd();

ver1x=gui_position[0]+(RG6+fallr)*cos(AG6+falla/RG6+gui_position[2]);
ver1y=gui_position[1]+(RG6+fallr)*sin(AG6+falla/RG6+gui_position[2]);


ver2x=gui_position[0]+(RG6+fallr)*cos(AG6-falla/RG6+gui_position[2]);
ver2y=gui_position[1]+(RG6+fallr)*sin(AG6-falla/RG6+gui_position[2]);

ver3x=gui_position[0]+(RG6-fallr)*cos(AG6+falla/RG6+gui_position[2]);
ver3y=gui_position[1]+(RG6-fallr)*sin(AG6+falla/RG6+gui_position[2]);

ver4x=gui_position[0]+(RG6-fallr)*cos(AG6-falla/RG6+gui_position[2]);
ver4y=gui_position[1]+(RG6-fallr)*sin(AG6-falla/RG6+gui_position[2]);

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver2y,ver2x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver2y,ver2x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver3y,ver3x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver3y,ver3x,0.0);
glEnd();

ver1x=gui_position[0]+(RG7+fallr)*cos(AG7+falla/RG7+gui_position[2]);
ver1y=gui_position[1]+(RG7+fallr)*sin(AG7+falla/RG7+gui_position[2]);


ver2x=gui_position[0]+(RG7+fallr)*cos(AG7-falla/RG7+gui_position[2]);
ver2y=gui_position[1]+(RG7+fallr)*sin(AG7-falla/RG7+gui_position[2]);

ver3x=gui_position[0]+(RG7-fallr)*cos(AG7+falla/RG7+gui_position[2]);
ver3y=gui_position[1]+(RG7-fallr)*sin(AG7+falla/RG7+gui_position[2]);

ver4x=gui_position[0]+(RG7-fallr)*cos(AG7-falla/RG7+gui_position[2]);
ver4y=gui_position[1]+(RG7-fallr)*sin(AG7-falla/RG7+gui_position[2]);

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver2y,ver2x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver2y,ver2x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver3y,ver3x,0.0);
glVertex3f(-ver4y,ver4x,0.0);
glEnd();

glBegin(GL_LINES);
glColor4f(1.0,0.0,0.0,1.0);
glVertex3f(-ver1y,ver1x,0.0);
glVertex3f(-ver3y,ver3x,0.0);
glEnd();
//predict orientation
glBegin(GL_LINES);
glColor4f(0.0,1.0,0.0,1.0);
glVertex3f(-prediction_position[1],prediction_position[0],0.0);
glVertex3f(-prediction_position[1]-0.5*sin(prediction_position[2]),prediction_position[0]+0.5*cos(prediction_position[2]),0.0);
glEnd();

//prediction_pose

Circle(prediction_position[0],prediction_position[1],0.1,0.1,10,'g');

cout<<gui_position[2]<<" "<<prediction_position[2]<<endl;

//goal point
Circle(goal[0],goal[1],0.05,0.05,10,'h');


//cones
Circle(1.0,0.0,0.057,0.057,10,'b');
Circle(2.0,0.0,0.057,0.057,10,'b');
Circle(3.0,0.0,0.057,0.057,10,'b');
Circle(5.0,0.0,0.057,0.057,10,'b');
Circle(6.0,0.0,0.057,0.057,10,'b');
Circle(7.0,0.0,0.057,0.057,10,'b');


//ranges
yaw=atan2(2*gui_orientation[3]*gui_orientation[2],(1-2*gui_orientation[2]*gui_orientation[2]));
 
//cout<< gui_orientation[0] << ", " << gui_orientation[1]<<","<<gui_orientation[2]<<","<<gui_orientation[3]<<","<<yaw<<endl;
glBegin( GL_LINES );

for(i=0;i<rsize;i++)
{
  

glColor4f(0.0,0.0,0.0,1.0);

glVertex3f( -gui_position[1], gui_position[0], gui_position[2]);
 
 float rx=gui_position[0]+ranges[i]*cos(angle_increment*i+yaw+angle_min);

 double angle=angle_increment*i+yaw+angle_min;
 float ry=gui_position[1]+ranges[i]*sin(angle_increment*i+yaw+angle_min);

 glVertex3f( -ry,rx,0);
double mmm=angle_increment*i+yaw+angle_min;

}

glEnd();

return;
}

void 
GUI::
Circle(float cx, float cy, float rx,float ry, int num_seg, char color){
float jiao=2*3.14159/float(num_seg);


float x=cx+rx*cos(jiao*0),y=cy+ry*sin(jiao*0);

glBegin(GL_LINE_LOOP);
//set color
if(color=='r')
glColor4f(1.0,0.0,0.0,1.0);
else if(color=='g')
glColor4f(0.0,1.0,0.0,1.0);
else if(color=='h')
glColor4f(0.0,0.0,0.0,1.0);
else
glColor4f(0.0,0.0,1.0,1.0);

for(int i1=0;i1<num_seg;i1++){
glVertex2f(-y,x);

x=cx+rx*cos(jiao*i1);
y=cy+ry*sin(jiao*i1);
}
glEnd();

 return;
}




