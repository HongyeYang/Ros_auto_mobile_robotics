//hongye
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <QtGui/QApplication>
#include <QtGui/QWidget>
#include <QtOpenGL/QGLWidget>
#include <cstdlib>
#include <QtCore>
#include <QTimer>
#include <GL/glu.h>
#include <vector>
#include <stdlib.h>
#include <string>

using namespace std;
using namespace ros;



class GUI: public QGLWidget {
Q_OBJECT
public:
GUI( QWidget * parent = NULL );

virtual ~GUI();
void handle_laserscan( const sensor_msgs::LaserScan::ConstPtr& msg );
void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
void handle_prediction( const geometry_msgs::PoseWithCovariance::ConstPtr& msg );
void handle_goal( const geometry_msgs::Pose::ConstPtr& msg );
void Circle(float cx, float cy, float rx,float ry, int num_seg, char color);



double gui_resolution,angle_min,angle_max,angle_increment;



QTimer* timer;
protected slots:
void timer_callback( void );
protected:
virtual void initializeGL();
virtual void paintGL();
};
