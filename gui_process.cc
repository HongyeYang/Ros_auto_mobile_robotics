#include <QtGui/QApplication>
#include "gui.h"
#include "gui_process_cmdline.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"

using namespace std;


int
main( int argc,
char* argv[] ){
gengetopt_args_info args;
cmdline_parser( argc, argv, &args );
QApplication app( argc, argv );
ros::init( argc, argv, "gui" );

 
ros::NodeHandle node_handle;
GUI gui;
 

ros::Subscriber subscriber_position = node_handle.subscribe( "/odom", 1,&GUI::handle_odom,&gui );

ros::Subscriber subscriber_scan = node_handle.subscribe( "/scan", 1, &GUI::handle_laserscan, &gui );

ros::Subscriber subscriber_prediction = node_handle.subscribe( "/prediction",1,&GUI::handle_prediction,&gui);

ros::Subscriber subscriber_goal = node_handle.subscribe( "/goal",1,&GUI::handle_goal,&gui);

gui.show();
return app.exec();
}

