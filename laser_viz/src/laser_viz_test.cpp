// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/LaserScan.h"

#include <cmath>

ros::Publisher obstacle_pub;

void obstacleHandleCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  obstacle_pub.publish(msg);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "laser_viz_obstacles");

  ros::NodeHandle n;

  obstacle_pub = n.advertise<sensor_msgs::LaserScan>("visualization_obstacle", 10);

  ros::Subscriber sub = n.subscribe("scan", 1000, obstacleHandleCallback);

  ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%
