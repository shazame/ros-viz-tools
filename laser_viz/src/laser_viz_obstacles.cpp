// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/LaserScan.h"

#include <cmath>

ros::Publisher obstacle_pub ;

void obstacleHandleCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
// %Tag(MARKER_INIT)%
  visualization_msgs::Marker obstacle, line_list;
  obstacle.header.frame_id = line_list.header.frame_id = "/laser";
  obstacle.header.stamp = line_list.header.stamp = ros::Time::now();
  obstacle.ns = line_list.ns = "laser_viz_obstacles";
  obstacle.action = line_list.action = visualization_msgs::Marker::ADD;
  obstacle.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

  obstacle.id = 0;
  line_list.id = 1;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
  obstacle.type = visualization_msgs::Marker::POINTS;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
// %EndTag(TYPE)%

// %Tag(SCALE)%
  obstacle.scale.x = 0.05;
  obstacle.scale.y = 0.05;

  line_list.scale.x = 0.02;
// %EndTag(SCALE)%

// %Tag(COLOR)%
    // Points are green
  obstacle.color.g = 1.0f;
  obstacle.color.a = 1.0;

  // Lines are blue
  line_list.color.b = 1.0f;
  line_list.color.a = 1.0; 
// %EndTag(COLOR)%
  
// %Tag(SCAN)%
  uint32_t i = 0;
  geometry_msgs::Point origin;
  origin.x = 0; origin.y = 0; origin.z = 0;

#if 0
  float max_range = 0;
  float max_range_angle = 0;
#endif

  for (float theta = msg->angle_min; theta < msg->angle_max; theta += msg->angle_increment) {
#if 0
	if (max_range < msg->ranges[i]) {
		max_range = msg->ranges[i];
		max_range_angle = theta;
	}
#else
    float r = msg->ranges[i];
    float x = r * cos(theta);     
    float y = r * sin(theta); 

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;

    obstacle.points.push_back(p);
	if (i % 10 == 0) {
		line_list.points.push_back(origin);
		line_list.points.push_back(p);
	}
#endif
    i++;
  }
#if 0
    float x = max_range * cos(max_range_angle);     
    float y = max_range * sin(max_range_angle); 

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;

    obstacle.points.push_back(p);
    line_list.points.push_back(origin);
    line_list.points.push_back(p);
#endif
// %EndTag(SCAN)%

// %Tag(PUBLISH)%
  obstacle_pub.publish(obstacle);
  obstacle_pub.publish(line_list);
// %EndTag(PUBLISH)%
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "laser_viz_obstacles");

  ros::NodeHandle n;

  obstacle_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacle", 100);

  ros::Subscriber sub = n.subscribe("scan", 100, obstacleHandleCallback);

  ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%
