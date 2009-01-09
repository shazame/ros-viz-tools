#include "SquareMarkerArray.hpp"

SquareMarkerArray::SquareMarkerArray(const char * frame_id, const char * ns): 
  frame_id(frame_id), ns(ns), ID(0){}

SquareMarkerArray::~SquareMarkerArray() {}

void SquareMarkerArray::SquareInit(visualization_msgs::Marker &square) {
  square.header.frame_id = frame_id;
  square.header.stamp = ros::Time::now();
  square.ns = ns;
  square.action = visualization_msgs::Marker::ADD;
  square.pose.orientation.w = 1.0;

  square.id = ID++;

  square.type = visualization_msgs::Marker::LINE_STRIP;

  square.color.g = 1.0f;
  square.color.a = 1.0;

  square.scale.x = 0.02;
}

void SquareMarkerArray::add(float tl_x, float tl_y, float br_x, float br_y) {
  visualization_msgs::Marker square;
  SquareInit(square);

  geometry_msgs::Point p;

  // top left point
  p.x = tl_x; p.y = tl_y;
  square.points.push_back(p);

  // top right point
  p.x = tl_x; p.y = br_y;
  square.points.push_back(p);

  // bottom right point
  p.x = br_x; p.y = br_y;
  square.points.push_back(p);

  // bottom left point
  p.x = br_x; p.y = tl_y;
  square.points.push_back(p);

  // top left point to complete the "circle"
  p.x = tl_x; p.y = tl_y;
  square.points.push_back(p);

  squares.markers.push_back(square);

  //squares.points.clear();

  // This object is used to create several markers under the same namespace
  // A unique id prevent new markers to replace old ones
  //squares.id++;
}

void SquareMarkerArray::add(float x, float y, float size) {
  add(x+size/2., y+size/2., x-size/2., y-size/2.);
}

void SquareMarkerArray::pub(const ros::Publisher & publisher) {
  publisher.publish(squares);
}
