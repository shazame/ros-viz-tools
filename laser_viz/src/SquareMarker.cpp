#include "SquareMarker.hpp"

SquareMarker::SquareMarker(const ros::Publisher & pub, const char * frame_id, const char * ns): publisher(pub){
  edges.header.frame_id = frame_id;
  edges.header.stamp = ros::Time::now();
  edges.ns = ns;
  edges.action = visualization_msgs::Marker::ADD;
  edges.pose.orientation.w = 1.0;

  edges.id = 0;

  edges.type = visualization_msgs::Marker::LINE_STRIP;

  edges.color.g = 1.0f;
  edges.color.a = 1.0;

  edges.scale.x = 0.02;
}

SquareMarker::~SquareMarker() {}

void SquareMarker::draw(float tl_x, float tl_y, float br_x, float br_y) {
  geometry_msgs::Point p;

  // top left point
  p.x = tl_x; p.y = tl_y;
  edges.points.push_back(p);

  // top right point
  p.x = tl_x; p.y = br_y;
  edges.points.push_back(p);

  // bottom right point
  p.x = br_x; p.y = br_y;
  edges.points.push_back(p);

  // bottom left point
  p.x = br_x; p.y = tl_y;
  edges.points.push_back(p);

  // top left point to complete the "circle"
  p.x = tl_x; p.y = tl_y;
  edges.points.push_back(p);

  publisher.publish(edges);

  edges.points.clear();

  // This object is used to create several markers under the same namespace
  // A unique id prevent new markers to replace old ones
  edges.id++;
}

void SquareMarker::draw(float x, float y, float size) {
  draw(x+size/2., y+size/2., x-size/2., y-size/2.);
}
