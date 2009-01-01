#ifndef SQUARE_MARKER_HPP
#define SQUARE_MARKER_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class SquareMarker {
  private:
    const ros::Publisher & publisher;
    visualization_msgs::Marker edges;

  public:
    SquareMarker(const ros::Publisher & pub, const char * frame_id, const char * ns);
    ~SquareMarker();

    /* top left and bottom right coordinates */
    void draw(float tl_x, float tl_y, float br_x, float br_y); 

    void draw(float x, float y, float size);
};

#endif /* SQUARE_MARKER_HPP */
