#ifndef SQUARE_MARKER_HPP
#define SQUARE_MARKER_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class SquareMarkerArray {
  private:
    visualization_msgs::MarkerArray squares;
    const char * frame_id, * ns;
    uint32_t ID;

    void SquareInit(visualization_msgs::Marker &square); 

  public:
    SquareMarkerArray(const char * frame_id, const char * ns);
    ~SquareMarkerArray();

    /* top left and bottom right coordinates */
    void add(float tl_x, float tl_y, float br_x, float br_y); 

    void add(float x, float y, float size);

    void pub(const ros::Publisher &);
};

#endif /* SQUARE_MARKER_HPP */
