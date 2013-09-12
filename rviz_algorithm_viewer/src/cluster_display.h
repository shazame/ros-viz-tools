#ifndef CLUSTER_DISPLAY_H
#define CLUSTER_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <rviz_algorithm_viewer/Cluster2.h>
#include <rviz/message_filter_display.h>

#include "color_transformer.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
}

namespace rviz_algorithm_viewer
{

class ClusterVisual;

class ClusterDisplay: public rviz::MessageFilterDisplay<rviz_algorithm_viewer::Cluster2>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ClusterDisplay();
  virtual ~ClusterDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updatePointsAndClusters();
  void updateRadius();
  void updateAlpha();

  void updateColorTransformer();
  void updateFlatColor();
  void updateAxisColor();
  void updateClusterColor();

  void updateHistoryLength();

  // Function to handle an incoming ROS message.
private:
  void processMessage( const rviz_algorithm_viewer::Cluster2::ConstPtr& msg );

  void initProperties(boost::shared_ptr<ClusterVisual> visual);

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<ClusterVisual> > visuals_;

  // User-editable property variables.
  rviz::BoolProperty  *pause_display_property_;
  rviz::BoolProperty  *show_points_property_, *show_clusters_property_;
  rviz::FloatProperty *alpha_property_, *radius_property_;
  rviz::IntProperty   *history_length_property_;

  ColorTransformer *color_transformer_;
};

} // end namespace rviz_algorithm_viewer

#endif // CLUSTER_DISPLAY_H
