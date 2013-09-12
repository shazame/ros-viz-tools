#ifndef CLUSTER_VISUAL_H
#define CLUSTER_VISUAL_H

#include <vector>

#include <rviz_algorithm_viewer/Cluster2.h>

#include "color_transformer.h"

namespace Ogre
{
class Vector3;
class Quaternion;
class ColourValue;
}

namespace rviz
{
class Shape;
}

namespace rviz_algorithm_viewer
{

// Each instance of ClusterVisual represents the visualization of a single
// rviz_algorithm_viewer::Cluster2 message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class ClusterVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  ClusterVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~ClusterVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const rviz_algorithm_viewer::Cluster2::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way ClusterVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set properties of the visual, which are user-editable
  // parameters and therefore don't come from the cluster message.
  void setAlpha( float a );
  void setFlatColor( float r, float g, float b  );
  void setAxisColor();
  void setClusterColor( const ColorTransformer& color_transformer );
  void updateColorAndAlpha();
  void setRadius( float r );
  void setPointsShow( bool show_points );
  void setClustersShow( bool show_clusters );

private:
  // A SceneNode whose pose is set to match the coordinate frame of
  // the Cluster message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  class ClusterPoints
  {
    public:
      ClusterPoints( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
      ~ClusterPoints();

      void setPointsColor( float r, float g, float b );
      void updatePointsColorAndAlpha();
      void setPointsRadius( float r );

      void addPoint( Ogre::Vector3 position );
      void displayPoints();
      void clearPoints();

      void updateEnvelope();
      const Ogre::Vector3& getPointsCenter();
      void displayEnvelope();
      void clearEnvelope();

    private:
      // points position are always stored when a message is received
      // hence they are separated from the point object
      std::vector<Ogre::Vector3> points_pos_;
      // The center and the radius of the envelope are stored each time a
      // message is received as well. Center position is used to determine the
      // color of the cluster when the Axis Color Transformer is chosen.
      Ogre::Vector3 envelope_center_;
      Ogre::Vector3 envelope_diameter_;

      // points and their envelope are only displayed when the corresponding
      // property is validated, thus the objects are only created when
      // necessary
      typedef boost::shared_ptr<rviz::Shape> PointPtr;
      std::vector<PointPtr> points_;
      boost::shared_ptr<rviz::Shape> envelope_;

      // Color of each cluster needs to be remembered when display is paused
      // and clusters are hide/shown
      boost::shared_ptr<Ogre::ColourValue> color_;

      Ogre::SceneNode* frame_node_;
      Ogre::SceneManager* scene_manager_;
  };

  typedef boost::shared_ptr<ClusterVisual::ClusterPoints> ClusterPointsPtr;

  // The object implementing the actual point cluster
  std::vector<ClusterPointsPtr> clusters_;

  Ogre::Vector3 getPointsPosMin();
  Ogre::Vector3 getPointsPosMax();

  // Global properties applied to every cluster
  static float radius_;
  static float alpha_;
  static bool  show_points_;
  static bool  show_clusters_;
};

} // end namespace rviz_algorithm_viewer

#endif // CLUSTER_VISUAL_H
