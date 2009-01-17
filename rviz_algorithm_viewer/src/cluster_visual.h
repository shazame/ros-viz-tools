#ifndef CLUSTER_VISUAL_H
#define CLUSTER_VISUAL_H

#include <vector>

#include <rviz_algorithm_viewer/Cluster2.h>

namespace Ogre
{
class Vector3;
class Quaternion;
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
  class ClusterPoints
  {
    public:
      ClusterPoints( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
      ~ClusterPoints();

      void clear();
      void addPoint( Ogre::Vector3 position );
      void setColor( float r, float g, float b, float a );

    private:
      typedef boost::shared_ptr<rviz::Shape> PointPtr;
      std::vector<PointPtr> points;

      Ogre::SceneNode* frame_node_;
      Ogre::SceneManager* scene_manager_;
  };

  typedef boost::shared_ptr<ClusterVisual::ClusterPoints> ClusterPointsPtr;

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

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the cluster message.
  void setColor( float r, float g, float b, float a );

  void setRadius( float r );

private:
  // The object implementing the actual point cluster
  std::vector<ClusterPointsPtr> clusters_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Cluster message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  static float radius_;
};

} // end namespace rviz_algorithm_viewer

#endif // CLUSTER_VISUAL_H
