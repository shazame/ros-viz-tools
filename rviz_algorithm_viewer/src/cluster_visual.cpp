#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include "ros/ros.h"

#include "cluster_visual.h"

namespace rviz_algorithm_viewer
{

ClusterVisual::ClusterVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Cluster's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  cluster_.reset(new rviz::Shape( rviz::Shape::Sphere, scene_manager_, frame_node_));
}

ClusterVisual::~ClusterVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void ClusterVisual::setMessage( const rviz_algorithm_viewer::Cluster2::ConstPtr& msg )
{
  Ogre::Vector3 scale( radius_, radius_, radius_ );
  cluster_->setScale( scale );

  // Convert the geometry_msgs::Point to an Ogre::Vector3
  Ogre::Vector3 pos( 
      msg->clusters[0].points[0].x, 
      msg->clusters[0].points[0].y, 
      msg->clusters[0].points[0].z );

  cluster_->setPosition( pos );
}

// Position and orientation are passed through to the SceneNode.
void ClusterVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void ClusterVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void ClusterVisual::setColor( float r, float g, float b, float a )
{
  cluster_->setColor( r, g, b, a );
}

void ClusterVisual::setRadius( float r )
{
  radius_ = r;
}

} // end namespace rviz_algorithm_viewer
