#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <geometry_msgs/Point.h>

#include "ros/ros.h"

#include "cluster_visual.h"

namespace rviz_algorithm_viewer
{

ClusterVisual::ClusterPoints::ClusterPoints( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

ClusterVisual::ClusterPoints::~ClusterPoints()
{
  //cluster_.clear();
  scene_manager_->destroySceneNode( frame_node_ );
}

void ClusterVisual::ClusterPoints::clear()
{
  points.clear();
}

void ClusterVisual::ClusterPoints::addPoint( Ogre::Vector3 position )
{
  // We create the sphere object within the frame node so that we can
  // set its position and direction relative to its header frame.
  PointPtr pts(new rviz::Shape( rviz::Shape::Sphere, scene_manager_, frame_node_));

  Ogre::Vector3 scale( radius_, radius_, radius_ );
  pts->setScale( scale );
  pts->setPosition( position );

  points.push_back( pts );
}
void ClusterVisual::ClusterPoints::setColor( float r, float g, float b, float a )
{
  std::vector<PointPtr>::iterator it = points.begin();
  std::vector<PointPtr>::iterator end = points.end();
  for (; it != end; ++it)
  {
    (*it)->setColor( r, g, b, a );
  }
}

void ClusterVisual::ClusterPoints::setRadius( float r )
{
  radius_ = r;
}

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

  clusters_.reset(new ClusterPoints( scene_manager_, frame_node_ ));
}

ClusterVisual::~ClusterVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void ClusterVisual::setMessage( const rviz_algorithm_viewer::Cluster2::ConstPtr& msg )
{
  clusters_->clear();

  std::vector<geometry_msgs::Point>::const_iterator it  = msg->clusters[0].points.begin();
  std::vector<geometry_msgs::Point>::const_iterator end = msg->clusters[0].points.end();
  for (; it != end; ++it)
  {
    Ogre::Vector3 pos( 
        (*it).x, 
        (*it).y, 
        (*it).z );

    clusters_->addPoint( pos );
  }
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

// Color is passed through to the Shape object.
void ClusterVisual::setColor( float r, float g, float b, float a )
{
  clusters_->setColor( r, g, b, a );
}

void ClusterVisual::setRadius( float r )
{
  clusters_->setRadius(r);
}

} // end namespace rviz_algorithm_viewer
