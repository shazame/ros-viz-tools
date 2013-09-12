#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <geometry_msgs/Point.h>
#include <rviz_algorithm_viewer/ClusterField.h>

#include <cmath>

#include "cluster_visual.h"
#include "color_transformer.h"

#include "Miniball.hpp"

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
}

ClusterVisual::~ClusterVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void ClusterVisual::setMessage( const rviz_algorithm_viewer::Cluster2::ConstPtr& msg )
{
  // Remove all the previous points and clusters from the display
  clusters_.clear();

  // Add every cluster of the message to the display
  std::vector<rviz_algorithm_viewer::ClusterField>::const_iterator clust_it  = msg->clusters.begin();
  std::vector<rviz_algorithm_viewer::ClusterField>::const_iterator clust_end = msg->clusters.end();
  for (; clust_it != clust_end; ++clust_it)
  {
    ClusterPointsPtr cluster_ptr(new ClusterPoints( scene_manager_, frame_node_ ));

    // Add to this cluster every points found in the message
    std::vector<geometry_msgs::Point>::const_iterator pts_it  = (*clust_it).points.begin();
    std::vector<geometry_msgs::Point>::const_iterator pts_end = (*clust_it).points.end();
    //float red, green, blue;
    for (; pts_it != pts_end; ++pts_it)
    {
      Ogre::Vector3 pos( 
          (*pts_it).x, 
          (*pts_it).y, 
          (*pts_it).z );

      cluster_ptr->addPoint( pos );

      //red   = (pos.x + 20) / 40.;
      //green = (pos.y + 20) / 40.;
      //blue  = (pos.z + 20) / 40.;
    }

    cluster_ptr->updateEnvelope();

    if ( show_points_ ) 
    {
      cluster_ptr->displayPoints();
    }

    if ( show_clusters_ ) 
    {
      cluster_ptr->displayEnvelope();
    }
    //cluster_ptr->setColor( red, green, blue, 0.5 );
    clusters_.push_back( cluster_ptr );
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

// Radius is passed through to all the clusters.
void ClusterVisual::setRadius( float r )
{
  radius_ = r;

  std::vector<ClusterPointsPtr>::iterator it = clusters_.begin();
  std::vector<ClusterPointsPtr>::iterator end = clusters_.end();
  for (; it != end; ++it)
  {
    (*it)->setPointsRadius( radius_ );
  }
}

void ClusterVisual::setAlpha( float a )
{
  alpha_ = a;
}

// Color is passed through to all the clusters.
void ClusterVisual::setFlatColor( float r, float g, float b )
{
  std::vector<ClusterPointsPtr>::iterator it = clusters_.begin();
  std::vector<ClusterPointsPtr>::iterator end = clusters_.end();
  for (; it != end; ++it)
  {
    (*it)->setPointsColor( r, g, b );
  }
}

// Color is computed for all the clusters based on their position.
void ClusterVisual::setAxisColor()
{
  // Get min/max x, y and z position of the clusters
  Ogre::Vector3 points_min_pos = getPointsPosMin();
  Ogre::Vector3 points_max_pos = getPointsPosMax();

  std::vector<ClusterPointsPtr>::iterator it;
  // Set the color of the cluster based on their positin
  for (it = clusters_.begin(); it != clusters_.end(); ++it)
  {
    Ogre::ColourValue color = ColorTransformer::getAxisColor(
        (*it)->getPointsCenter(),
        points_min_pos,
        points_max_pos );

    (*it)->setPointsColor( color.r, color.g, color.b );
  }
}

// Color is updated through to all the clusters.
void ClusterVisual::updateColorAndAlpha()
{
  std::vector<ClusterPointsPtr>::iterator it = clusters_.begin();
  std::vector<ClusterPointsPtr>::iterator end = clusters_.end();
  for (; it != end; ++it)
  {
    (*it)->updatePointsColorAndAlpha();
  }
}

// Property state is save and points are instantaneously displayed or removed
void ClusterVisual::setPointsShow( bool show_points )
{
  // Do not recreate points if already existing
  if ( show_points_ == show_points )
  {
    return;
  }

  show_points_ = show_points;

  std::vector<ClusterPointsPtr>::iterator it = clusters_.begin();
  std::vector<ClusterPointsPtr>::iterator end = clusters_.end();
  if ( show_points_ )
  {
    for (; it != end; ++it)
    {
      (*it)->displayPoints();
    }
  }
  else
  {
    for (; it != end; ++it)
    {
      (*it)->clearPoints();
    }
  }
}

// Property state is save and envelope are instantaneously displayed or removed
void ClusterVisual::setClustersShow( bool show_clusters )
{
  // Do not recreate clusters if already existing
  if ( show_clusters_ == show_clusters )
  {
    return;
  }

  show_clusters_ = show_clusters;

  std::vector<ClusterPointsPtr>::iterator it = clusters_.begin();
  std::vector<ClusterPointsPtr>::iterator end = clusters_.end();
  if ( show_clusters_ )
  {
    for (; it != end; ++it)
    {
      (*it)->displayEnvelope();
    }
  }
  else
  {
    for (; it != end; ++it)
    {
      (*it)->clearEnvelope();
    }
  }
}

Ogre::Vector3 ClusterVisual::getPointsPosMin()
{
  Ogre::Vector3 min_pos;

  std::vector<ClusterPointsPtr>::iterator it = clusters_.begin();
  min_pos = (*it)->getPointsCenter();

  for (; it != clusters_.end(); ++it)
  {
    Ogre::Vector3 pos = (*it)->getPointsCenter();

    min_pos.x = ( pos.x < min_pos.x )? pos.x: min_pos.x;
    min_pos.y = ( pos.y < min_pos.y )? pos.y: min_pos.y;
    min_pos.z = ( pos.z < min_pos.z )? pos.z: min_pos.z;
  }

  return min_pos;
}

Ogre::Vector3 ClusterVisual::getPointsPosMax()
{
  Ogre::Vector3 max_pos;

  std::vector<ClusterPointsPtr>::iterator it = clusters_.begin();
  max_pos = (*it)->getPointsCenter();

  for (; it != clusters_.end(); ++it)
  {
    Ogre::Vector3 pos = (*it)->getPointsCenter();

    max_pos.x = ( pos.x > max_pos.x )? pos.x: max_pos.x;
    max_pos.y = ( pos.y > max_pos.y )? pos.y: max_pos.y;
    max_pos.z = ( pos.z > max_pos.z )? pos.z: max_pos.z;
  }

  return max_pos;
}

float ClusterVisual::radius_ = 0.2;
float ClusterVisual::alpha_ = 1.0;
bool  ClusterVisual::show_points_   = false;
bool  ClusterVisual::show_clusters_ = true;


ClusterVisual::ClusterPoints::ClusterPoints( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

ClusterVisual::ClusterPoints::~ClusterPoints()
{
  scene_manager_->destroySceneNode( frame_node_ );
}

void ClusterVisual::ClusterPoints::addPoint( Ogre::Vector3 position )
{
  points_pos_.push_back( position );
}

void ClusterVisual::ClusterPoints::updateEnvelope()
{
  // Convert vector of Ogre::Vector3 to vector of array 
  // to allow iteration over coordinates
  std::vector<float*> vp;
  std::vector<Ogre::Vector3>::iterator it = points_pos_.begin();
  std::vector<Ogre::Vector3>::iterator end = points_pos_.end();
  for (; it != end; ++it)
  {
    // The coordinates are stored in an array
    float* p = new float[3];
    p[0] = (*it).x;
    p[1] = (*it).y;
    p[2] = (*it).z;

    vp.push_back(p);
  }

  // Define the types of iterators through the points and their coordinates
  typedef std::vector<float*>::const_iterator PointIterator;
  typedef const float* CoordIterator;

  // Compute bounding sphere center and radius
  typedef Miniball::
    Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> >
    MB;
  MB mb ( 3, vp.begin(), vp.end() ); // 3 is the dimension of the points

  // Convert Miniball position and diameter into Ogre::Vector3
  float d = 2 * std::sqrt( mb.squared_radius() );
  envelope_diameter_ = Ogre::Vector3( d, d, d );

  const float* center = mb.center();
  float x = *(center++);
  float y = *(center++);
  float z = *center;
  envelope_center_ = Ogre::Vector3( x, y, z );

  // Clean up
  for (std::vector<float*>::iterator it = vp.begin(); it != vp.end(); ++it) {
    delete[] *it;
  }
}

const Ogre::Vector3& ClusterVisual::ClusterPoints::getPointsCenter()
{
  return envelope_center_;
}

// Color is passed through to all the Shape objects.
void ClusterVisual::ClusterPoints::setPointsColor( float r, float g, float b )
{
  color_.reset( new Ogre::ColourValue( r, g, b ) );
}

// Color is updated through to all the Shape objects.
void ClusterVisual::ClusterPoints::updatePointsColorAndAlpha()
{
  if ( show_points_ )
  {
    std::vector<PointPtr>::iterator it = points_.begin();
    std::vector<PointPtr>::iterator end = points_.end();
    for (; it != end; ++it)
    {
      (*it)->setColor( color_->r, color_->g, color_->b, alpha_ );
    }
  }

  if ( show_clusters_ )
  {
    envelope_->setColor( color_->r, color_->g, color_->b, alpha_ );
  }
}

// Radius is passed through to all the Shape objects.
void ClusterVisual::ClusterPoints::setPointsRadius( float r )
{
  Ogre::Vector3 scale( r, r, r );

  if ( show_points_ )
  {
    std::vector<PointPtr>::iterator it = points_.begin();
    std::vector<PointPtr>::iterator end = points_.end();
    for (; it != end; ++it)
    {
      (*it)->setScale( scale );
    }
  }
}

void ClusterVisual::ClusterPoints::displayPoints()
{
  std::vector<Ogre::Vector3>::iterator pos_it = points_pos_.begin();
  std::vector<Ogre::Vector3>::iterator pos_end = points_pos_.end();
  for (; pos_it != pos_end; ++pos_it)
  {
    // We create the sphere object within the frame node so that we can
    // set its position and direction relative to its header frame.
    PointPtr pts(new rviz::Shape( rviz::Shape::Sphere, scene_manager_, frame_node_));

    Ogre::Vector3 scale( radius_, radius_, radius_ );
    pts->setScale( scale );
    pts->setPosition( *pos_it );

    points_.push_back( pts );
  }
}

void ClusterVisual::ClusterPoints::clearPoints()
{
  points_.clear();
}

void ClusterVisual::ClusterPoints::displayEnvelope()
{
  envelope_.reset( new rviz::Shape( rviz::Shape::Sphere, scene_manager_, frame_node_) );

  // Update envelope with computed diameter and position
  envelope_->setScale( envelope_diameter_ );
  envelope_->setPosition( envelope_center_ );
}

void ClusterVisual::ClusterPoints::clearEnvelope()
{
  envelope_.reset();
}

} // end namespace rviz_algorithm_viewer
