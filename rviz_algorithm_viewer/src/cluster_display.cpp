#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "cluster_visual.h"
#include "cluster_display.h"

namespace rviz_algorithm_viewer
{

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ClusterDisplay::ClusterDisplay()
{
  pause_display_property_ = new rviz::BoolProperty( "Pause", false, 
                                                    "Stop processing new messages to pause the display.",
                                                    this );

  show_points_property_ = new rviz::BoolProperty( "Show points", false, 
                                                  "Show every points.",
                                                  this, SLOT( updatePointsAndClusters() ));

  show_clusters_property_ = new rviz::BoolProperty( "Show clusters", true, 
                                                    "Show a bounding sphere of the points of each cluster.",
                                                    this, SLOT( updatePointsAndClusters() ));

  radius_property_ = new rviz::FloatProperty( "Radius", 0.2,
                                              "Radius of a point.",
                                              this, SLOT( updateRadius() ));
  radius_property_->setMin( 0.0001 );

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  color_transformer_ = new ColorTransformer( this );

  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 100000 );
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void ClusterDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

ClusterDisplay::~ClusterDisplay()
{
}

// Clear the visuals by deleting their objects.
void ClusterDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Enable or disable points and clusters display
void ClusterDisplay::updatePointsAndClusters()
{
  bool show_points   = show_points_property_->getBool();
  bool show_clusters = show_clusters_property_->getBool();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setPointsShow( show_points );
    visuals_[ i ]->setClustersShow( show_clusters );
    visuals_[ i ]->updateColorAndAlpha();
  }
}

// Set the current alpha values for each visual.
void ClusterDisplay::updateAlpha()
{
  float alpha  = alpha_property_->getFloat();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setAlpha( alpha );
    visuals_[ i ]->updateColorAndAlpha();
  }
}

// Set the current color values for each visual.
void ClusterDisplay::updateColor()
{
  Ogre::ColourValue color = color_transformer_->getFlatColor();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setColor( color.r, color.g, color.b );
    visuals_[ i ]->updateColorAndAlpha();
  }
}

// Set the current radius value for each visual.
void ClusterDisplay::updateRadius()
{
  float radius = radius_property_->getFloat();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setRadius( radius );
  }
}

void ClusterDisplay::updateColorTransformer()
{
  ColorTransformer::ColorType enabled_color_type = color_transformer_->getColorType();

  color_transformer_->updateProperties( enabled_color_type );

  switch ( enabled_color_type )
  {
    COLOR_FLAT:
      break;
    COLOR_AXIS:
      break;
    COLOR_CLUSTER:
      break;
    default:
      break;
  }
}

// Set the number of past visuals to show.
void ClusterDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// This is our callback to handle an incoming message.
void ClusterDisplay::processMessage( const rviz_algorithm_viewer::Cluster2::ConstPtr& msg )
{
  // When pause is activated, messages are not processed
  if ( pause_display_property_->getBool() )
  {
    return;
  }

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Cluster message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<ClusterVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new ClusterVisual( context_->getSceneManager(), scene_node_ ));
  }

  // Now set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );
  initProperties( visual );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

void ClusterDisplay::initProperties(boost::shared_ptr<ClusterVisual> visual)
{
  bool show_points   = show_points_property_->getBool();
  visual->setPointsShow( show_points );

  bool show_clusters = show_clusters_property_->getBool();
  visual->setClustersShow( show_clusters );

  float alpha = alpha_property_->getFloat();
  visual->setAlpha( alpha );
  Ogre::ColourValue color = color_transformer_->getFlatColor();
  visual->setColor( color.r, color.g, color.b );
  visual->updateColorAndAlpha();

  float radius = radius_property_->getFloat();
  visual->setRadius( radius );
}

} // end namespace rviz_algorithm_viewer

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_algorithm_viewer::ClusterDisplay,rviz::Display )
