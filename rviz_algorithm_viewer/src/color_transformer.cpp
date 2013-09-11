#include <OGRE/OgreColourValue.h>

#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>

#include "color_transformer.h"

#include <ros/ros.h>

namespace rviz_algorithm_viewer
{

ColorTransformer::ColorTransformer( rviz::Property* parent_property )
{
  color_transformer_property_ = new rviz::EnumProperty( "Color Transformer", "",
                                                        "Set the transformer to use to set the color of the points.",
                                                        parent_property, SLOT( updateColorTransformer() ));

  color_transformer_property_->addOption( "FlatColor"   , COLOR_FLAT );
  color_transformer_property_->addOption( "AxisColor"   , COLOR_AXIS );
  color_transformer_property_->addOption( "ClusterColor", COLOR_CLUSTER );

  flat_color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the clusters.",
                                             color_transformer_property_, SLOT( updateFlatColor() ), parent_property );

}

ColorTransformer::~ColorTransformer()
{
}

void ColorTransformer::updateProperties( ColorType enabled_color_type )
{
  switch ( enabled_color_type )
  {
    case COLOR_FLAT:
      flat_color_property_->setHidden( false );
      color_transformer_property_->expand();
      break;
    case COLOR_AXIS:
      flat_color_property_->setHidden( true );
      break;
    case COLOR_CLUSTER:
      flat_color_property_->setHidden( true );
      break;
    default:
      ROS_DEBUG( "Error setting the color type." );
      break;
  }
}

ColorTransformer::ColorType ColorTransformer::getColorType() const
{
  return (ColorTransformer::ColorType) color_transformer_property_->getOptionInt();
}

Ogre::ColourValue ColorTransformer::getFlatColor() const
{
  return flat_color_property_->getOgreColor();
}

void ColorTransformer::set( float r, float g, float b, float a )
{
}

} // end namespace rviz_algorithm_viewer
