#include <OGRE/OgreColourValue.h>

#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
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
  createFlatProperties( parent_property, color_transformer_property_ );

  color_transformer_property_->addOption( "AxisColor"   , COLOR_AXIS );
  createAxisProperties( parent_property, color_transformer_property_ );

  color_transformer_property_->addOption( "ClusterColor", COLOR_CLUSTER );
  createClusterProperties( parent_property, color_transformer_property_ );
}

void ColorTransformer::createFlatProperties( rviz::Property* grandparent_property, rviz::Property* parent_property )
{
  flat_color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                                  "Color to draw the clusters.",
                                                  parent_property, SLOT( updateFlatColor() ), grandparent_property );
}

void ColorTransformer::createAxisProperties( rviz::Property* grandparent_property, rviz::Property* parent_property )
{

}

void ColorTransformer::createClusterProperties( rviz::Property* grandparent_property, rviz::Property* parent_property )
{
  use_rainbow_property_ = new rviz::BoolProperty( "Use rainbow", true,
                                                  "Use a rainbow to color the clusters or interpolate between two.",
                                                  parent_property, SLOT( updateUseRainbow() ), this);

  min_color_property_ = new rviz::ColorProperty( "Min Color", QColor( 255, 255, 255 ),
                                                 "Color to use for the first cluster. Then color is interpolated between this and Max Color.",
                                                 parent_property );

  max_color_property_ = new rviz::ColorProperty( "Max Color", QColor( 0, 0, 0 ),
                                                 "Color to use for the last cluster. Before, color is interpolated between Min Color and this.",
                                                 parent_property );
}

void ColorTransformer::updateUseRainbow()
{
  bool hide_range_color = use_rainbow_property_->getBool();

  min_color_property_->setHidden( hide_range_color );
  max_color_property_->setHidden( hide_range_color );
}

void ColorTransformer::setHiddenFlatProperties( bool hide )
{
  flat_color_property_->setHidden( hide );
}

void ColorTransformer::setHiddenAxisProperties( bool hide )
{

}

void ColorTransformer::setHiddenClusterProperties( bool hide )
{
  use_rainbow_property_->setHidden( hide );

  if ( use_rainbow_property_->getBool() )
  {
    hide = true;
  }

  min_color_property_->setHidden( hide );
  max_color_property_->setHidden( hide );
}

ColorTransformer::~ColorTransformer()
{
}

void ColorTransformer::updateProperties( ColorType enabled_color_type )
{
  setHiddenFlatProperties   ( enabled_color_type != COLOR_FLAT    );
  setHiddenAxisProperties   ( enabled_color_type != COLOR_AXIS    );
  setHiddenClusterProperties( enabled_color_type != COLOR_CLUSTER );

  color_transformer_property_->expand();
}

ColorTransformer::ColorType ColorTransformer::getColorType() const
{
  return (ColorTransformer::ColorType) color_transformer_property_->getOptionInt();
}

Ogre::ColourValue ColorTransformer::getFlatColor() const
{
  return flat_color_property_->getOgreColor();
}

//void ColorTransformer::set( float r, float g, float b, float a )
//{
//}

} // end namespace rviz_algorithm_viewer
