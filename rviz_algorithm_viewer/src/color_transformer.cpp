#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreVector3.h>

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
                                                  parent_property, SLOT( updateColorTransformer() ), grandparent_property );

  min_color_property_ = new rviz::ColorProperty( "Min Color", QColor( 255, 255, 255 ),
                                                 "Color to use for the first cluster. Then color is interpolated between this and Max Color.",
                                                 parent_property, SLOT( updateColorTransformer() ), grandparent_property );

  max_color_property_ = new rviz::ColorProperty( "Max Color", QColor( 0, 0, 0 ),
                                                 "Color to use for the last cluster. Before, color is interpolated between Min Color and this.",
                                                 parent_property, SLOT( updateColorTransformer() ), grandparent_property );
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

Ogre::ColourValue ColorTransformer::getAxisColor( Ogre::Vector3 pos, Ogre::Vector3 min_pos, Ogre::Vector3 max_pos )
{
  Ogre::ColourValue color( 
      (pos.x - min_pos.x) / (max_pos.x - min_pos.x),
      (pos.y - min_pos.y) / (max_pos.y - min_pos.y),
      (pos.z - min_pos.z) / (max_pos.z - min_pos.z) );
  return color;
}

Ogre::ColourValue ColorTransformer::getClusterColor( float fraction ) const
{
  Ogre::ColourValue color;

  if ( use_rainbow_property_->getBool() )
  {
    color = ColorTransformer::getRainbowColor( fraction );
  }
  else
  {
    Ogre::ColourValue min_color = min_color_property_->getOgreColor();
    Ogre::ColourValue max_color = max_color_property_->getOgreColor();

    color.r = fraction * max_color.r + (1.0f - fraction) * min_color.r;
    color.g = fraction * max_color.g + (1.0f - fraction) * min_color.g;
    color.b = fraction * max_color.b + (1.0f - fraction) * min_color.b;
  }

  return color;
}

Ogre::ColourValue ColorTransformer::getRainbowColor( float fraction )
{
  Ogre::ColourValue color;

  fraction = std::min(fraction, 1.0f);
  fraction = std::max(fraction, 0.0f);

  float h = fraction * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if ( !(i&1) ) f = 1 -f; // if i si even
  float n = 1 - f;

  if      (i <= 1) color.r = n, color.g = 0, color.b = 1;
  else if (i == 2) color.r = 0, color.g = n, color.b = 1;
  else if (i == 3) color.r = 0, color.g = 1, color.b = n;
  else if (i == 4) color.r = n, color.g = 1, color.b = 0;
  else if (i >= 5) color.r = 1, color.g = n, color.b = 0;

  return color;
}

} // end namespace rviz_algorithm_viewer
