#ifndef COLOR_MANAGER_H
#define COLOR_MANAGER_H

#include <QObject>

namespace Ogre
{
class ColourValue;
class Vector3;
}

namespace rviz
{
class Property;
class EnumProperty;
class BoolProperty;
class ColorProperty;
}

namespace rviz_algorithm_viewer
{

class ColorTransformer: public QObject
{
Q_OBJECT
public:
  enum ColorType { COLOR_FLAT, COLOR_AXIS, COLOR_CLUSTER };

  ColorTransformer( rviz::Property* parent_property );
  ~ColorTransformer();

  void updateProperties( ColorType enabled_color_type );

  ColorType         getColorType() const;
  Ogre::ColourValue getFlatColor() const;

  static Ogre::ColourValue getAxisColor(
      Ogre::Vector3 pos,
      Ogre::Vector3 min_pos,
      Ogre::Vector3 max_pos );

  //void set( float r, float g, float b, float a );

private Q_SLOTS:
  void updateUseRainbow();

private:
  void createFlatProperties   ( rviz::Property* grandparent_property, rviz::Property* parent_property );
  void createAxisProperties   ( rviz::Property* grandparent_property, rviz::Property* parent_property );
  void createClusterProperties( rviz::Property* grandparent_property, rviz::Property* parent_property );

  void setHiddenFlatProperties   ( bool hide );
  void setHiddenAxisProperties   ( bool hide );
  void setHiddenClusterProperties( bool hide );

  //float r_, b_, g_, a_;

  rviz::EnumProperty  *color_transformer_property_;

  // Properties for the flat color transformer
  rviz::ColorProperty *flat_color_property_;

  // Properties for the cluster color transformer
  rviz::BoolProperty  *use_rainbow_property_;
  rviz::ColorProperty *min_color_property_;
  rviz::ColorProperty *max_color_property_;
};

} // end namespace rviz_algorithm_viewer

#endif // COLOR_MANAGER_H
