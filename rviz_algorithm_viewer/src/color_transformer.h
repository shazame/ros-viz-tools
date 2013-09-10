#ifndef COLOR_MANAGER_H
#define COLOR_MANAGER_H

#include <QObject>

namespace Ogre
{
class ColourValue;
}

namespace rviz
{
  class Property;
  class EnumProperty;
  class ColorProperty;
  //class BoolProperty;
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

  void set( float r, float g, float b, float a );

private:

  float r_, b_, g_, a_;

  rviz::EnumProperty  *color_transformer_property_;
  rviz::ColorProperty *flat_color_property_;
};

} // end namespace rviz_algorithm_viewer

#endif // COLOR_MANAGER_H
