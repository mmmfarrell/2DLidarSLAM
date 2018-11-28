#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include "robot.h"
#include "robotmapper.h"

namespace robo
{
RobotMapper::RobotMapper(Robot *robot_ptr)
  : robot_ptr_{ robot_ptr }
  , map_image_(map_width_pixels_, map_height_pixels_, QImage::Format_Grayscale8)
{
  this->resetMap();
}

void RobotMapper::getMap(QImage& map)
{
  map = map_image_;
}

void RobotMapper::updateMap(const std::vector<float>& laser_scan)
{
  for (unsigned int i{ 0 }; i < laser_scan.size(); i++)
  {
    if (laser_scan[i] > max_laser_depth_)
      continue;

    float laser_angle{ static_cast<float>(min_laser_angle_rad_ +
                                          i * laser_angle_increment_) };
    double total_angle{ laser_angle + robot_ptr_->getHeading() };
    float laser_depth{ laser_scan[i] };
    float laser_return_x{ laser_depth * cosf(total_angle) };
    float laser_return_y{ laser_depth * sinf(total_angle) };

    float world_x = laser_return_x + robot_ptr_->getX();
    float world_y = laser_return_y + robot_ptr_->getY();
    QPoint map_pixel = this->worldToPixel(QPointF(world_x, world_y));
    map_image_.setPixel(map_pixel, occupied_color_);
  }
}

void RobotMapper::resetMap()
{
  map_image_.fill(unknown_color_);
}

QPoint RobotMapper::worldToPixel(const QPointF& world_point)
{
  float world_x = world_point.x();
  float world_y = world_point.y();

  float map_x_diff = map_origin_.x() - world_x;
  int map_x_pix = map_x_diff / map_resolution_meters_;

  float map_y_diff = map_origin_.y() - world_y;
  int map_y_pix = map_y_diff / map_resolution_meters_;

  QPoint pixel_map_point(map_y_pix, map_x_pix);
  return pixel_map_point;
}
}  // namespace robo
