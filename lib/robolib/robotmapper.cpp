#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include <algorithm>
#include <math.h>
#include <iostream>

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

    double world_x{ laser_return_x + robot_ptr_->getX() };
    double world_y{ laser_return_y + robot_ptr_->getY() };
    QPoint map_pixel{ this->worldToPixel(QPointF(world_x, world_y)) };
    map_image_.setPixel(map_pixel, occupied_color_);
  }

  for (unsigned int pix_row{ 0 }; pix_row < map_height_pixels_; pix_row++)
  {
    for (unsigned int pix_col{ 0 }; pix_col < map_width_pixels_; pix_col++)
    {
      //std::cout << "row: " << pix_row << " col: " << pix_col << std::endl;
      QPoint pix_point{ pix_row, pix_col };
      map_log_odds_(pix_row, pix_col) = this->inverseLaserSensorModel(
          pix_point, laser_scan);
    }
  }
}

void RobotMapper::resetMap()
{
  map_log_odds_.resize(map_width_pixels_, map_height_pixels_);
  map_log_odds_.setZero();
  map_image_.fill(unknown_color_);
  //map_image_.fill(free_color_);
}

QPoint RobotMapper::worldToPixel(const QPointF &world_point)
{
  double world_x{ world_point.x() };
  double world_y{ world_point.y() };

  double map_x_diff{ map_origin_.x() - world_x };
  int map_x_pix{ static_cast<int>(map_x_diff / map_resolution_meters_) };

  double map_y_diff{ map_origin_.y() - world_y };
  int map_y_pix{ static_cast<int>(map_y_diff / map_resolution_meters_) };

  QPoint pixel_map_point(map_y_pix, map_x_pix);
  return pixel_map_point;
}

QPointF RobotMapper::pixelToWorld(const QPoint &pixel_point)
{
  int px{ pixel_point.x() };
  int py{ pixel_point.y() };

  double world_x{ map_origin_.y() - py * map_resolution_meters_ };
  double world_y{ map_origin_.x() - px * map_resolution_meters_ };

  return QPointF(world_x, world_y);
}

unsigned int RobotMapper::determineClosestLaserIndex(double angle)
{
  return 0;
}

//double RobotMapper::inverseLaserSensorModel(const QPoint &pixel_point,
                                            //double angle, double range)
double RobotMapper::inverseLaserSensorModel(
    const QPoint &pixel_point, const std::vector<float> &laser_scan)
{
  // Algorithm from "Probabilistic Robotics" by Thrun, et. al.
  // Table 9.2 "Algorith Inverse Range Sensor Model" pg 288
  double robot_x{ robot_ptr_->getX() };
  double robot_y{ robot_ptr_->getY() };
  double robot_heading{ robot_ptr_->getHeading() };

  QPointF world_point{ this->pixelToWorld(pixel_point) };
  double range_to_point = sqrt(pow(robot_x - world_point.x(), 2.) +
                               pow(robot_y - world_point.y(), 2.));
  double angle_to_point =
      atan2(robot_y - world_point.y(), robot_x - world_point.x()) -
      robot_heading;

  // Alpha is the thickness of obstacles
  double alpha{ map_resolution_meters_ / 2. };

  // Beta is the width of a sensor beam
  double beta{ laser_angle_increment_ / 2. };

  unsigned int closest_laser_index{ this->determineClosestLaserIndex(
      angle_to_point) };

  //double max_range_meas{ std::min(max_laser_depth_, range + alpha / 2.) };
  //double abs_angle_diff{ abs(angle_to_point - angle) };

  //if ((range_to_point > max_range_meas) || (abs_angle_diff > beta / 2.))
  //{
    //return log_odds_null_;
  //}
  //else if ((range < max_laser_depth_) &&
           //(abs(range_to_point - range) < alpha / 2.))
  //{
    //return log_odds_occupied_;
  //}
  //else if (range_to_point <= range)
  //{
    //return log_odds_free_;
  //}

  return log_odds_null_;
}
}  // namespace robo
