#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include <algorithm>
#include <math.h>
#include <iostream> // TODO remove

#include "robot.h"
#include "robotmapper.h"

namespace robo
{
RobotMapper::RobotMapper(Robot *robot_ptr) :
  robot_ptr_{ robot_ptr }
{
  this->resetMap();
}

void RobotMapper::getMap(QImage &map) const
{
  this->convertLogOddsMapToQImage(map);
}

void RobotMapper::updateMap(const LaserScan &laser_scan)
{
  for (int pix_row{ 0 }; pix_row < map_height_pixels_; pix_row++)
  {
    for (int pix_col{ 0 }; pix_col < map_width_pixels_; pix_col++)
    {
      QPoint pix_point{ pix_row, pix_col };
      map_log_odds_(pix_row, pix_col) += this->inverseLaserSensorModel(
          pix_point, laser_scan) - log_odds_null_;
    }
  }
}

void RobotMapper::resetMap()
{
  map_log_odds_.resize(map_width_pixels_, map_height_pixels_);
  map_log_odds_.setZero();
}

QRgb RobotMapper::getOccupiedColor() const
{
  return this->occupied_color_;
}

QRgb RobotMapper::getUnknownColor() const
{
  return this->unknown_color_;
}

QRgb RobotMapper::getFreeColor() const
{
  return this->free_color_;
}

int RobotMapper::getMapPixelRows() const
{
  return static_cast<unsigned int>(this->map_height_pixels_);
}

int RobotMapper::getMapPixelCols() const
{
  return static_cast<unsigned int>(this->map_width_pixels_);
}

QPointF RobotMapper::getMapOrigin() const
{
  return this->map_origin_;
}

QPoint RobotMapper::worldToPixel(const QPointF &world_point) const
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

QPointF RobotMapper::pixelToWorld(const QPoint &pixel_point) const
{
  int px{ pixel_point.x() };
  int py{ pixel_point.y() };

  double world_x{ map_origin_.x() - py * map_resolution_meters_ };
  double world_y{ map_origin_.y() - px * map_resolution_meters_ };

  return QPointF(world_x, world_y);
}

double RobotMapper::getLogOddsOccupied() const
{
  return this->log_odds_occupied_;
}

double RobotMapper::getLogOddsNull() const
{
  return this->log_odds_null_;
}

double RobotMapper::getLogOddsFree() const
{
  return this->log_odds_free_;
}

unsigned int RobotMapper::determineClosestLaserIndex(
    double angle, const LaserScan &laser_scan)
{
  double angle_from_laser_start{ angle - laser_scan.min_angle };
  double number_of_angle_increments{ angle_from_laser_start /
                                     laser_scan.angle_increment };
  int laser_index{ static_cast<int>(std::round(number_of_angle_increments)) };

  // Note: this only works if laser covers all 360 degrees around
  int number_of_laser_returns{ static_cast<int>(laser_scan.ranges.size()) };
  while (laser_index >= number_of_laser_returns)
    laser_index -= number_of_laser_returns;

  return static_cast<unsigned int>(laser_index);
}

double RobotMapper::determineLaserAngle(unsigned int laser_index,
                                        const LaserScan &laser_scan)
{
  return laser_scan.min_angle + laser_index * laser_scan.angle_increment;
}

double RobotMapper::inverseLaserSensorModel(const QPoint &pixel_point,
                                            const LaserScan &laser_scan)
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
      atan2(-(world_point.y() - robot_y), world_point.x() - robot_x) +
      robot_heading;
  angle_to_point = this->wrapAngle(angle_to_point);

  unsigned int closest_laser_index{ this->determineClosestLaserIndex(
      angle_to_point, laser_scan) };

  double laser_range{ laser_scan.ranges[closest_laser_index] };
  double laser_angle{ this->determineLaserAngle(closest_laser_index,
                                                laser_scan) };

  // Alpha is the thickness of obstacles
  double alpha{ map_resolution_meters_ };

  // Beta is the width of a sensor beam
  double beta{ laser_scan.angle_increment };

  double max_range_meas{ std::min(laser_scan.max_range,
                                  laser_range + alpha / 2.) };
  double angle_diff{ angle_to_point - laser_angle };
  angle_diff = this->wrapAngle(angle_diff);

  double abs_angle_diff{ std::abs(angle_diff) };

  if ((range_to_point > max_range_meas) || (abs_angle_diff > beta / 2.))
  {
    return log_odds_null_;
  }
  else if ((laser_range < laser_scan.max_range) &&
           (abs(range_to_point - laser_range) < alpha / 2.))
  {
    return log_odds_occupied_;
  }
  else if (range_to_point <= laser_range)
  {
    return log_odds_free_;
  }

  return log_odds_null_;
}

double RobotMapper::logOddsToProbability(double log_odds) const
{
  return 1. - 1. / (1. + exp(log_odds));
}

void RobotMapper::convertLogOddsMapToQImage(QImage &map) const
{
  int rows{ static_cast<int>(map_log_odds_.rows()) };
  int cols{ static_cast<int>(map_log_odds_.cols()) };
  map = QImage(rows, cols, QImage::Format_Grayscale8);
  map.fill(unknown_color_);

  for (int row{ 0 }; row < rows; row++)
  {
    for (int col{ 0 }; col < cols; col++)
    {
      double log_odds_map_value{ map_log_odds_(row, col) };
      double probability_occupied{ this->logOddsToProbability(
          log_odds_map_value) };

      if (probability_occupied > 0.5)
      {
        map.setPixel(QPoint(row, col), occupied_color_);
      }
      else if (probability_occupied < 0.5)
      {
        map.setPixel(QPoint(row, col), free_color_);
      }
    }
  }
}

double RobotMapper::wrapAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle <= -M_PI)
    angle += 2 * M_PI;

  return angle;
}
}  // namespace robo
