#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include <algorithm>
#include <math.h>
#include <iostream>

#include "slam2d.h"

namespace robo
{
Slam2D::Slam2D()
{
  this->resetMap();
  this->resetRobotPose();
}

void Slam2D::getMap(QImage &map) const
{
  grid_map_.getMapQImage(map);
}

void Slam2D::updateRobotPoseEstimate(double velocity, double omega, double dt)
{
  double theta{ robot_pose_(2) };

  robot_pose_(0) += dt * velocity * cos(theta);
  robot_pose_(1) += dt * velocity * sin(theta);
  robot_pose_(2) += dt * omega;

  robot_pose_(2) = this->wrapAngle(robot_pose_(2));

}

void Slam2D::updateMap(const LaserScan &laser_scan)
{
  for (int pix_row{ 0 }; pix_row < grid_map_.getMapPixelRows(); pix_row++)
  {
    for (int pix_col{ 0 }; pix_col < grid_map_.getMapPixelCols(); pix_col++)
    {
      QPoint pix_point{ pix_row, pix_col };
      //map_log_odds_(pix_row, pix_col) += this->inverseLaserSensorModel(
          //pix_point, laser_scan) - log_odds_null_;
      double meas_model{ this->inverseLaserSensorModel(pix_point, laser_scan) -
                         log_odds_null_ };
      grid_map_.addLogOdds(pix_point, meas_model);
    }
  }
}

void Slam2D::resetMap()
{
  grid_map_.resetMap();
}

void Slam2D::resetRobotPose()
{
  robot_pose_.setZero();
}

void Slam2D::getRobotPoseEstimate(Eigen::Vector3d& pose) const
{
  pose = robot_pose_;
}

QRgb Slam2D::getUnknownColor() const
{
  return grid_map_.getUnknownColor();
}

unsigned int Slam2D::determineClosestLaserIndex(
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

double Slam2D::determineLaserAngle(unsigned int laser_index,
                                        const LaserScan &laser_scan)
{
  return laser_scan.min_angle + laser_index * laser_scan.angle_increment;
}

double Slam2D::inverseLaserSensorModel(const QPoint &pixel_point,
                                            const LaserScan &laser_scan)
{
  // Algorithm from "Probabilistic Robotics" by Thrun, et. al.
  // Table 9.2 "Algorith Inverse Range Sensor Model" pg 288
  double robot_x{ robot_pose_(0) };
  double robot_y{ robot_pose_(1) };
  double robot_heading{ robot_pose_(2) };

  QPointF world_point{ grid_map_.pixelToWorld(pixel_point) };
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
  double alpha{ grid_map_.getMapResolution() };

  // Beta is the width of a sensor beam
  //double beta{ laser_scan.angle_increment };
  double beta{ 0.05 };

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

double Slam2D::wrapAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle <= -M_PI)
    angle += 2 * M_PI;

  return angle;
}
}  // namespace robo
