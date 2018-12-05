#ifndef ROBOTMAPPER_H
#define ROBOTMAPPER_H

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include <Eigen/Core>

#include "laserscan.h"

namespace robo
{
class Robot;

class RobotMapper
{
public:
  RobotMapper(Robot *robot_ptr);
  void getMap(QImage& map) const;
  void updateMap(const LaserScan& laser_scan);
  void resetMap();

  QRgb getOccupiedColor() const;
  QRgb getUnknownColor() const;
  QRgb getFreeColor() const;

  int getMapPixelRows() const;
  int getMapPixelCols() const;

  QPointF getMapOrigin() const;

  QPoint worldToPixel(const QPointF &world_point) const;
  QPointF pixelToWorld(const QPoint &pixel_point) const;

  double getLogOddsOccupied() const;
  double getLogOddsNull() const;
  double getLogOddsFree() const;

protected:
  Robot* robot_ptr_;

  Eigen::MatrixXd map_log_odds_;
  //QImage map_image_;

  const unsigned int map_width_meters_{ 80 };
  const unsigned int map_height_meters_{ 80 };
  const double map_resolution_meters_{ 1.0 };
  const int map_width_pixels_{ static_cast<int>(map_width_meters_ /
                                                map_resolution_meters_) };
  const int map_height_pixels_{ static_cast<int>(map_height_meters_ /
                                                 map_resolution_meters_) };

  const double max_laser_depth_{ 50. };
  const double min_laser_angle_rad_{ -M_PI };
  const double max_laser_angle_rad_{ M_PI };
  const double laser_angle_increment_{ M_PI / 60. };
  const unsigned int number_laser_returns_{ static_cast<unsigned int>(
      (max_laser_angle_rad_ - min_laser_angle_rad_) / laser_angle_increment_) };

  const QRgb occupied_color_{ qRgb(0, 0, 0) };
  const QRgb unknown_color_{ qRgb(100, 100, 100) };
  const QRgb free_color_{ qRgb(255, 255, 255) };

  const double log_odds_null_{ 0. };
  const double log_odds_occupied_{ 1. };
  const double log_odds_free_{ -1. };

  const float map_origin_x_{ 70. };
  const float map_origin_y_{ 45. };
  const QPointF map_origin_{ map_origin_x_, map_origin_y_ };

  unsigned int determineClosestLaserIndex(double angle,
                                          const LaserScan &laser_scan);
  double determineLaserAngle(unsigned int laser_index,
                             const LaserScan &laser_scan);
  double inverseLaserSensorModel(const QPoint &pixel_point,
                                 const LaserScan &laser_scan);

  double logOddsToProbability(double log_odds) const;
  void convertLogOddsMapToQImage(QImage &qimage_map) const;
};
}  // namespace robo

#endif /* ROBOTMAPPER_H */
