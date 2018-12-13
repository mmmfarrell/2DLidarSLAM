#ifndef SLAM2D_H
#define SLAM2D_H

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "grid2d.h"
#include "laserscan.h"
#include "ceres_scan_matcher.h"

namespace robo
{

class Slam2D
{
public:
  Slam2D();
  void getMap(QImage& map) const;
  void updateRobotPoseEstimate(double velocity, double omega, double dt);
  void updateMap(const LaserScan& laser_scan);

  void resetMap();
  void resetRobotPose();
  void resetRobotPose(double x, double y, double yaw);

  void getRobotPoseEstimate(Eigen::Vector3d& pose) const;
  QRgb getUnknownColor() const;

protected:
  robo::Grid2D grid_map_;
  Eigen::MatrixXd point_cloud_;
  Eigen::Vector3d robot_pose_;
  robo::CeresScanMatcher scan_matcher_;
  ceres::Solver::Summary ceres_summary_;

  bool first_update_{ true };

  const double log_odds_null_{ 0. };
  const double log_odds_occupied_{ 0.2 };
  const double log_odds_free_{ -0.2 };

  unsigned int determineClosestLaserIndex(double angle,
                                          const LaserScan &laser_scan);
  double determineLaserAngle(unsigned int laser_index,
                             const LaserScan &laser_scan);
  double inverseLaserSensorModel(const QPoint &pixel_point,
                                 const LaserScan &laser_scan);

  double wrapAngle(double angle);
};
}  // namespace robo

#endif /* SLAM2D_H */
