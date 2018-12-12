#ifndef CERES_SCAN_MATCHER_H
#define CERES_SCAN_MATCHER_H

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "grid2d.h"

namespace robo
{
class CeresScanMatcher
{
public:
  CeresScanMatcher();

  void Match(//const Eigen::Vector2d& target_translation,
      const Eigen::Vector3d& initial_pose_estimate,
      const Eigen::MatrixXd& point_cloud,
      const Grid2D& grid,
      Eigen::Vector3d& pose_estimate,
      ceres::Solver::Summary* summary) const;

private:
  ceres::Solver::Options ceres_solver_options_;

  const double translation_weight_{ 10. };
  const double rotation_weight_{ 40. };
  const double occupied_space_weight_{ 10.0 };
};
}

#endif /* CERES_SCAN_MATCHER_H */
