#include <ceres/ceres.h>

#include "grid2d.h"
#include "occupied_space_cost_function_2d.h"
#include "rotation_delta_cost_functor_2d.h"
#include "translation_delta_cost_functor_2d.h"

#include "ceres_scan_matcher.h"

namespace robo
{
CeresScanMatcher::CeresScanMatcher()
{
  ceres_solver_options_.use_nonmonotonic_steps = true;
  ceres_solver_options_.max_num_iterations = 50;
  ceres_solver_options_.num_threads = 4;
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;

}

void CeresScanMatcher::Match(//const Eigen::Vector2d &target_translation,
                             const Eigen::Vector3d &initial_pose_estimate,
                             const Eigen::MatrixXd &point_cloud,
                             const Grid2D &grid,
                             Eigen::Vector3d &pose_estimate,
                             ceres::Solver::Summary *summary) const
{
  double ceres_pose_estimate[3] = { initial_pose_estimate(0),
                                    initial_pose_estimate(1),
                                    initial_pose_estimate(2) };
  ceres::Problem problem;
  pose_estimate = initial_pose_estimate;

  double num_points{ static_cast<double>(point_cloud.cols()) };
  double occupied_weight{ this->occupied_space_weight_ /
                          std::sqrt(num_points) };
  problem.AddResidualBlock(
      CreateOccupiedSpaceCostFunction2D(occupied_weight, point_cloud, grid),
      nullptr, ceres_pose_estimate);

  const Eigen::Vector2d target_translation =
      initial_pose_estimate.block<2, 1>(0, 0);
  //target_translation(0) = initial_pose_estimate(0);
  //target_translation(1) = initial_pose_estimate(1);

  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          this->translation_weight_, target_translation),
      nullptr, ceres_pose_estimate);

  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          this->rotation_weight_, ceres_pose_estimate[2]),
      nullptr, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  pose_estimate(0) = ceres_pose_estimate[0];
  pose_estimate(1) = ceres_pose_estimate[1];
  pose_estimate(2) = ceres_pose_estimate[2];
}
}
