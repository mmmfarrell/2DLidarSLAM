#include <gtest/gtest.h>

#include <Eigen/Core>
#include <QPointF>
#include <iostream>

#include "grid2d.h"
#include "ceres_scan_matcher.h"

class CeresSinglePointTest : public ::testing::Test
{
protected:
  robo::Grid2D grid_;
  Eigen::MatrixXd single_point_set_;
  robo::CeresScanMatcher scan_matcher_;
  const double true_x_diff{ -0.5 };
  const double true_y_diff{ 0.5 };

  CeresSinglePointTest()
  {
    QPointF world_point{ -3.5, 2.5 };

    single_point_set_.resize(2, 1);
    single_point_set_ << world_point.x() - true_x_diff,
        world_point.y() - true_y_diff;

    grid_.setProbabilityOfWorldPoint(world_point, grid_.getMaxProbability());
  }

  void testFromInitialPose(double initial_x, double initial_y)
  {
    Eigen::Vector3d initial_pose_estimate;
    initial_pose_estimate << initial_x, initial_y, 0.;

    Eigen::Vector3d final_pose_estimate;

    ceres::Solver::Summary summary;

    scan_matcher_.Match(initial_pose_estimate, single_point_set_, grid_,
                        final_pose_estimate, &summary);

    double abs_tolerance{ 0.1 };
    EXPECT_NEAR(final_pose_estimate(0), true_x_diff, abs_tolerance);
    EXPECT_NEAR(final_pose_estimate(1), true_y_diff, abs_tolerance);
  }
};

TEST_F(CeresSinglePointTest, startAtPerfectPose_ConvergesToCorrectPose)
{
  this->testFromInitialPose(-0.5, 0.5);
}

TEST_F(CeresSinglePointTest, startAtXError_ConvergesToCorrectPose)
{
  this->testFromInitialPose(-0.45, 0.5);
}

TEST_F(CeresSinglePointTest, startAtYError_ConvergesToCorrectPose)
{
  this->testFromInitialPose(-0.5, 0.45);
}

TEST_F(CeresSinglePointTest, startAtXYError_ConvergesToCorrectPose)
{
  this->testFromInitialPose(-0.45, 0.45);
}
