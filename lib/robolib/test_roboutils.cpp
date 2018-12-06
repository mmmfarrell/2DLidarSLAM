#include <gtest/gtest.h>

#include <Eigen/Core>

#include "laserscan.h"
#include "roboutils.h"

void EXPECT_EIGEN_MATS_NEAR(Eigen::MatrixXd &mat1,
                            Eigen::MatrixXd &mat2, double abs_tolerance)
{
  ASSERT_EQ(mat1.cols(), mat2.cols());
  ASSERT_EQ(mat1.rows(), mat2.rows());

  int num_rows{ static_cast<int>(mat1.rows()) };
  int num_cols{ static_cast<int>(mat1.cols()) };

  for (int row{0}; row < num_rows; row++)
  {
    for (int col{0}; col < num_cols; col++)
    {
      EXPECT_NEAR(mat1(row, col), mat2(row, col), abs_tolerance);
    }
  }
}

class SmallLaserScan : public ::testing::Test
{
public:
  robo::LaserScan small_laser_scan_;

  const double laser_min_range_{ 0. };
  const double laser_max_range_{ 50. };
  const double laser_min_angle_{ -M_PI };
  const double laser_max_angle_{ M_PI };
  const double laser_angle_increment_{ M_PI / 2. };
  const std::vector<double> laser_ranges{ 2., 4., 10., 5. };
  SmallLaserScan()
  {
    small_laser_scan_.min_range = laser_min_range_;
    small_laser_scan_.max_range = laser_max_range_;
    small_laser_scan_.min_angle = laser_min_angle_;
    small_laser_scan_.max_angle = laser_max_angle_;
    small_laser_scan_.angle_increment = laser_angle_increment_;
    small_laser_scan_.ranges = laser_ranges;
  }
};

TEST_F(SmallLaserScan, convertLaserScanToPoints_ReturnsCorrectPoints)
{
  Eigen::MatrixXd points;
  robo::laserScanToPoints(small_laser_scan_, points);

  int true_number_of_points{ 4 };
  Eigen::MatrixXd true_points(2, true_number_of_points);
  true_points(0, 0) = -2.;
  true_points(1, 0) = 0.;

  true_points(0, 1) = 0.;
  true_points(1, 1) = 4.;

  true_points(0, 2) = 10.;
  true_points(1, 2) = 0.;

  true_points(0, 3) = 0.;
  true_points(1, 3) = -5.;

  double abs_tolerance{ 1e-5 };
  EXPECT_EIGEN_MATS_NEAR(points, true_points, abs_tolerance);
}
