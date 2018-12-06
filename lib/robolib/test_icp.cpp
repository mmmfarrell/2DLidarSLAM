#include <gtest/gtest.h>

#include <Eigen/Core>
#include <iostream>  // TODO remove

#include "icp.h"

void EXPECT_NEAR_T_VEC(const Eigen::Vector2d &tvec1,
                       const Eigen::Vector2d &tvec2, double abs_tolerance)
{
  EXPECT_NEAR(tvec1(0), tvec2(0), abs_tolerance);
  EXPECT_NEAR(tvec1(1), tvec2(1), abs_tolerance);
}

class SmallSetOfPoints : public ::testing::Test
{
public:
  Eigen::MatrixXd small_point_set_;
  Eigen::Matrix2d R_out_;
  Eigen::Vector2d t_out_;

  const int number_of_points{ 10 };
  const double xy_min{ -5. };
  const double xy_max{ 5. };
  SmallSetOfPoints()
  {
    small_point_set_.resize(2, number_of_points);
    small_point_set_.setZero();
    std::default_random_engine generator;
    generator.seed(1.);
    std::uniform_real_distribution<double> distribution(xy_min, xy_max);

    for (int i{ 0 }; i < number_of_points; i++)
    {
      small_point_set_(0, i) = distribution(generator);
      small_point_set_(1, i) = distribution(generator);
    }
  }
};

TEST_F(
    SmallSetOfPoints,
    noRotationOrTranslationComputeICP_ReturnsIdentityRotationAndNoTranslation)
{
  Eigen::MatrixXd points2;
  points2 = small_point_set_;
  robo::icpMatch(small_point_set_, points2, R_out_, t_out_);

  Eigen::Matrix2d true_rotation_matrix;
  true_rotation_matrix.setIdentity();

  Eigen::Vector2d true_translation_vector;
  true_translation_vector.setZero();

  EXPECT_EQ(R_out_, true_rotation_matrix);
  EXPECT_EQ(t_out_, true_translation_vector);
}

TEST_F(SmallSetOfPoints,
       onlyTranslationComputeICP_ReturnsIdentityRotationAndCorrectTranslation)
{
  Eigen::MatrixXd prev_points;
  prev_points = small_point_set_;

  Eigen::MatrixXd curr_points;
  curr_points = small_point_set_;

  Eigen::Vector2d true_translation_vector;
  true_translation_vector << 0.1, -0.2;
  curr_points.colwise() -= true_translation_vector;
  double true_rotation_angle{ 0. };

  robo::icpMatch(prev_points, curr_points, R_out_, t_out_);
  double rotation_angle{ robo::twoDRotationMatToAngle(R_out_) };

  double abs_tolerance{ 1.e-5 };
  EXPECT_NEAR(rotation_angle, true_rotation_angle, abs_tolerance);
  EXPECT_NEAR_T_VEC(t_out_, true_translation_vector, abs_tolerance);
}

TEST_F(SmallSetOfPoints,
       onlyRotationComputeICP_ReturnsCorrectRotationAndNoTranslation)
{
  Eigen::MatrixXd prev_points;
  prev_points = small_point_set_;

  Eigen::MatrixXd curr_points;
  curr_points = small_point_set_;

  Eigen::Matrix2d true_rotation_matrix;
  double true_rotation_angle{ 0.1 };
  robo::angleTo2DRotationMatrix(-true_rotation_angle, true_rotation_matrix);
  curr_points = true_rotation_matrix * curr_points;

  robo::icpMatch(prev_points, curr_points, R_out_, t_out_);
  double rotation_angle{ robo::twoDRotationMatToAngle(R_out_) };

  Eigen::Vector2d true_translation_vector;
  true_translation_vector.setZero();

  double abs_tolerance{ 1.e-5 };
  EXPECT_NEAR(rotation_angle, true_rotation_angle, abs_tolerance);
  EXPECT_NEAR_T_VEC(t_out_, true_translation_vector, abs_tolerance);
}

TEST_F(SmallSetOfPoints,
       rotateAndTranslatePointsComputeICP_ReturnsCorrectRotationAndTranslation)
{
  Eigen::MatrixXd prev_points;
  prev_points = small_point_set_;

  Eigen::MatrixXd curr_points;
  curr_points = small_point_set_;

  Eigen::Matrix2d true_rotation_matrix;
  double true_rotation_angle{ -0.01 };
  robo::angleTo2DRotationMatrix(-true_rotation_angle, true_rotation_matrix);
  curr_points = true_rotation_matrix * curr_points;

  Eigen::Vector2d true_translation_vector;
  true_translation_vector << 0.2, -0.1;
  curr_points.colwise() -= true_translation_vector;

  robo::icpMatch(prev_points, curr_points, R_out_, t_out_);
  double rotation_angle{ robo::twoDRotationMatToAngle(R_out_) };

  double abs_tolerance{ 1.e-2 };
  EXPECT_NEAR(rotation_angle, true_rotation_angle, abs_tolerance);
  EXPECT_NEAR_T_VEC(t_out_, true_translation_vector, abs_tolerance);
}

TEST_F(
    SmallSetOfPoints,
    mixUpRotateAndTranslatePointsComputeICP_ReturnsCorrectRotationAndTranslation)
{
  Eigen::MatrixXd prev_points;
  prev_points = small_point_set_;

  int number_of_points{ static_cast<int>(prev_points.cols()) };
  Eigen::MatrixXd curr_points{ 2, number_of_points };
  for (int i{ 0 }; i < number_of_points; i++)
  {
    curr_points.block<2, 1>(0, i) = prev_points.block<2, 1>(0, number_of_points - 1 - i);
  }

  Eigen::Matrix2d true_rotation_matrix;
  double true_rotation_angle{ -0.01 };
  robo::angleTo2DRotationMatrix(-true_rotation_angle, true_rotation_matrix);
  curr_points = true_rotation_matrix * curr_points;

  Eigen::Vector2d true_translation_vector;
  true_translation_vector << 0.2, -0.1;
  curr_points.colwise() -= true_translation_vector;

  robo::icpMatch(prev_points, curr_points, R_out_, t_out_);
  double rotation_angle{ robo::twoDRotationMatToAngle(R_out_) };

  double abs_tolerance{ 1.e-2 };
  EXPECT_NEAR(rotation_angle, true_rotation_angle, abs_tolerance);
  EXPECT_NEAR_T_VEC(t_out_, true_translation_vector, abs_tolerance);
}
