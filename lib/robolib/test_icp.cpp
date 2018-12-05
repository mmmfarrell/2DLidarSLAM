#include <gtest/gtest.h>

#include <Eigen/Core>
#include <iostream>  // TODO remove

#include "icp.h"

void EXPECT_EQ_T_VEC(const Eigen::Vector2d& tvec1, const Eigen::Vector2d& tvec2)
{
  EXPECT_DOUBLE_EQ(tvec1(0), tvec2(0));
  EXPECT_DOUBLE_EQ(tvec1(1), tvec2(1));
}

void EXPECT_EQ_R_MAT(const Eigen::Matrix2d& rmat1, const Eigen::Matrix2d& rmat2)
{
  EXPECT_DOUBLE_EQ(rmat1(0, 0), rmat2(0, 0));
  EXPECT_DOUBLE_EQ(rmat1(0, 1), rmat2(0, 1));
  EXPECT_DOUBLE_EQ(rmat1(1, 0), rmat2(1, 0));
  EXPECT_DOUBLE_EQ(rmat1(1, 1), rmat2(1, 1));
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
  // TODO perfect associativity
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
  // TODO perfect associativity
  Eigen::MatrixXd prev_points;
  prev_points = small_point_set_;

  Eigen::MatrixXd curr_points;
  curr_points = small_point_set_;

  Eigen::Vector2d true_translation_vector;
  true_translation_vector << 1., -2;
  curr_points.colwise() -= true_translation_vector;

  robo::svdMotionEstimation(prev_points, curr_points, R_out_, t_out_);

  Eigen::Matrix2d true_rotation_matrix;
  true_rotation_matrix.setIdentity();

  EXPECT_EQ_R_MAT(R_out_, true_rotation_matrix);
  EXPECT_EQ_T_VEC(t_out_, true_translation_vector);
}

TEST_F(SmallSetOfPoints,
       onlyRotationComputeICP_ReturnsCorrectRotationAndNoTranslation)
{
  // TODO perfect associativity
  Eigen::MatrixXd prev_points;
  prev_points = small_point_set_;

  Eigen::MatrixXd curr_points;
  curr_points = small_point_set_;

  Eigen::Matrix2d true_rotation_matrix;
  double rotation_angle{ 0.1 };
  robo::angleTo2DRotationMatrix(rotation_angle, true_rotation_matrix);
  //curr_points.colwise() *= true_rotation_matrix;
  curr_points = true_rotation_matrix * curr_points;

  robo::svdMotionEstimation(prev_points, curr_points, R_out_, t_out_);

  Eigen::Vector2d true_translation_vector;
  true_translation_vector.setZero();

  EXPECT_EQ_R_MAT(R_out_, true_rotation_matrix);
  EXPECT_EQ_T_VEC(t_out_, true_translation_vector);
}

