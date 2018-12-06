#include "icp.h"

#include <iostream> // TODO remove
#include <Eigen/Core>
#include <Eigen/SVD>
#include <math.h>
#include <limits>
#include <vector>

namespace robo
{
void icpMatch(Eigen::MatrixXd &prev_points, Eigen::MatrixXd &curr_points,
              Eigen::Matrix2d &R, Eigen::Vector2d &t)
{
  // ICP algorithm adapted from "Introduction to Mobile Robotics: Iterative
  // Closest Point Algorithm
  // https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf
  // Also from the python implementation publically available here
  // https://github.com/AtsushiSakai/PythonRobotics/blob/d7fe083b1c4bfa28ec0d4473d04ed3ac8a78059f/SLAM/iterative_closest_point/iterative_closest_point.py

  double eps = 0.00001;
  int max_iters = 100;

  double prev_error = 1000.;
  double d_error = 1000.;

  Eigen::Matrix3d H;
  H.setIdentity();

  int iter_count = 0;

  while (d_error >= eps)
  {
    iter_count += 1;

    Eigen::MatrixXd algined_prev_points{ nearestNeighborAssociation(prev_points, curr_points) };

    svdMotionEstimation(algined_prev_points, curr_points, R, t);

    curr_points = (R * curr_points).colwise() + t;
    updateHomogeneousMatrix(H, R, t);

    double error{ calculateTotalDistanceError(prev_points, curr_points) };
    d_error = abs(prev_error - error);
    prev_error = error;

    if (d_error <= eps)
    {
      //std::cout << "ICP converged" << std::endl;
      break;
    }
    else if (iter_count >= max_iters)
    {
      std::cout << "ICP did not converge..." << std::endl;
      break;
    }
  }

  R = H.block<2, 2>(0, 0);
  t = H.block<2, 1>(0, 2);
}

void svdMotionEstimation(Eigen::MatrixXd &prev_points,
                         Eigen::MatrixXd &curr_points, Eigen::Matrix2d &R,
                         Eigen::Vector2d &t)
{
  Eigen::Vector2d prev_mean{ prev_points.rowwise().mean() };
  Eigen::Vector2d curr_mean{ curr_points.rowwise().mean() };

  Eigen::MatrixXd prev_points_shifted{ prev_points.colwise() - prev_mean };
  Eigen::MatrixXd curr_points_shifted{ curr_points.colwise() - curr_mean };

  Eigen::MatrixXd W{ curr_points_shifted * prev_points_shifted.transpose() };
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  R = (svd.matrixU() * svd.matrixV().transpose()).transpose();

  t = prev_mean - (R * curr_mean);
}

void updateHomogeneousMatrix(Eigen::Matrix3d &H, const Eigen::Matrix2d &R,
                             const Eigen::Vector2d &t)
{
  Eigen::Matrix3d H_change;
  H_change.setZero();

  H_change(0, 0) = R(0, 0);
  H_change(0, 1) = R(0, 1);
  H_change(1, 0) = R(1, 0);
  H_change(1, 1) = R(1, 1);
  H_change(2, 2) = 1.;

  H_change(0, 2) = t(0);
  H_change(1, 2) = t(1);

  H = H * H_change;
}

double calculateTotalDistanceError(Eigen::MatrixXd &prev_points,
                                   Eigen::MatrixXd &curr_points)
{
  Eigen::MatrixXd difference_points{ prev_points - curr_points };
  Eigen::MatrixXd distance_errors{ difference_points.colwise().norm() };
  double sum_dist_errors{ distance_errors.sum() };

  return sum_dist_errors;
}

Eigen::MatrixXd nearestNeighborAssociation(const Eigen::MatrixXd &prev_points,
                                           const Eigen::MatrixXd &curr_points)
{
  int number_of_points{ static_cast<int>(prev_points.cols()) };
  Eigen::MatrixXd aligned_prev_points(2, number_of_points);

  // Loop through curr_points
  for (int i{ 0 }; i < number_of_points; i++)
  {
    int min_index{ -1 };
    double min_distance{ std::numeric_limits<double>::max() };

    // Loop through all prev_points and find the point that is closest to the
    // point in curr_points
    for (int j{ 0 }; j < number_of_points; j++)
    {
      double distance{
        (prev_points.block<2, 1>(0, j) - curr_points.block<2, 1>(0, i)).norm() };

      if (distance < min_distance)
      {
        min_distance = distance;
        min_index = j;
      }
    }

    aligned_prev_points.block<2, 1>(0, i) =
        prev_points.block<2, 1>(0, min_index);
  }

  return aligned_prev_points;
}

void angleTo2DRotationMatrix(double angle, Eigen::Matrix2d& rot_mat)
{
  rot_mat(0, 0) = cos(angle);
  rot_mat(0, 1) = -sin(angle);
  rot_mat(1, 0) = sin(angle);
  rot_mat(1, 1) = cos(angle);
}

double twoDRotationMatToAngle(const Eigen::Matrix2d& rot_mat)
{
  double cos_angle{ rot_mat(0, 0) };
  double sin_angle{ rot_mat(1, 0) };

  return atan2(sin_angle, cos_angle);
}

}
