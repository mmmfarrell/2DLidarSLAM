#ifndef ICP_H
#define ICP_H

#include <vector>
#include <Eigen/Core>

namespace robo
{
void icpMatch(Eigen::MatrixXd &prev_points, Eigen::MatrixXd &curr_points,
              Eigen::Matrix2d &R, Eigen::Vector2d &t);

void svdMotionEstimation(Eigen::MatrixXd &prev_points,
                         Eigen::MatrixXd &curr_points, Eigen::Matrix2d &R,
                         Eigen::Vector2d &t);

void updateHomogeneousMatrix(Eigen::Matrix3d &H, const Eigen::Matrix2d &R,
                             const Eigen::Vector2d &t);

double calculateTotalDistanceError(Eigen::MatrixXd &prev_points,
                                   Eigen::MatrixXd &curr_points);
Eigen::MatrixXd nearestNeighborAssociation(const Eigen::MatrixXd &prev_points,
                                           const Eigen::MatrixXd &curr_points);

void angleTo2DRotationMatrix(double angle, Eigen::Matrix2d& rot_mat);
double twoDRotationMatToAngle(const Eigen::Matrix2d& rot_mat);
}

#endif /* ICP_H */
