#ifndef ICP_H
#define ICP_H

#include <Eigen/Core>

namespace robo
{
void icpMatch(Eigen::MatrixXd &points1, Eigen::MatrixXd &points2,
              Eigen::Matrix2d &R, Eigen::Vector2d &t);

void svdMotionEstimation(Eigen::MatrixXd &points1, Eigen::MatrixXd &points2,
                         Eigen::Matrix2d &R, Eigen::Vector2d &t);

// TODO move me
void angleTo2DRotationMatrix(double angle, Eigen::Matrix2d& rot_mat);
}

#endif /* ICP_H */
