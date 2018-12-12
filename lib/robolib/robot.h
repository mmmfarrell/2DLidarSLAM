#ifndef ROBOT_H
#define ROBOT_H

#include <Eigen/Core>
#include <random>

namespace robo
{

typedef Eigen::Matrix<double, 5, 1> stateVector;

class Robot
{
public:

  typedef enum {
    X,
    Y,
    HEADING,
    VEL,
    OMEGA
  } state_variables;

  Robot();

  void resetState();
  stateVector getState() const;

  double getX() const;
  double getY() const;
  double getHeading() const;
  double getVelocity() const;
  double getOmega() const;

  double getNoisyVelocity(double std_dev);
  double getNoisyOmega(double std_dev);

  void resetDesiredVelocities();

  void setHeadingDegrees(double robot_heading_degrees);

  void moveForward();
  void moveBackward();
  void rotateRight();
  void rotateLeft();

  void setDesiredVelocity(double desired_velocity);
  void setDesiredOmega(double desired_omega);
  
  void propagateDynamics(double time_step);

private:
  stateVector state_;
  double desired_vel_;
  double desired_omega_;

  std::default_random_engine random_gen_;
  std::normal_distribution<double> normal_rand_dist_;

  stateVector xdot(stateVector state);
  void motorDynamics(double time_step);

  double degreesToRadians(double degrees);
};
} // namespace robo

#endif /* ROBOT_H */
