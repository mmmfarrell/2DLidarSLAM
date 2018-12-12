#include "robot.h"

#include <Eigen/Core>
#include <random>
#include <math.h>

namespace robo
{
Robot::Robot()
{
  this->resetState();
  this->resetDesiredVelocities();
}

void Robot::resetState()
{
  state_.setZero();
}

stateVector Robot::getState() const
{
  return state_;
}

double Robot::getX() const
{
  return state_(Robot::X);
}

double Robot::getY() const
{
  return state_(Robot::Y);
}

double Robot::getHeading() const
{
  return state_(Robot::HEADING);
}

double Robot::getVelocity() const
{
  return state_(Robot::VEL);
}

double Robot::getOmega() const
{
  return state_(Robot::OMEGA);
}

double Robot::getNoisyVelocity(double std_dev)
{
  return std_dev * normal_rand_dist_(random_gen_) + getVelocity();
}

double Robot::getNoisyOmega(double std_dev)
{
  return std_dev * normal_rand_dist_(random_gen_) + getOmega();
}

void Robot::resetDesiredVelocities()
{
  this->desired_vel_ = 0.0;
  this->desired_omega_ = 0.0;
}

void Robot::setHeadingDegrees(double robot_heading_degrees)
{
  state_(Robot::HEADING) = degreesToRadians(robot_heading_degrees);
}

void Robot::moveForward()
{
  double robot_heading{ state_(Robot::HEADING) };
  state_(Robot::X) += 1.0 * cos(robot_heading);
  state_(Robot::Y) += 1.0 * sin(robot_heading);
}

void Robot::moveBackward()
{
  double robot_heading{ state_(Robot::HEADING) };
  state_(Robot::X) -= 1.0 * cos(robot_heading);
  state_(Robot::Y) -= 1.0 * sin(robot_heading);
}

void Robot::rotateRight()
{
  state_(Robot::HEADING) -= 0.0872665;
}

void Robot::rotateLeft()
{
  state_(Robot::HEADING) += 0.0872665;
}

void Robot::setDesiredVelocity(double desired_velocity)
{
  this->desired_vel_ = desired_velocity;
}

void Robot::setDesiredOmega(double desired_omega)
{
  this->desired_omega_ = desired_omega;
}

void Robot::propagateDynamics(double time_step)
{
  this->motorDynamics(time_step);

  // RK4 integration adapted from
  // https://www.codeproject.com/Tips/792927/Fourth-Order-Runge-Kutta-Method-in-Python
  stateVector k1 = this->xdot(state_) * time_step;
  stateVector k2 = this->xdot(state_ + k1 / 2.) * time_step;
  stateVector k3 = this->xdot(state_ + k2 / 2.) * time_step;
  stateVector k4 = this->xdot(state_ + k3) * time_step;

  state_ += (k1 + 2 * (k2 + k3) + k4) / 6.;
}

stateVector Robot::xdot(stateVector state)
{
  // Unicycle Robot dynamics from eq. 1
  // http://vislab.isr.ist.utl.pt/publications/08-jetc-rcarona-vcontrol.pdf
  stateVector xdot;
  xdot.setZero();
  double velocity{ state(Robot::VEL) };
  double omega{ state(Robot::OMEGA) };
  double theta{ state(Robot::HEADING) };

  xdot(Robot::X) = velocity * cos(theta);
  xdot(Robot::Y) = velocity * sin(theta);
  xdot(Robot::HEADING) = omega;

  return xdot;
}

void Robot::motorDynamics(double time_step)
{
  // calculate the actual output force using low-pass-filters to introduce a
  // first-order approximation of delay in motor reponse for velocities
  // x(t+1) = Ce^(-t/tau)dt <- transfer to z-domain using backward
  // differentiation
  double velocity{ this->state_(Robot::VEL) };
  double tau_vel{ 0.5 };
  double alpha_vel{ time_step / (tau_vel + time_step) };
  state_(Robot::VEL) = (1 - alpha_vel) * velocity + alpha_vel * desired_vel_;

  double omega{ this->state_(Robot::OMEGA) };
  double tau_omega{ 0.5 };
  double alpha_omega{ time_step / (tau_omega + time_step) };
  state_(Robot::OMEGA) =
      (1 - alpha_omega) * omega + alpha_omega * desired_omega_;
}

double Robot::degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.;
}
} // namespace robo
