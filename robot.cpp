#include "robot.h"
#include "pose.h"

#include <math.h>

Robot::Robot()
{
  this->resetPose();
}

void Robot::resetPose()
{
  pose_.x = 0.0;
  pose_.y = 0.0;
  pose_.heading = 0.0;
}

Pose Robot::getPose() const
{
  return pose_;
}

void Robot::setHeadingDegrees(double robot_heading_degrees)
{
  pose_.heading = degreesToRadians(robot_heading_degrees);
}

void Robot::moveForward()
{
  pose_.x += 1.0 * cos(pose_.heading);
  pose_.y += 1.0 * sin(pose_.heading);
}

void Robot::rotateRight()
{
  pose_.heading -= 0.0872665;
}

void Robot::rotateLeft()
{
  pose_.heading += 0.0872665;
}

double Robot::degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.;
}
