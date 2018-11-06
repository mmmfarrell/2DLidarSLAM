#include "robot.h"
#include "pose.h"

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

void Robot::moveForward()
{
  pose_.x += 1.0;
}

void Robot::rotateRight()
{
  pose_.heading -= 0.0872665;
}
