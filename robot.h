#ifndef ROBOT_H
#define ROBOT_H

#include "pose.h"

class Robot
{
public:
  Robot();

  void resetPose();
  Pose getPose() const;

  void setHeadingDegrees(double robot_heading_degrees);

  void moveForward();
  void rotateRight();
  void rotateLeft();

private:
  Pose pose_;

  double degreesToRadians(double degrees);
};

#endif /* ROBOT_H */
