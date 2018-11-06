#ifndef ROBOT_H
#define ROBOT_H

#include "pose.h"

class Robot
{
public:
  Robot();

  void resetPose();
  Pose getPose() const;

  void moveForward();
  void rotateRight();
  void rotateLeft();

private:
  Pose pose_;
};

#endif /* ROBOT_H */
