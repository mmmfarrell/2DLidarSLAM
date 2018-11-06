#include <gtest/gtest.h>

#include "robot.h"
#include "pose.h"

void posesEqual(Pose pose1, Pose pose2)
{
  EXPECT_EQ(pose1.x, pose2.x);
  EXPECT_EQ(pose1.y, pose2.y);
  EXPECT_EQ(pose1.heading, pose2.heading);
}

void posesNear(Pose pose1, Pose pose2)
{
  double abs_error{ 1e-5 };
  EXPECT_NEAR(pose1.x, pose2.x, abs_error);
  EXPECT_NEAR(pose1.y, pose2.y, abs_error);
  EXPECT_NEAR(pose1.heading, pose2.heading, abs_error);
}

TEST(ARobot, initializeRobot_PoseAt0)
{
  Robot robot;

  Pose trueZeroPose;
  trueZeroPose.x = 0.0;
  trueZeroPose.y = 0.0;
  trueZeroPose.heading = 0.0;

  posesEqual(trueZeroPose, robot.getPose());
}

TEST(ARobot, moveForward_NewPoseOneMeterForward)
{
  Robot robot;
  robot.moveForward();

  Pose trueNewPose;
  trueNewPose.x = 1.0;
  trueNewPose.y = 0.0;
  trueNewPose.heading = 0.0;

  posesEqual(trueNewPose, robot.getPose());
}

TEST(ARobot, rotateRight_NewPoseNegative5Degrees)
{
  Robot robot;
  robot.rotateRight();

  Pose trueNewPose;
  trueNewPose.x = 0.0;
  trueNewPose.y = 0.0;
  double degrees{ -5.0 };
  double radians{ degrees * M_PI / 180. };
  trueNewPose.heading = radians;

  posesNear(trueNewPose, robot.getPose());
}

TEST(ARobot, rotateLeft_NewPosePositive5Degrees)
{
  Robot robot;
  robot.rotateLeft();

  Pose trueNewPose;
  trueNewPose.x = 0.0;
  trueNewPose.y = 0.0;
  double degrees{ 5.0 };
  double radians{ degrees * M_PI / 180. };
  trueNewPose.heading = radians;

  posesNear(trueNewPose, robot.getPose());
}
