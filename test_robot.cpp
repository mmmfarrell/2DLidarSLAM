#include <gtest/gtest.h>

#include "robot.h"
#include "pose.h"

void posesEqual(Pose pose1, Pose pose2)
{
  EXPECT_EQ(pose1.x, pose2.x);
  EXPECT_EQ(pose1.y, pose2.y);
  EXPECT_EQ(pose1.heading, pose2.heading);
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
