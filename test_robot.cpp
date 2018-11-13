#include <gtest/gtest.h>
#include <Eigen/Core>

#include "robot.h"

void EXPECT_STATES_NEAR(robo::stateVector state1, robo::stateVector state2)
{
  double abs_error{ 1e-5 };
  EXPECT_NEAR(state1(0), state2(0), abs_error);
  EXPECT_NEAR(state1(1), state2(1), abs_error);
  EXPECT_NEAR(state1(2), state2(2), abs_error);
  EXPECT_NEAR(state1(3), state2(3), abs_error);
  EXPECT_NEAR(state1(4), state2(4), abs_error);
}

TEST(ARobot, initializeRobot_PoseAt0)
{
  robo::Robot robot;

  double true_robot_x{ 0.0 };
  double true_robot_y{ 0.0 };
  double true_robot_heading{ 0.0 };

  EXPECT_EQ(true_robot_x, robot.getX());
  EXPECT_EQ(true_robot_y, robot.getY());
  EXPECT_EQ(true_robot_heading, robot.getHeading());
}

TEST(ARobot, moveForward_NewPoseOneMeterForward)
{
  robo::Robot robot;
  robot.moveForward();

  double true_robot_x{ 1.0 };
  double true_robot_y{ 0.0 };
  double true_robot_heading{ 0.0 };

  EXPECT_EQ(true_robot_x, robot.getX());
  EXPECT_EQ(true_robot_y, robot.getY());
  EXPECT_EQ(true_robot_heading, robot.getHeading());
}

TEST(ARobot, rotateRight_NewPoseNegative5Degrees)
{
  robo::Robot robot;
  robot.rotateRight();

  double true_robot_x{ 0.0 };
  double true_robot_y{ 0.0 };
  double true_robot_heading_degrees{ -5.0 };
  double true_robot_heading_radians{ true_robot_heading_degrees * M_PI / 180. };

  double abs_error{ 1e-5 };
  EXPECT_EQ(true_robot_x, robot.getX());
  EXPECT_EQ(true_robot_y, robot.getY());
  EXPECT_NEAR(true_robot_heading_radians, robot.getHeading(), abs_error);
}

TEST(ARobot, rotateLeft_NewPosePositive5Degrees)
{
  robo::Robot robot;
  robot.rotateLeft();

  double true_robot_x{ 0.0 };
  double true_robot_y{ 0.0 };
  double true_robot_heading_degrees{ 5.0 };
  double true_robot_heading_radians{ true_robot_heading_degrees * M_PI / 180. };

  double abs_error{ 1e-5 };
  EXPECT_EQ(true_robot_x, robot.getX());
  EXPECT_EQ(true_robot_y, robot.getY());
  EXPECT_NEAR(true_robot_heading_radians, robot.getHeading(), abs_error);
}

TEST(ARobotRotated, moveForward_RobotMovesAlongItsFowardAxis)
{
  robo::Robot robot;
  double robot_heading_degrees(90.0);
  robot.setHeadingDegrees(robot_heading_degrees);
  robot.moveForward();

  double true_robot_x{ 0.0 };
  double true_robot_y{ 1.0 };
  double true_robot_heading_radians{ robot_heading_degrees * M_PI / 180. };

  double abs_error{ 1e-5 };
  EXPECT_NEAR(true_robot_x, robot.getX(), abs_error);
  EXPECT_NEAR(true_robot_y, robot.getY(), abs_error);
  EXPECT_NEAR(true_robot_heading_radians, robot.getHeading(), abs_error);
}

TEST(ARobotRotated, moveBackward_RobotMovesBackAlongItsFowardAxis)
{
  robo::Robot robot;
  double robot_heading_degrees(90.0);
  robot.setHeadingDegrees(robot_heading_degrees);
  robot.moveBackward();

  double true_robot_x{ 0.0 };
  double true_robot_y{ -1.0 };
  double true_robot_heading_radians{ robot_heading_degrees * M_PI / 180. };

  double abs_error{ 1e-5 };
  EXPECT_NEAR(true_robot_x, robot.getX(), abs_error);
  EXPECT_NEAR(true_robot_y, robot.getY(), abs_error);
  EXPECT_NEAR(true_robot_heading_radians, robot.getHeading(), abs_error);
}

TEST(ARobotAtRest, propagateDynamics_NewStateStillAtRest)
{
  robo::Robot robot;

  double time_step{ 0.01 };
  robot.propagateDynamics(time_step);

  Eigen::Matrix<double, 5, 1> trueNewState;
  trueNewState.setZero();

  EXPECT_STATES_NEAR(trueNewState, robot.getState());
}

//TEST(ARobotAtRestWithDesiredVelocity, propagateDynamics_NewStateCorrect)
//{
  //robo::Robot robot;
  //double robot_desired_velocity{ 1.0 };
  //robot.setDesiredVelocity(robot_desired_velocity);

  //double time_step{ 0.01 };
  //robot.propagateDynamics(time_step);

  //Eigen::Matrix<double, 5, 1> trueNewState;
  //trueNewState.setZero();

  //std::cout << "true state: " << trueNewState << std::endl;
  //std::cout << "actual state: " << robot.getState() << std::endl;

  ////EXPECT_STATES_NEAR(trueNewPose, robot.getPose());

//}
