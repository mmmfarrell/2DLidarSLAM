#include <gtest/gtest.h>

#include <Eigen/Core>
#include <QImage>
#include <QPointF>
#include <iostream>

#include "slam2d.h"

TEST(NewSlam2DInstance, getMap_MapAllUnknown)
{
  robo::Slam2D slammer;

  QImage map;
  slammer.getMap(map);

  QImage true_map{ 80, 80, QImage::Format_Grayscale8 };
  true_map.fill(slammer.getUnknownColor());

  EXPECT_EQ(map, true_map);
}

class SmallLaserScanTest : public ::testing::Test
{
public:
  robo::LaserScan small_laser_scan_;
  robo::Slam2D slammer_;

  const double laser_min_range_{ 0. };
  const double laser_max_range_{ 50. };
  const double laser_min_angle_{ -M_PI };
  const double laser_max_angle_{ M_PI };
  const double laser_angle_increment_{ M_PI / 2. };
  const std::vector<double> laser_ranges{ 60., 4., 60., 5. };

  const int num_updates{ 10 };
  SmallLaserScanTest()
  {
    small_laser_scan_.min_range = laser_min_range_;
    small_laser_scan_.max_range = laser_max_range_;
    small_laser_scan_.min_angle = laser_min_angle_;
    small_laser_scan_.max_angle = laser_max_angle_;
    small_laser_scan_.angle_increment = laser_angle_increment_;
    small_laser_scan_.ranges = laser_ranges;

    for (int i{ 0 }; i < num_updates; i++)
    {
      slammer_.updateMap(small_laser_scan_);
    }
  }
};

TEST_F(SmallLaserScanTest, getMap_ReturnsCorrectMap)
{
  QImage map;
  slammer_.getMap(map);
  //map.save("qimage.png");

  QImage true_map{ 80, 80, QImage::Format_Grayscale8 };
  true_map.fill(slammer_.getUnknownColor());

  EXPECT_EQ(map, true_map);
}

TEST_F(SmallLaserScanTest, moveRobotAndUpdate_ReturnsCorrectMap)
{
  double robot_vel{ 1.0 };
  double robot_omega{ 0.0 };
  double dt{ 0.1 };
  int num_steps{ 100 };

  for (int i{ 0 }; i < num_steps; i++)
  {
    slammer_.updateRobotPoseEstimate(robot_vel, robot_omega, dt);
    slammer_.updateMap(small_laser_scan_);
  }

  QImage map;
  slammer_.getMap(map);
  //map.save("qimage.png");

  QImage true_map{ 80, 80, QImage::Format_Grayscale8 };
  true_map.fill(slammer_.getUnknownColor());

  EXPECT_EQ(map, true_map);
}
