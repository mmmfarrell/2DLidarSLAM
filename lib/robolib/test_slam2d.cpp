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

class SlamSmallLaserScanTest : public ::testing::Test
{
public:
  robo::LaserScan small_laser_scan_;
  robo::Slam2D slammer_;

  const double laser_min_range_{ 0. };
  const double laser_max_range_{ 50. };
  const double laser_min_angle_{ -M_PI };
  const double laser_max_angle_{ M_PI };
  const double laser_angle_increment_{ M_PI / 2. };
  const std::vector<double> laser_ranges{ 3., 4., 2., 5. };

  const int num_updates{ 10 };
  SlamSmallLaserScanTest()
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

TEST_F(SlamSmallLaserScanTest,
       getMap_OccupiedSpaceReturnsColorCorrespondingToOccupied)
{
  QImage map;
  slammer_.getMap(map);

  int occupied_pixel_point_x{ 41 };
  int occupied_pixel_point_y{ 70 };
  QRgb occupied_pixel_rgb{ map.pixel(occupied_pixel_point_x,
                                           occupied_pixel_point_y) };
  int occupied_pixel_gray_val{ qGray(occupied_pixel_rgb) };

  QRgb unknown_rgb{ slammer_.getUnknownColor() };
  int unknown_gray_val{ qGray(unknown_rgb) };

  EXPECT_LT(occupied_pixel_gray_val, unknown_gray_val);
}

TEST_F(SlamSmallLaserScanTest, getMap_FreeSpaceReturnsColorCorrespondingToFree)
{
  QImage map;
  slammer_.getMap(map);

  int occupied_pixel_point_x{ 44 };
  int occupied_pixel_point_y{ 70 };
  QRgb occupied_pixel_rgb{ map.pixel(occupied_pixel_point_x,
                                           occupied_pixel_point_y) };
  int occupied_pixel_gray_val{ qGray(occupied_pixel_rgb) };

  QRgb unknown_rgb{ slammer_.getUnknownColor() };
  int unknown_gray_val{ qGray(unknown_rgb) };

  EXPECT_GT(occupied_pixel_gray_val, unknown_gray_val);
}
