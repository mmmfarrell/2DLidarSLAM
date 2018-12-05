#include <gtest/gtest.h>
#include <QImage>

#include "robot.h"
#include "robotmapper.h"

class ANewRobotMapper : public ::testing::Test
{
public:
  robo::Robot new_robot_;
  robo::RobotMapper mapper_{ &new_robot_ };
};

TEST_F(ANewRobotMapper, askForMap_MapAllUnknown)
{
  QImage map_image;
  mapper_.getMap(map_image);

  int map_width{ mapper_.getMapPixelCols() };
  int map_height{ mapper_.getMapPixelRows() };
  QImage true_map_image{ map_width, map_height, QImage::Format_Grayscale8 };
  true_map_image.fill(mapper_.getUnknownColor());

  EXPECT_EQ(map_image, true_map_image);
}

TEST_F(ANewRobotMapper, convertWorldOriginToPixelPoint_ReturnsCorrectPixel)
{
  float world_point_x{ 0. };
  float world_point_y{ 0. };
  QPointF world_point(world_point_x, world_point_y);

  QPoint pixel_point{ mapper_.worldToPixel(world_point) };

  int true_pixel_point_x{ 45 };
  int true_pixel_point_y{ 70 };
  QPoint true_pixel_point{ true_pixel_point_x, true_pixel_point_y };

  EXPECT_EQ(pixel_point, true_pixel_point);
}

TEST_F(ANewRobotMapper, convertWorldPointToPixelPoint_ReturnsCorrectPixel)
{
  float world_point_x{ 32. };
  float world_point_y{ 14. };
  QPointF world_point(world_point_x, world_point_y);

  QPoint pixel_point{ mapper_.worldToPixel(world_point) };

  int true_pixel_point_x{ 31 };
  int true_pixel_point_y{ 38 };
  QPoint true_pixel_point{ true_pixel_point_x, true_pixel_point_y };

  EXPECT_EQ(pixel_point, true_pixel_point);
}

TEST_F(ANewRobotMapper, convertPixelOriginToWorldPoint_ReturnsCorrectWorldPoint)
{
  int pixel_point_x{ 0 };
  int pixel_point_y{ 0 };
  QPoint pixel_point{ pixel_point_x, pixel_point_y };

  QPointF world_point{ mapper_.pixelToWorld(pixel_point) };

  QPointF true_world_map_origin{ mapper_.getMapOrigin() };

  EXPECT_EQ(world_point, true_world_map_origin);
}

TEST_F(ANewRobotMapper, convertPixelPointToWorldPoint_ReturnsCorrectWorldPoint)
{
  int pixel_point_x{ 64 };
  int pixel_point_y{ 11 };
  QPoint pixel_point{ pixel_point_x, pixel_point_y };

  QPointF world_point{ mapper_.pixelToWorld(pixel_point) };

  float true_world_point_x{ 59. };
  float true_world_point_y{ -19. };
  QPointF true_world_point{ true_world_point_x, true_world_point_y };

  EXPECT_EQ(world_point, true_world_point);
}

class RobotMapperAndSmallLaserScan : public ::testing::Test,
                                     public robo::RobotMapper
{
public:
  robo::Robot new_robot_;
  robo::LaserScan small_laser_scan_;

  const double laser_min_range_{ 0. };
  const double laser_max_range_{ 50. };
  const double laser_min_angle_{ -M_PI };
  const double laser_max_angle_{ M_PI };
  const double laser_angle_increment_{ M_PI / 2. };
  const std::vector<double> laser_ranges{ 2., 4., 10., 5. };
  RobotMapperAndSmallLaserScan() : robo::RobotMapper{ &new_robot_ }
  {
    small_laser_scan_.min_range = laser_min_range_;
    small_laser_scan_.max_range = laser_max_range_;
    small_laser_scan_.min_angle = laser_min_angle_;
    small_laser_scan_.max_angle = laser_max_angle_;
    small_laser_scan_.angle_increment = laser_angle_increment_;
    small_laser_scan_.ranges = laser_ranges;
  }
};

TEST_F(RobotMapperAndSmallLaserScan,
       determineClosestLaserIndexToAnAngle_ReturnsCorrectIndex)
{
  double angle{ 2.1 };

  unsigned int laser_index{ this->determineClosestLaserIndex(
      angle, small_laser_scan_) };

  unsigned int true_laser_index{ 3 };
  EXPECT_EQ(laser_index, true_laser_index);
}

TEST_F(RobotMapperAndSmallLaserScan,
       inveseLaserSensorModelOfNullSpace_ReturnsLogOddsNullValue)
{
  int pixel_point_x{ 0 };
  int pixel_point_y{ 0 };
  QPoint pixel_point{ pixel_point_x, pixel_point_y };

  double inverse_sensor_model{ this->inverseLaserSensorModel(
      pixel_point, small_laser_scan_) };

  EXPECT_EQ(inverse_sensor_model, this->getLogOddsNull());
}

TEST_F(RobotMapperAndSmallLaserScan,
       inveseLaserSensorModelOfFreeSpace_ReturnsLogOddsFreeValue)
{
  int pixel_point_x{ 46 };
  int pixel_point_y{ 68 };
  QPoint pixel_point{ pixel_point_x, pixel_point_y };

  double inverse_sensor_model{ this->inverseLaserSensorModel(
      pixel_point, small_laser_scan_) };

  EXPECT_EQ(inverse_sensor_model, this->getLogOddsFree());
}

TEST_F(RobotMapperAndSmallLaserScan,
       inveseLaserSensorModelOfOccupiedSpace_ReturnsLogOddsOccupiedValue)
{
  int pixel_point_x{ 41 };
  int pixel_point_y{ 70 };
  QPoint pixel_point{ pixel_point_x, pixel_point_y };

  double inverse_sensor_model{ this->inverseLaserSensorModel(
      pixel_point, small_laser_scan_) };

  EXPECT_EQ(inverse_sensor_model, this->getLogOddsOccupied());
}

TEST_F(RobotMapperAndSmallLaserScan,
       updateMapWithLaserScan_NewMapIsCorrect)
{
  this->updateMap(small_laser_scan_);

  QImage map_image;
  this->getMap(map_image);

  QImage true_map_image;
  ASSERT_TRUE(
      true_map_image.load("../../../lib/robolib/test_artifacts/testmap.png"));

  EXPECT_EQ(map_image, true_map_image);
}
