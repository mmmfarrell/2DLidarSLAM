#include <Eigen/Core>

#include "laserscan.h"
#include "roboutils.h"

namespace robo
{
void laserScanToPoints(const LaserScan& laser_scan, Eigen::MatrixXd& points)
{
  std::vector<int> valid_laser_return_idx;

  for (unsigned int i{ 0 }; i < laser_scan.ranges.size(); i++)
  {
    double laser_depth{ laser_scan.ranges[i] };

    if (laser_depth > laser_scan.max_range)
      continue;
    else
      valid_laser_return_idx.push_back(i);
  }

  unsigned int number_valid_returns{ static_cast<unsigned int>(
      valid_laser_return_idx.size()) };
  points.resize(2, number_valid_returns);

  for (unsigned int i{0}; i < number_valid_returns; i++)
  {
    int laser_return_idx{ valid_laser_return_idx[i] };
    double laser_depth{ laser_scan.ranges[laser_return_idx] };
    double laser_angle{ laser_scan.min_angle +
                        laser_return_idx * laser_scan.angle_increment };
    double laser_return_x{ laser_depth * cos(-laser_angle) };
    double laser_return_y{ laser_depth * sin(-laser_angle) };
    points(0, i) = laser_return_x;
    points(1, i) = laser_return_y;
  }
}
}
