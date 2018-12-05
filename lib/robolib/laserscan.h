#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <vector>

namespace robo
{
struct LaserScan
{
  double min_range;
  double max_range;
  double min_angle;
  double max_angle;
  double angle_increment;
  std::vector<float> ranges;
};
}  // namespace robo

#endif /* LASERSCAN_H */

