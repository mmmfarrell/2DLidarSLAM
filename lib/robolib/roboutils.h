#ifndef ROBOUTILS_H
#define ROBOUTILS_H

#include <Eigen/Core>
#include "laserscan.h"

namespace robo
{
void laserScanToPoints(const LaserScan& laser_scan, Eigen::MatrixXd& points);
}

#endif /* ROBOUTILS_H */
