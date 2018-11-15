#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <vector>

#include <osg/Vec3d>
#include <osgUtil/LineSegmentIntersector>

namespace robo
{
class Robot;
}

namespace osg
{
class Group;
}

class LaserScanner
{
public:
  LaserScanner(osg::Group *scene, robo::Robot *robot);

  double getMaxDepth() const;
  double getMinAngleRadians() const;
  double getMaxAngleRadians() const;
  double getAngleIncrement() const;
  unsigned int getNumberLaserReturns() const;

  void getScan(std::vector<float>& laser_scan);

private:
  osg::Group *osg_scene_;
  robo::Robot *robot_ptr_;
  osg::ref_ptr<osgUtil::LineSegmentIntersector> line_seg_intersector_{ nullptr };

  const osg::Vec3d lidar_position_offset_{ 0., 0., 4. };
  const double max_laser_depth_{ 50. };
  const double min_angle_rad_{ -M_PI };
  const double max_angle_rad_{ M_PI };
  const double angle_increment_{ M_PI / 60. };
  const unsigned int number_laser_returns_{ static_cast<unsigned int>(
      (max_angle_rad_ - min_angle_rad_) / angle_increment_) };
};

#endif /* LASERSCANNER_H */
