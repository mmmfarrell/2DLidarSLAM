#ifndef LASERSCANNER_H
#define LASERSCANNER_H

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

  double getScan();

private:
  osg::Group *osg_scene_;
  robo::Robot *robot_ptr_;
  osg::ref_ptr<osgUtil::LineSegmentIntersector> line_seg_intersector_{ nullptr };

  const osg::Vec3d lidar_position_offset_{ 0., 0., 4. };
  const double max_laser_depth{ 50. };
};

#endif /* LASERSCANNER_H */
