#include "laserscanner.h"
#include "robot.h"

#include <math.h>
#include <limits>

#include <osg/Group>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

namespace robo
{
LaserScanner::LaserScanner(osg::Group *scene, robo::Robot *robot) :
  osg_scene_{ scene },
  robot_ptr_{ robot }
{
  // TODO why do I need these dummy things
  osg::Vec3d start{ 0., 0., 0. };
  osg::Vec3d end{ 4., 0., 0. };
  line_seg_intersector_ = new osgUtil::LineSegmentIntersector(start, end);
}

double LaserScanner::getMaxDepth() const
{
  return this->max_laser_depth_;
}

double LaserScanner::getMinAngleRadians() const
{
  return this->min_angle_rad_;
}

double LaserScanner::getMaxAngleRadians() const
{
  return this->max_angle_rad_;
}

double LaserScanner::getAngleIncrement() const
{
  return this->angle_increment_;
}

unsigned int LaserScanner::getNumberLaserReturns() const
{
  return this->number_laser_returns_;
}

void LaserScanner::getScan(std::vector<float>& laser_scan)
{
  if (laser_scan.size() != number_laser_returns_)
    return; // TODO maybe throw error?

  for (unsigned int i{ 0 }; i < number_laser_returns_; i++)
  {
    line_seg_intersector_->reset();
    osgUtil::IntersectionVisitor iv(line_seg_intersector_.get());
    osg_scene_->accept(iv);

    osg::Vec3d robot_position{ robot_ptr_->getX(), robot_ptr_->getY(), 0. };
    osg::Vec3d laser_start{ robot_position + lidar_position_offset_ };
    line_seg_intersector_->setStart(laser_start);

    double laser_angle{ min_angle_rad_ + i * angle_increment_ };

    double robot_heading{ robot_ptr_->getHeading() };
    double total_heading{ robot_heading + laser_angle };
    double laser_x{ max_laser_depth_ * cos(total_heading) };
    double laser_y{ max_laser_depth_ * sin(total_heading) };
    osg::Vec3d laser_xy{ laser_x, laser_y, 0. };
    osg::Vec3d laser_end{ laser_xy + laser_start };
    line_seg_intersector_->setEnd(laser_end);

    bool contains_intersections{
      line_seg_intersector_->containsIntersections() };

    double depth = std::numeric_limits<float>::max();

    if (contains_intersections)
    {
      osg::Vec3d first_int = line_seg_intersector_->getFirstIntersection()
                                 .getWorldIntersectPoint();
      depth = (first_int - laser_start).length();
    }

    laser_scan[i] = depth;
  }
}
} // namespace robo
