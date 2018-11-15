#include "laserscanner.h"
#include "robot.h"

#include <iostream>
#include <math.h>

#include <osg/Group>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

LaserScanner::LaserScanner(osg::Group *scene, robo::Robot *robot) :
  osg_scene_{ scene },
  robot_ptr_{ robot }
{
  // TODO why do I need these dummy things
  osg::Vec3d start{ 0., 0., 0. };
  osg::Vec3d end{ 0., 0., 0. };
  line_seg_intersector_ = new osgUtil::LineSegmentIntersector(start, end);
}

double LaserScanner::getMaxDepth() const
{
  return this->max_laser_depth_;
}

double LaserScanner::getScan()
{
  line_seg_intersector_->reset();
  osgUtil::IntersectionVisitor iv(line_seg_intersector_.get());
  osg_scene_->accept(iv);

  //std::cout << "Lidar" << std::endl;
  osg::Vec3d robot_position{ robot_ptr_->getX(), robot_ptr_->getY(), 0. };
  osg::Vec3d laser_start{ robot_position + lidar_position_offset_ };
  line_seg_intersector_->setStart(laser_start);
  //std::cout << "laser start: " << laser_start[0] << ", " << laser_start[1] << ", "
    //<< laser_start[2] << std::endl;

  double robot_heading{ robot_ptr_->getHeading() };
  double laser_x{ max_laser_depth_ * cos(robot_heading) };
  double laser_y{ max_laser_depth_ * sin(robot_heading) };
  osg::Vec3d laser_xy{ laser_x, laser_y, 0. };
  osg::Vec3d laser_end{ laser_xy + laser_start };
  line_seg_intersector_->setEnd(laser_end);
  //std::cout << "laser end: " << laser_end[0] << ", " << laser_end[1] << ", "
    //<< laser_end[2] << std::endl;

  bool contains_intersections{ line_seg_intersector_->containsIntersections() };
  osg::Vec3d first_int =
      line_seg_intersector_->getFirstIntersection().getWorldIntersectPoint();
  //std::cout << "first int: " << first_int[0] << ", " << first_int[1] << ", "
    //<< first_int[2] << std::endl;

  double depth{ (first_int - laser_start).length() };

  if (contains_intersections)
    return depth;
  else
    return DBL_MAX;
}
