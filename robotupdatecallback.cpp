#include "robotupdatecallback.h"
#include "robot.h"
#include "pose.h"

#include <iostream>
#include <osg/Vec3d>
#include <osg/PositionAttitudeTransform>

RobotUpdateCallback::RobotUpdateCallback(Robot* robot) :
  robot_ptr_{ robot }
{
}

RobotUpdateCallback::~RobotUpdateCallback()
{
}

void RobotUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{
  osg::PositionAttitudeTransform *pat{
    dynamic_cast<osg::PositionAttitudeTransform *>(node) };
  if (pat == nullptr)
    return;

  Pose robot_pose{ robot_ptr_->getPose() };
  std::cout << "actual position: " << robot_pose.x << std::endl;
  osg::Vec3d newRobotPosition{ robot_pose.x, robot_pose.y, 0. };
  pat->setPosition(newRobotPosition);

  traverse(node, nv);
}
