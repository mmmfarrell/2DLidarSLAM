#include "robotupdatecallback.h"
#include "robot.h"

#include <osg/Vec3d>
#include <osg/Quat>
#include <osg/Matrixd>
#include <osg/PositionAttitudeTransform>

RobotUpdateCallback::RobotUpdateCallback(robo::Robot* robot) :
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

  osg::Vec3d newRobotPosition{ robot_ptr_->getX(), robot_ptr_->getY(), 0. };
  pat->setPosition(newRobotPosition);

  osg::Matrixd heading_rot_mat;
  heading_rot_mat.makeRotate(robot_ptr_->getHeading() + osg::DegreesToRadians(55.), osg::Vec3(0, 0, 1));

  osg::Quat newRobotAttitude{ heading_rot_mat.getRotate() };
  pat->setAttitude(newRobotAttitude);

  traverse(node, nv);
}
