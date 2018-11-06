#include "robotupdatecallback.h"
#include "robot.h"
#include "pose.h"

#include <iostream>
#include <osg/Vec3d>
#include <osg/Quat>
#include <osg/Matrixd>
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
  //osg::Vec3d newRobotPosition{ robot_pose.x, robot_pose.y, 0. };
  //pat->setPosition(newRobotPosition);

  osg::Quat attitude{ pat->getAttitude() };

  osg::Matrixd mxR;
  mxR.makeRotate(osg::DegreesToRadians(robot_pose.x),osg::Vec3(0,1,0));

  osg::Matrixd mxP;
  mxP.makeRotate(osg::DegreesToRadians(90.),osg::Vec3(1,0,0));
  osg::Matrixd mxH;
  mxH.makeRotate(osg::DegreesToRadians(45.),osg::Vec3(0,0,1));
  osg::Matrixd total_transform{ mxR * mxP * mxH };
  osg::Quat newRobotAttitude{ total_transform.getRotate() };
  pat->setAttitude(newRobotAttitude);

  traverse(node, nv);
}
