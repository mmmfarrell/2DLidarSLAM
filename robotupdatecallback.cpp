#include "robotupdatecallback.h"
#include "robot.h"

#include <iostream>
#include <osg/Vec3d>
#include <osg/Quat>
#include <osg/Matrixd>
#include <osg/PositionAttitudeTransform>

RobotUpdateCallback::RobotUpdateCallback(robo::Robot* robot) :
  robot_ptr_{ robot }
{
  this->computeModelRotationMatrix();
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

  osg::Matrixd total_rot_mat{ heading_rot_mat * model_rotation_matrix_ };
  osg::Quat newRobotAttitude{ total_rot_mat.getRotate() };
  pat->setAttitude(newRobotAttitude);

  traverse(node, nv);
}

void RobotUpdateCallback::computeModelRotationMatrix()
{
  osg::Matrixd model_roll_mat;
  model_roll_mat.makeRotate(-0.0, osg::Vec3(1,0,0));

  osg::Matrixd model_yaw_mat;
  model_yaw_mat.makeRotate(osg::DegreesToRadians(0.), osg::Vec3(0,0,1));

  model_rotation_matrix_ = model_roll_mat * model_yaw_mat;
}
