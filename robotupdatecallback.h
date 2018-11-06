#ifndef ROBOTUPDATECALLBACK_H
#define ROBOTUPDATECALLBACK_H

#include <osg/NodeCallback>

class Robot;

class RobotUpdateCallback : public osg::NodeCallback
{
public:
  RobotUpdateCallback(Robot* robot);
  virtual ~RobotUpdateCallback();

  virtual void operator()(osg::Node *node, osg::NodeVisitor *nv);

private:
  Robot *robot_ptr_;
};

#endif /* ROBOTUPDATECALLBACK_H */
