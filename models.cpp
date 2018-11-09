#include "models.h"
#include <string>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>

osg::ref_ptr<osg::Node> create_scaled_model(osg::ref_ptr<osg::Node> model,
                                            double bounding_radius)
{
  osg::BoundingSphere bb{ model->getBound() };

  osg::ref_ptr<osg::PositionAttitudeTransform> scale_trans{
    new osg::PositionAttitudeTransform };

  double bounding_box_radius{ bb.radius() };
  double radius_ratio{ bounding_radius / bounding_box_radius };
  scale_trans->setScale(osg::Vec3d(radius_ratio, radius_ratio, radius_ratio));
  scale_trans->addChild(model);

  return scale_trans.release();
}

osg::ref_ptr<osg::Node> translate_model_to_origin(osg::ref_ptr<osg::Node> model)
{
  osg::BoundingSphere bb{ model->getBound() };

  osg::ref_ptr<osg::PositionAttitudeTransform> position_trans{
    new osg::PositionAttitudeTransform };
  osg::Vec3d pos{ bb.center() };
  pos = osg::Vec3d(pos.x() * -1, pos.y() * -1, pos.z() * -1);
  position_trans->setPosition(pos);
  position_trans->addChild(model);
  return position_trans.release();
}

osg::ref_ptr<osg::Node> translate_model(osg::ref_ptr<osg::Node> model, osg::Vec3d &translation)
{
  osg::ref_ptr<osg::PositionAttitudeTransform> position_trans{
    new osg::PositionAttitudeTransform };
  //osg::Vec3d pos{  };
  //pos = osg::Vec3d(pos.x() * -1, pos.y() * -1, pos.z() * -1);
  position_trans->setPosition(translation);
  position_trans->addChild(model);
  return position_trans.release();
}

osg::ref_ptr<osg::Node> create_model(std::string file_name)
{
  osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(file_name);

  if (model.valid())
  {
    osg::StateSet *state_set{ model->getOrCreateStateSet() };
    state_set->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    state_set->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
  }
  return model.release();
}

osg::ref_ptr<osg::Node> create_robot()
{
  // TODO catch failed load
  std::string robot_file{ "/home/mmmfarrell/Downloads/r2d22.3ds" };
  //std::string robot_file{ "/home/mmmfarrell/Downloads/tread.3ds" };
  //std::string robot_file{ "/home/mmmfarrell/Downloads/irobot.3ds" };
  double robot_bound_radius{ 1. };

  osg::ref_ptr<osg::Node> model = create_model(robot_file);
  osg::ref_ptr<osg::Node> scaled_model =
      create_scaled_model(model, robot_bound_radius);
  osg::ref_ptr<osg::Node> model_at_origin =
      translate_model_to_origin(scaled_model);

  //osg::Vec3d robot_translation{ 0., 0., 100. };
  //osg::ref_ptr<osg::Node> translated_model =
      //translate_model(model_at_origin, robot_translation);

  //return translated_model.release();
  return model_at_origin.release();
}

osg::ref_ptr<osg::Node> create_maze()
{
  // TODO catch failed load
  std::string maze_file{ "/home/mmmfarrell/Downloads/Maze.3ds" };
  double maze_bound_radius{ 50. };

  osg::ref_ptr<osg::Node> model = create_model(maze_file);
  osg::ref_ptr<osg::Node> scaled_model =
      create_scaled_model(model, maze_bound_radius);
  osg::ref_ptr<osg::Node> translated_model =
      translate_model_to_origin(scaled_model);

  return translated_model.release();
}
