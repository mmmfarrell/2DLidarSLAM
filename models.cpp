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

osg::ref_ptr<osg::Node> create_translated_model(osg::ref_ptr<osg::Node> model)
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

osg::ref_ptr<osg::Node> create_model(std::string file_name)
{
  osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(file_name);
  // osg::ref_ptr<osg::Node> model =
  // osgDB::readNodeFile("../meshes/robot/lost_robot.ive");
  // osg::ref_ptr<osg::Node> model =
  // osgDB::readNodeFile("/home/mmmfarrell/Downloads/turtlebot.dae");
   //osg::ref_ptr<osg::Node> model =
   //osgDB::readNodeFile("/home/mmmfarrell/Downloads/bb8/bb8.3ds");
  // osg::ref_ptr<osg::Node> model =
  // osgDB::readNodeFile("/home/mmmfarrell/Downloads/car.3ds");
  // osg::ref_ptr<osg::Node> model =
  // osgDB::readNodeFile("/home/mmmfarrell/Downloads/tread.3ds"); // works well
  // osg::ref_ptr<osg::Node> model =
  // osgDB::readNodeFile("/home/mmmfarrell/Downloads/irobot.3ds"); // works
  // osg::ref_ptr<osg::Node> model =
  // osgDB::readNodeFile("/home/mmmfarrell/Downloads/r2d22.3ds"); // workd not
  // as pretty
  // osg::ref_ptr<osg::Node> model =
  // osgDB::readNodeFile("/home/mmmfarrell/Downloads/r2d2.3ds"); // works

  if (model.valid())
  {
    osg::StateSet *state_set{ model->getOrCreateStateSet() };
    state_set->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    state_set->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
  }
  return model.release();
}

osg::ref_ptr<osg::Node> create_ironman(double boundingRadius)
{
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/Maze.3ds");
    osg::ref_ptr<osg::Node> model = create_model();
    osg::ref_ptr<osg::Node> scaledModel = create_scaled_model(model,boundingRadius);
    osg::ref_ptr<osg::Node> translatedModel = create_translated_model(scaledModel);

    return translatedModel.release();
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
      create_translated_model(scaled_model);

  return translated_model.release();
}
