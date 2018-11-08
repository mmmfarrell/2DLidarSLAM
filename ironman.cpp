#include "ironman.h"
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>

//The model has it's own scale.  This function creates a node to scale the model to fit inside the bounding sphere defined by the radius.
osg::ref_ptr<osg::Node> create_scaled_model(osg::ref_ptr<osg::Node> model, double boundingRadius)
{
    osg::BoundingSphere bb = model->getBound();

     osg::ref_ptr<osg::PositionAttitudeTransform> scaleTrans = new osg::PositionAttitudeTransform;
     double boundingBoxRadius = bb.radius();
     double radiusRatio{boundingRadius/boundingBoxRadius};
     scaleTrans->setScale(osg::Vec3d(radiusRatio,radiusRatio,radiusRatio));
     scaleTrans->addChild(model);

     return scaleTrans.release();
}

//The model is at it's own position in 3d space.  This funciton creates a node to position the model at the origin.
osg::ref_ptr<osg::Node> create_translated_model(osg::ref_ptr<osg::Node> model)
{
    osg::BoundingSphere bb = model->getBound();
    osg ::ref_ptr<osg::PositionAttitudeTransform> positionTrans = new osg::PositionAttitudeTransform;
    osg::Vec3d pos=bb.center();
    pos=osg::Vec3d(pos.x()*-1,pos.y()*-1,pos.z()*-1);
    positionTrans->setPosition(pos);
    positionTrans->addChild(model);
    return positionTrans.release();
}


osg::ref_ptr<osg::Node> create_model()
{
    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/Maze.3ds");
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("../meshes/robot/lost_robot.ive");
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/turtlebot.dae");
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/bb8/bb8.3ds");
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/car.3ds");
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/tread.3ds"); // works well
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/irobot.3ds"); // works
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/r2d22.3ds"); // workd not as pretty
    //osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("/home/mmmfarrell/Downloads/r2d2.3ds"); // works

    if(model.valid())
    {
        osg::StateSet* stateSet = model->getOrCreateStateSet();
        stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
        stateSet->setMode(GL_RESCALE_NORMAL ,osg::StateAttribute::ON);
    }
    return model.release();
}

osg::ref_ptr<osg::Node> create_ironman(double boundingRadius)
{
    osg::ref_ptr<osg::Node> model = create_model();
    osg::ref_ptr<osg::Node> scaledModel = create_scaled_model(model,boundingRadius);
    osg::ref_ptr<osg::Node> translatedModel = create_translated_model(scaledModel);

    return translatedModel.release();
}
