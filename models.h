#ifndef MODELS_H
#define MODELS_H
#include <osg/Node>
#include <osg/Vec3d>
// TODO what about ref_ptr

osg::ref_ptr<osg::Node> create_robot();
osg::ref_ptr<osg::Node> create_maze();
osg::ref_ptr<osg::Node> translate_model(osg::ref_ptr<osg::Node> model,
                                        osg::Vec3d &translation);

#endif // MODELS_H
