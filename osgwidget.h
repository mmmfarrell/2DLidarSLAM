#ifndef MEEN_570_OSGWIDGET
#define MEEN_570_OSGWIDGET

#include <QOpenGLWidget>

#include <osg/ref_ptr>

namespace robo
{
class Robot;
}

namespace osg
{
class Group;
class Camera;
class Node;
class Geometry;
}

namespace osgViewer
{
class GraphicsWindowEmbedded;
class CompositeViewer;
class View;
}

namespace osgGA
{
class TrackballManipulator;
class EventQueue;
}

class OSGWidget : public QOpenGLWidget
{
  Q_OBJECT

public:
  OSGWidget(QWidget *parent = 0, Qt::WindowFlags f = 0);
  virtual ~OSGWidget();

  void displayRobot(robo::Robot* robot);

protected:
  virtual void paintEvent(QPaintEvent *paintEvent);
  virtual void paintGL();
  virtual void resizeGL(int width, int height);

  virtual void keyPressEvent(QKeyEvent *event);

  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseReleaseEvent(QMouseEvent *event);
  virtual void wheelEvent(QWheelEvent *event);
  virtual bool event(QEvent *event);

  osgGA::EventQueue *getEventQueue() const;

  void repaintOSGGraphicsAfterInteraction(QEvent *event);
  virtual void onResize(int width, int height);
  unsigned int getMouseButtonNumber(QMouseEvent *event);

  osg::Camera* createCamera();

  void setupViewCamera();
  void setCameraToTrackNode(osg::Node* node_to_track);

  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> graphics_window_;
  osg::ref_ptr<osgViewer::CompositeViewer> viewer_;
  osg::ref_ptr<osgViewer::View> view_;
  osg::ref_ptr<osg::Group> root_;

  robo::Robot *robot_ptr_;
};

#endif
