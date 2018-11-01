#ifndef MEEN_570_OSGWIDGET
#define MEEN_570_OSGWIDGET

#include <QOpenGLWidget>

#include <osg/ref_ptr>

namespace osg
{
class Group;
class Camera;
class Geode;
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

protected:
  virtual void paintEvent(QPaintEvent *paintEvent);
  virtual void paintGL();
  virtual void resizeGL(int width, int height);

  virtual void keyPressEvent(QKeyEvent *event);
  virtual void keyReleaseEvent(QKeyEvent *event);

  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseReleaseEvent(QMouseEvent *event);
  virtual void wheelEvent(QWheelEvent *event);
  virtual bool event(QEvent *event);
  void repaint_osg_graphics_after_interaction(QEvent *event);

private:
  virtual void on_resize(int width, int height);
  osgGA::EventQueue *getEventQueue() const;

  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> mGraphicsWindow;
  osg::ref_ptr<osgViewer::CompositeViewer> mViewer;
  osg::ref_ptr<osgViewer::View> mView;
  osg::ref_ptr<osg::Group> mRoot;

  unsigned int get_mouse_button_number(QMouseEvent *event);
  osg::Camera* create_camera();
  osg::Geode* create_sphere();
  osgGA::TrackballManipulator* create_manipulator();
};

#endif
