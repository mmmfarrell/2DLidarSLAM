#include "osgwidget.h"

#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Material>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osgGA/TrackballManipulator>
#include <osgGA/EventQueue>
#include <osgGA/GUIEventAdapter>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#include<osgDB/ReadFile>

#include <stdexcept>
#include <vector>
#include <iostream> // TODO remove

#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>

OSGWidget::OSGWidget(QWidget *parent, Qt::WindowFlags flags) : 
  QOpenGLWidget{ parent, flags },
  graphics_window_{ new osgViewer::GraphicsWindowEmbedded{
        this->x(), this->y(), this->width(), this->height() } },
  viewer_{ new osgViewer::CompositeViewer },
  view_{ new osgViewer::View},
  root_{ new osg::Group }
{
  osg::ref_ptr<osg::Camera> camera{ this->createCamera() };
  view_->setCamera(camera);

  view_->setSceneData(root_.get());
  view_->addEventHandler(new osgViewer::StatsHandler);

  osg::ref_ptr<osgGA::TrackballManipulator> manipulator{
    this->createManipulator() };
  view_->setCameraManipulator(manipulator);

  viewer_->addView(view_);
  viewer_->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
  viewer_->realize();
  view_->home();

  osg::ref_ptr<osg::Node> robotNode{ this-> createRobot() };
  root_->addChild(robotNode);

  this->setFocusPolicy(Qt::StrongFocus);
  this->setMinimumSize(100, 100);
  this->setMouseTracking(true);
  this->update();
}

OSGWidget::~OSGWidget()
{
}

void OSGWidget::paintEvent(QPaintEvent *)
{
  this->makeCurrent();

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  this->paintGL();

  painter.end();

  this->doneCurrent();
}

void OSGWidget::paintGL()
{
  viewer_->frame();
}

void OSGWidget::resizeGL(int width, int height)
{
  this->getEventQueue()->windowResize(this->x(), this->y(), width, height);
  graphics_window_->resized(this->x(), this->y(), width, height);

  this->onResize(width, height);
}

void OSGWidget::keyPressEvent(QKeyEvent *event)
{
  QString key_string{ event->text() };
  const char *key_data{ key_string.toLocal8Bit().data() };

  if (event->key() == Qt::Key_H)
  {
    view_->home();
    return;
  }

  this->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(*key_data));
}

void OSGWidget::keyReleaseEvent(QKeyEvent *event)
{
  QString key_string{ event->text() };
  const char *key_data{ key_string.toLocal8Bit().data() };

  this->getEventQueue()->keyRelease(
      osgGA::GUIEventAdapter::KeySymbol(*key_data));
}

void OSGWidget::mouseMoveEvent(QMouseEvent *event)
{
  auto pixel_ratio{ this->devicePixelRatio() };

  this->getEventQueue()->mouseMotion(
      static_cast<float>(event->x() * pixel_ratio),
      static_cast<float>(event->y() * pixel_ratio));
}

void OSGWidget::mousePressEvent(QMouseEvent *event)
{
  unsigned int button{ this->getMouseButtonNumber(event) };

  auto pixel_ratio{ this->devicePixelRatio() };

  this->getEventQueue()->mouseButtonPress(
      static_cast<float>(event->x() * pixel_ratio),
      static_cast<float>(event->y() * pixel_ratio), button);
}

void OSGWidget::mouseReleaseEvent(QMouseEvent *event)
{
  unsigned int button{ this->getMouseButtonNumber(event) };

  auto pixel_ratio{ this->devicePixelRatio() };

  this->getEventQueue()->mouseButtonRelease(
      static_cast<float>(pixel_ratio * event->x()),
      static_cast<float>(pixel_ratio * event->y()), button);
}

void OSGWidget::wheelEvent(QWheelEvent *event)
{
  event->accept();
  int delta{ event->delta() };

  osgGA::GUIEventAdapter::ScrollingMotion motion{
    delta > 0 ? osgGA::GUIEventAdapter::SCROLL_UP :
                osgGA::GUIEventAdapter::SCROLL_DOWN };

  this->getEventQueue()->mouseScroll(motion);
}

bool OSGWidget::event(QEvent *event)
{
  bool handled{ QOpenGLWidget::event(event) };

  repaintOSGGraphicsAfterInteraction(event);

  return handled;
}

void OSGWidget::repaintOSGGraphicsAfterInteraction(QEvent *event)
{
  switch (event->type())
  {
    case QEvent::KeyPress:
    case QEvent::KeyRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::Wheel:
      this->update();
      break;

    default:
      break;
  }
}

void OSGWidget::onResize(int width, int height)
{
  std::vector<osg::Camera *> cameras;
  viewer_->getCameras(cameras);

  int viewport_x{ 0 };
  int viewport_y{ 0 };
  auto pixel_ratio{ this->devicePixelRatio() };
  int viewport_width{ width * pixel_ratio };
  int viewport_height{ height * pixel_ratio };
  cameras[0]->setViewport(viewport_x, viewport_y, viewport_width,
                          viewport_height);
}

osgGA::EventQueue *OSGWidget::getEventQueue() const
{
  osgGA::EventQueue *eventQueue{ graphics_window_->getEventQueue() };

  if (eventQueue)
    return eventQueue;
  else
    throw std::runtime_error("Unable to obtain valid event queue");
}

unsigned int OSGWidget::getMouseButtonNumber(QMouseEvent *event)
{
  unsigned int button{ 0 };

  unsigned int left_mouse_button{ 1 };
  unsigned int middle_mouse_button{ 2 };
  unsigned int right_mouse_button{ 3 };

  switch (event->button())
  {
    case Qt::LeftButton:
      button = left_mouse_button;
      break;

    case Qt::MiddleButton:
      button = middle_mouse_button;
      break;

    case Qt::RightButton:
      button = right_mouse_button;
      break;

    default:
      break;
  }

  return button;
}

osg::Camera* OSGWidget::createCamera()
{
  osg::Camera *camera{ new osg::Camera };

  int viewport_x{ 0 };
  int viewport_y{ 0 };
  auto pixel_ratio{ this->devicePixelRatio() };
  int viewport_width{ this->width() * pixel_ratio };
  int viewport_height{ this->height() * pixel_ratio };
  camera->setViewport(viewport_x, viewport_y, viewport_width, viewport_height);

  osg::Vec4 white_color_rgba{ 1.f, 1.f, 1.f, 1.f };
  camera->setClearColor(white_color_rgba);

  float camera_fov{ 45.0f };
  float camera_min_visible_distance{ 1.0f };
  float camera_max_visible_distance{ 1000.0f };
  float aspect_ratio{ static_cast<float>(this->width()) /
                      static_cast<float>(this->height()) };
  camera->setProjectionMatrixAsPerspective(camera_fov, aspect_ratio,
                                           camera_min_visible_distance,
                                           camera_max_visible_distance);

  camera->setGraphicsContext(this->graphics_window_);

  return camera;
}


osgGA::TrackballManipulator *OSGWidget::createManipulator()
{
  osgGA::TrackballManipulator *manipulator{ new osgGA::TrackballManipulator };
  manipulator->setAllowThrow(false);

  osg::Vec3 home_eye_position{ 0.f, -20.f, 3.f };
  osg::Vec3 home_center_position{ 0.f, 0.f, 0.f };
  osg::Vec3 home_up_direction_vector{ 0.f, 0.f, 1.f };
  manipulator->setHomePosition(home_eye_position, home_center_position,
                               home_up_direction_vector);
  return manipulator;
}

osg::Node *OSGWidget::createRobot()
{
  std::string robot_mesh_file{ "../meshes/robot.stl" };
  osg::Node* robot_node{ osgDB::readNodeFile(robot_mesh_file)};

  osg::ref_ptr<osg::Material> material{ new osg::Material };
  material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
  osg::Vec4 black_color_rgba{ 0.f, 0.f, 0.f, 1.f };
  material->setDiffuse(osg::Material::FRONT_AND_BACK, black_color_rgba);

  osg::StateSet *stateSet{ robot_node->getOrCreateStateSet() };
  stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
  stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
  return robot_node;
}
