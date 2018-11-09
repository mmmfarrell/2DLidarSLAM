#include "osgwidget.h"
#include "robotupdatecallback.h"
#include "models.h"

#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/PositionAttitudeTransform>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/EventQueue>
#include <osgGA/GUIEventAdapter>
#include <osgUtil/SmoothingVisitor>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#include <osgDB/ReadFile>

#include <stdexcept>
#include <vector>

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

  osg::ref_ptr<osg::Node> maze{ create_maze()};
  root_->addChild(maze);

  viewer_->addView(view_);
  viewer_->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
  viewer_->realize();
  view_->home();

  this->setupViewCamera();

  this->setFocusPolicy(Qt::StrongFocus);
  this->setMinimumSize(100, 100);
  this->setMouseTracking(true);
  this->update();
}

OSGWidget::~OSGWidget()
{
}

void OSGWidget::displayRobot(Robot *robot)
{
  robot_ptr_ = robot;

  osg::ref_ptr<osg::PositionAttitudeTransform> robot_transform{
    new osg::PositionAttitudeTransform };
  osg::ref_ptr<RobotUpdateCallback> robotUpdateCallbackPtr{
    new RobotUpdateCallback(robot_ptr_) };
  robot_transform->setUpdateCallback(robotUpdateCallbackPtr);

  osg::ref_ptr<osg::Node> robot_node{ create_robot()};
  robot_transform->addChild(robot_node);
  root_->addChild(robot_transform);

  this->setCameraToTrackNode(robot_node.get());
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
  if (event->key() == Qt::Key_H)
  {
    view_->home();
  }
  else
  {
    QString key_string{ event->text() };
    const char *key_data{ key_string.toLocal8Bit().data() };
    this->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(*key_data));
  }

  QOpenGLWidget::keyPressEvent(event);
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

void OSGWidget::setupViewCamera()
{
  view_->setLightingMode(osg::View::SKY_LIGHT);

  osg::Light *light{ view_->getLight() };
  osg::Vec4 light_pos{ 0.f, 0.f, 100.f, 0.f };
  light->setPosition(light_pos);
}

void OSGWidget::setCameraToTrackNode(osg::Node* node_to_track)
{
  osgGA::NodeTrackerManipulator::TrackerMode trackerMode =
      osgGA::NodeTrackerManipulator::NODE_CENTER;
  osgGA::NodeTrackerManipulator::RotationMode rotationMode =
      osgGA::NodeTrackerManipulator::ELEVATION_AZIM;

  osg::ref_ptr<osgGA::NodeTrackerManipulator> manipulator{
    new osgGA::NodeTrackerManipulator };

  osg::Vec3 home_eye_position{ -30.f, 0.f, 30.f };
  osg::Vec3 home_center_position{ 0.f, 0.f, 0.f };
  osg::Vec3 home_up_direction_vector{ 0.f, 0.f, 1.f };
  manipulator->setHomePosition(home_eye_position, home_center_position,
                               home_up_direction_vector);

  manipulator->setTrackerMode(trackerMode);
  manipulator->setRotationMode(rotationMode);
  manipulator->setTrackNode(node_to_track);
  view_->setCameraManipulator(manipulator);
}
