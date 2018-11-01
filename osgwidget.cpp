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

#include <stdexcept>
#include <vector>

#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>

OSGWidget::OSGWidget(QWidget *parent, Qt::WindowFlags flags) : 
  QOpenGLWidget{ parent, flags },
  mGraphicsWindow{ new osgViewer::GraphicsWindowEmbedded{
        this->x(), this->y(), this->width(), this->height() } },
  mViewer{ new osgViewer::CompositeViewer },
  mView{ new osgViewer::View},
  mRoot{ new osg::Group }
{
  osg::ref_ptr<osg::Camera> camera{ this->create_camera() };
  mView->setCamera(camera);

  mView->setSceneData(mRoot.get());
  mView->addEventHandler(new osgViewer::StatsHandler);

  osg::ref_ptr<osgGA::TrackballManipulator> manipulator{
    this->create_manipulator() };
  mView->setCameraManipulator(manipulator);

  mViewer->addView(mView);
  mViewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
  mViewer->realize();
  mView->home();

  osg::Geode *geode{ this->create_sphere() };
  mRoot->addChild(geode);

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
  mViewer->frame();
}

void OSGWidget::resizeGL(int width, int height)
{
  this->getEventQueue()->windowResize(this->x(), this->y(), width, height);
  mGraphicsWindow->resized(this->x(), this->y(), width, height);

  this->on_resize(width, height);
}

void OSGWidget::keyPressEvent(QKeyEvent *event)
{
  QString keyString{ event->text() };
  const char *keyData{ keyString.toLocal8Bit().data() };

  if (event->key() == Qt::Key_H)
  {
    mView->home();
    return;
  }

  this->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::keyReleaseEvent(QKeyEvent *event)
{
  QString keyString{ event->text() };
  const char *keyData{ keyString.toLocal8Bit().data() };

  this->getEventQueue()->keyRelease(
      osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::mouseMoveEvent(QMouseEvent *event)
{
  auto pixelRatio{ this->devicePixelRatio() };

  this->getEventQueue()->mouseMotion(
      static_cast<float>(event->x() * pixelRatio),
      static_cast<float>(event->y() * pixelRatio));
}

void OSGWidget::mousePressEvent(QMouseEvent *event)
{
  unsigned int button{ this->get_mouse_button_number(event) };

  auto pixelRatio{ this->devicePixelRatio() };

  this->getEventQueue()->mouseButtonPress(
      static_cast<float>(event->x() * pixelRatio),
      static_cast<float>(event->y() * pixelRatio), button);
}

void OSGWidget::mouseReleaseEvent(QMouseEvent *event)
{
  unsigned int button{ this->get_mouse_button_number(event) };

  auto pixelRatio{ this->devicePixelRatio() };

  this->getEventQueue()->mouseButtonRelease(
      static_cast<float>(pixelRatio * event->x()),
      static_cast<float>(pixelRatio * event->y()), button);
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

  repaint_osg_graphics_after_interaction(event);

  return handled;
}

void OSGWidget::repaint_osg_graphics_after_interaction(QEvent *event)
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

void OSGWidget::on_resize(int width, int height)
{
  std::vector<osg::Camera *> cameras;
  mViewer->getCameras(cameras);

  int viewportX{ 0 };
  int viewportY{ 0 };
  auto pixelRatio{ this->devicePixelRatio() };
  int viewportWidth{ width * pixelRatio };
  int viewportHeight{ height * pixelRatio };
  cameras[0]->setViewport(viewportX, viewportY, viewportWidth, viewportHeight);
}

osgGA::EventQueue *OSGWidget::getEventQueue() const
{
  osgGA::EventQueue *eventQueue{ mGraphicsWindow->getEventQueue() };

  if (eventQueue)
    return eventQueue;
  else
    throw std::runtime_error("Unable to obtain valid event queue");
}

unsigned int OSGWidget::get_mouse_button_number(QMouseEvent *event)
{
  unsigned int button{ 0 };

  unsigned int leftMouseButton{ 1 };
  unsigned int middleMouseButton{ 2 };
  unsigned int rightMouseButton{ 3 };

  switch (event->button())
  {
    case Qt::LeftButton:
      button = leftMouseButton;
      break;

    case Qt::MiddleButton:
      button = middleMouseButton;
      break;

    case Qt::RightButton:
      button = rightMouseButton;
      break;

    default:
      break;
  }

  return button;
}

osg::Camera* OSGWidget::create_camera()
{
  osg::Camera *camera{ new osg::Camera };

  int viewportX{ 0 };
  int viewportY{ 0 };
  auto pixelRatio{ this->devicePixelRatio() };
  int viewportWidth{ this->width() * pixelRatio };
  int viewportHeight{ this->height() * pixelRatio };
  camera->setViewport(viewportX, viewportY, viewportWidth, viewportHeight);

  osg::Vec4 blackColorRGBA{ 0.f, 0.f, 0.f, 1.f };
  camera->setClearColor(blackColorRGBA);

  float cameraFOV{ 45.0 };
  float cameraMinVisibleDistance{ 1.0 };
  float cameraMaxVisibleDistance{ 1000.0 };
  float aspectRatio{ static_cast<float>(this->width()) /
                     static_cast<float>(this->height()) };
  camera->setProjectionMatrixAsPerspective(cameraFOV, aspectRatio,
                                           cameraMinVisibleDistance,
                                           cameraMaxVisibleDistance);

  camera->setGraphicsContext(this->mGraphicsWindow);

  return camera;
}

osg::Geode *OSGWidget::create_sphere()
{
  osg::Vec3 sphereCenter{ 0.f, 0.f, 0.f };
  float sphereRadius{ 2.0 };
  osg::Sphere *sphere{ new osg::Sphere{ sphereCenter, sphereRadius } };
  osg::ShapeDrawable *sd{ new osg::ShapeDrawable{ sphere } };

  osg::Vec4 aquaColorRGBA{ 0.f, 1.f, 1.f, 1.f };
  sd->setColor(aquaColorRGBA);
  sd->setName("Sphere");

  osg::Geode *geode{ new osg::Geode };
  geode->addDrawable(sd);

  osg::Material *material{ new osg::Material };
  material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);

  osg::StateSet *stateSet{ geode->getOrCreateStateSet() };
  stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
  stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

  return geode;
}

osgGA::TrackballManipulator *OSGWidget::create_manipulator()
{
  osgGA::TrackballManipulator *manipulator{ new osgGA::TrackballManipulator };
  manipulator->setAllowThrow(false);

  osg::Vec3 homeEyePosition{ 0.f, -20.f, 3.f };
  osg::Vec3 homeCenterPosition{ 0.f, 0.f, 0.f };
  osg::Vec3 homeUpDirectionVector{ 0.f, 0.f, 1.f };
  manipulator->setHomePosition(homeEyePosition, homeCenterPosition,
                               homeUpDirectionVector);
  return manipulator;
}
