#include "mainwindow.h"
#include "ui_mainwindowform.h"
#include "osgwidget.h"
#include "robot.h"
#include "laserscan.h"
#include "laserscanner.h"
#include "laserscanwidget.h"
#include "robotmapper.h"
#include "mapviewer.h"

#include "icp.h"
#include "roboutils.h"

#include <limits>

#include <QDockWidget>
#include <QKeyEvent>
#include <QImage>
#include <QTimerEvent>

#include <QDebug> // TODO remove
#include <iostream> // TODO remove

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow{parent},
  main_window_ui_{ new Ui::MainWindowForm },
  osg_widget_{ new OSGWidget{ this } },
  robot_{ new robo::Robot },
  robot_mapper_{ new robo::RobotMapper{ robot_.get() } }
{
  main_window_ui_->setupUi(this);

  laser_scan_dock_widget_ = new QDockWidget("Raw Laser Scan Data", this);
  this->addDockWidget(Qt::BottomDockWidgetArea, laser_scan_dock_widget_);
  laser_scan_widget_ = new LaserScanWidget;
  laser_scan_dock_widget_->setWidget(laser_scan_widget_);

  map_dock_widget_ = new QDockWidget("Robot Map", this);
  this->addDockWidget(Qt::BottomDockWidgetArea, map_dock_widget_);
  map_view_widget_ = new MapViewer;
  map_dock_widget_->setWidget(map_view_widget_);

  QImage map_image;
  robot_mapper_->getMap(map_image);
  map_view_widget_->setImage(map_image);

  slam_dock_widget_ = new QDockWidget("SLAM Map", this);
  this->addDockWidget(Qt::BottomDockWidgetArea, slam_dock_widget_);
  slam_view_widget_ = new MapViewer;
  slam_dock_widget_->setWidget(slam_view_widget_);

  slammer_.getMap(map_image);
  slam_view_widget_->setImage(map_image);

  this->setupLidar();

  osg_widget_->displayRobot(robot_.get());
  this->setCentralWidget(osg_widget_);
  
  velocity_scale_factor_ = 1;

  double robot_dynamics_rate_ms{ 1. / robot_dynamics_rate_hz_ * 1000. };
  dynamics_timer_id_ = this->startTimer(robot_dynamics_rate_ms);

  double lidar_rate_ms{ 1. / lidar_rate_hz_ * 1000. };
  lidar_timer_id_ = this->startTimer(lidar_rate_ms);
}

MainWindow::~MainWindow()
{
  this->killTimer(dynamics_timer_id_);
  this->killTimer(lidar_timer_id_);
  delete main_window_ui_;
}

void MainWindow::on_actionAbout_triggered()
{
  std::cout << "push" << std::endl;
}

void MainWindow::on_actionExit_triggered()
{
  QApplication::quit();
}

void MainWindow::on_actionIncrease_Speed_triggered()
{
  this->incrementVelocityScaleFactor();
}

void MainWindow::on_actionDecrease_Speed_triggered()
{
  this->decrementVelocityScaleFactor();
}

void MainWindow::on_actionReset_Pose_triggered()
{
  robot_->resetState();
  this->resetMappingMap();
  this->resetSlamMap();
}

void MainWindow::on_actionMapShowHide_triggered()
{
  if (map_dock_widget_->isVisible())
    map_dock_widget_->setVisible(false);
  else
    map_dock_widget_->setVisible(true);
}

void MainWindow::on_actionMapReset_Map_triggered()
{
  this->resetMappingMap();
}

void MainWindow::on_actionMapSave_Map_triggered()
{
  std::cout << map_view_widget_->saveMap("occupancy_grid_map.png") << "save"
            << std::endl;
}

void MainWindow::on_actionSlamShowHide_triggered()
{
  if (slam_dock_widget_->isVisible())
    slam_dock_widget_->setVisible(false);
  else
    slam_dock_widget_->setVisible(true);
}

void MainWindow::on_actionSlamReset_Map_triggered()
{
  this->resetSlamMap();
}

void MainWindow::on_actionSlamSave_Map_triggered()
{
  std::cout << "push" << std::endl;
}

void MainWindow::on_actionShortcuts_triggered()
{
  std::cout << "push" << std::endl;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
  switch (event->key())
  {
    case Qt::Key_Up:
    case Qt::Key_Down:
    case Qt::Key_Right:
    case Qt::Key_Left:
      keys_pressed_map_[event->key()] = true;
      break;
    case Qt::Key_PageUp:
      this->incrementVelocityScaleFactor();
      break;
    case Qt::Key_PageDown:
      this->decrementVelocityScaleFactor();
      break;
    case Qt::Key_R:
      this->resetMappingMap();
      this->resetSlamMap();
      break;
    case Qt::Key_Escape:
    case Qt::Key_Q:
      QApplication::quit();
      break;
  }

  QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
  switch (event->key())
  {
    case Qt::Key_Up:
    case Qt::Key_Down:
    case Qt::Key_Right:
    case Qt::Key_Left:
      keys_pressed_map_[event->key()] = false;
  }

  QMainWindow::keyPressEvent(event);
}

void MainWindow::setupLidar()
{
  lidar_.reset(new robo::LaserScanner(osg_widget_->getScene(), robot_.get()));
}

void MainWindow::incrementVelocityScaleFactor()
{
  if (velocity_scale_factor_ != max_vel_scale_factor_)
    velocity_scale_factor_++;
  qDebug() << "Velocity scale factor: " << velocity_scale_factor_;
}

void MainWindow::decrementVelocityScaleFactor()
{
  if (velocity_scale_factor_ != 1)
    velocity_scale_factor_--;
  qDebug() << "Velocity scale factor: " << velocity_scale_factor_;
}

void MainWindow::initKeysPressedMap()
{
  keys_pressed_map_[Qt::Key_Up] = false;
  keys_pressed_map_[Qt::Key_Down] = false;
  keys_pressed_map_[Qt::Key_Right] = false;
  keys_pressed_map_[Qt::Key_Left] = false;
}

void MainWindow::handlePressedKeys()
{
  double desired_velocity{ 0.0 };
  double base_velocity{ 2.0 };
  if (keys_pressed_map_[Qt::Key_Up])
    desired_velocity = base_velocity * velocity_scale_factor_;
  else if (keys_pressed_map_[Qt::Key_Down])
    desired_velocity = -base_velocity * velocity_scale_factor_;
  robot_->setDesiredVelocity(desired_velocity);

  double desired_omega{ 0.0 };
  double base_omega{ 0.4 };
  if (keys_pressed_map_[Qt::Key_Right])
    desired_omega = -base_omega * velocity_scale_factor_;
  else if (keys_pressed_map_[Qt::Key_Left])
    desired_omega = base_omega * velocity_scale_factor_;
  robot_->setDesiredOmega(desired_omega);
}

void MainWindow::timerEvent(QTimerEvent *event)
{
  if (event->timerId() == dynamics_timer_id_)
    this->dynamicsTimerEvent();
  else if (event->timerId() == lidar_timer_id_)
    this->lidarTimerEvent();
}

void MainWindow::dynamicsTimerEvent()
{
  this->handlePressedKeys();

  double robot_dynamics_rate_s{ 1. / robot_dynamics_rate_hz_ };
  robot_->propagateDynamics(robot_dynamics_rate_s);

  double dt{ 1. / this->robot_dynamics_rate_hz_ };
  double robot_vel{ robot_->getNoisyVelocity(robot_vel_std_dev_) };
  double robot_omega{ robot_->getNoisyOmega(robot_omega_std_dev_) };

  slammer_.updateRobotPoseEstimate(robot_vel, robot_omega, dt);
}

void MainWindow::lidarTimerEvent()
{
  robo::LaserScan laser_scan;
  lidar_->getScan(laser_scan);

  laser_scan_widget_->updateLaserScan(laser_scan);

  robot_mapper_->updateMap(laser_scan);

  QImage map_image;
  robot_mapper_->getMap(map_image);
  map_view_widget_->setImage(map_image);

  slammer_.updateMap(laser_scan);
  slammer_.getMap(map_image);
  slam_view_widget_->setImage(map_image);
}

void MainWindow::resetMappingMap()
{
  robot_mapper_->resetMap();
}

void MainWindow::resetSlamMap()
{
  slammer_.resetRobotPose(robot_->getX(), robot_->getY(), robot_->getHeading());
  slammer_.resetMap();
}
