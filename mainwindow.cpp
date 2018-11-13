#include "mainwindow.h"
#include "ui_mainwindowform.h"
#include "osgwidget.h"
#include "robot.h"

#include <QDockWidget>
#include <QKeyEvent>
#include <QTimerEvent>

#include <QDebug> // TODO remove

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow{parent},
  main_window_ui_{ new Ui::MainWindowForm },
  osg_widget_{ new OSGWidget{ this } },
  robot_{ new robo::Robot }
{
  osg_widget_->displayRobot(robot_.get());

  main_window_ui_->setupUi(this);
  this->setCentralWidget(osg_widget_);
  
  velocity_scale_factor_ = 1;

  double robot_dynamics_rate_ms{ 1. / robot_dynamics_rate_hz_ * 1000. };
  timer_id_ = this->startTimer(robot_dynamics_rate_ms);
}

MainWindow::~MainWindow()
{
  this->killTimer(timer_id_);
  delete main_window_ui_;
}

void MainWindow::on_actionExit_triggered()
{
  QApplication::quit();
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
  double base_velocity{ 1.0 };
  if (keys_pressed_map_[Qt::Key_Up])
    desired_velocity = base_velocity * velocity_scale_factor_;
  else if (keys_pressed_map_[Qt::Key_Down])
    desired_velocity = -base_velocity * velocity_scale_factor_;
  robot_->setDesiredVelocity(desired_velocity);

  double desired_omega{ 0.0 };
  double base_omega{ 0.25 };
  if (keys_pressed_map_[Qt::Key_Right])
    desired_omega = -base_omega * velocity_scale_factor_;
  else if (keys_pressed_map_[Qt::Key_Left])
    desired_omega = base_omega * velocity_scale_factor_;
  robot_->setDesiredOmega(desired_omega);
}

void MainWindow::timerEvent(QTimerEvent *)
{
  this->handlePressedKeys();

  double robot_dynamics_rate_s{ 1. / robot_dynamics_rate_hz_ };
  robot_->propagateDynamics(robot_dynamics_rate_s);
}
