#include "mainwindow.h"
#include "ui_mainwindowform.h"
#include "osgwidget.h"
#include "robot.h"

#include <QDockWidget>
#include <QKeyEvent>
#include <QTimerEvent>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow{parent},
  main_window_ui_{ new Ui::MainWindowForm },
  osg_widget_{ new OSGWidget{ this } },
  robot_{ new robo::Robot }
{
  osg_widget_->displayRobot(robot_.get());

  main_window_ui_->setupUi(this);
  this->setCentralWidget(osg_widget_);
  
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
      robot_->setDesiredVelocity(1.0);
      break;
    case Qt::Key_Down:
      robot_->setDesiredVelocity(-1.0);
      break;
    case Qt::Key_Right:
      robot_->setDesiredOmega(-1.0);
      break;
    case Qt::Key_Left:
      robot_->setDesiredOmega(1.0);
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
      robot_->setDesiredVelocity(0.0);
      break;
    case Qt::Key_Down:
      robot_->setDesiredVelocity(0.0);
      break;
    case Qt::Key_Right:
      robot_->setDesiredOmega(0.0);
      break;
    case Qt::Key_Left:
      robot_->setDesiredOmega(0.0);
      break;
  }

  QMainWindow::keyPressEvent(event);
}

void MainWindow::timerEvent(QTimerEvent *)
{
  double robot_dynamics_rate_s{ 1. / robot_dynamics_rate_hz_ };
  robot_->propagateDynamics(robot_dynamics_rate_s);
}
