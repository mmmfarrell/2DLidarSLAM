#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <map>
#include <memory>

#include <QMainWindow>
#include <QKeyEvent>
#include <QtCore>

#include "robot.h"

class OSGWidget;
class QTimerEvent;
class QDockWidget;
class LaserScanWidget;

namespace Ui
{
class MainWindowForm;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

public slots:
  void on_actionExit_triggered();

  void keyPressEvent(QKeyEvent *event);
  void keyReleaseEvent(QKeyEvent *event);

protected:
  Ui::MainWindowForm *main_window_ui_;
  OSGWidget *osg_widget_{ nullptr };

  QDockWidget *laser_scan_dock_widget_{ nullptr };
  LaserScanWidget* laser_scan_widget_{ nullptr };

  std::unique_ptr<robo::Robot> robot_{ nullptr };

  int velocity_scale_factor_;
  const int max_vel_scale_factor_{ 5 };
  void incrementVelocityScaleFactor();
  void decrementVelocityScaleFactor();

  void initKeysPressedMap();
  void handlePressedKeys();
  std::map<int, bool> keys_pressed_map_;

  int timer_id_;
  const double robot_dynamics_rate_hz_{ 100.0 };
  void timerEvent(QTimerEvent *);
};

#endif  // MAINWINDOW_H
