#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <map>
#include <memory>

#include <QMainWindow>
#include <QKeyEvent>
#include <QtCore>

#include "robot.h"
#include "laserscanner.h"
#include "robotmapper.h"
#include "slam2d.h"

class OSGWidget;
class QTimerEvent;
class QDockWidget;
class LaserScanWidget;
class MapViewer;

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

  QDockWidget *map_dock_widget_{ nullptr };
  MapViewer* map_view_widget_{ nullptr };

  QDockWidget *slam_dock_widget_{ nullptr };
  MapViewer* slam_view_widget_{ nullptr };

  std::unique_ptr<robo::Robot> robot_{ nullptr };

  std::unique_ptr<robo::LaserScanner> lidar_{ nullptr };
  void setupLidar();

  std::unique_ptr<robo::RobotMapper> robot_mapper_{ nullptr };
  robo::Slam2D slammer_;
  double robot_vel_std_dev_{ 0.1 };
  double robot_omega_std_dev_{ 0.1 };

  int velocity_scale_factor_;
  const int max_vel_scale_factor_{ 5 };
  void incrementVelocityScaleFactor();
  void decrementVelocityScaleFactor();

  void initKeysPressedMap();
  void handlePressedKeys();
  std::map<int, bool> keys_pressed_map_;

  void timerEvent(QTimerEvent *event);

  int dynamics_timer_id_;
  const double robot_dynamics_rate_hz_{ 100.0 };
  void dynamicsTimerEvent();

  int lidar_timer_id_;
  const double lidar_rate_hz_{ 5.0 };
  void lidarTimerEvent();

  void resetMappingMap();
  void resetSlamMap();
};

#endif  // MAINWINDOW_H
