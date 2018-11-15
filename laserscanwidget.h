#ifndef LASERSCANWIDGET_H
#define LASERSCANWIDGET_H

#include <vector>
#include <QPainter>
#include <QWidget>

class QPaintEvent;

class LaserScanWidget : public QWidget
{
public:
  LaserScanWidget(QWidget *parent = 0);

  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;

  void setMaxLaserDepth(double max_laser_depth);
  void updateLaserScan(std::vector<float> new_scan);

protected:
  void paintEvent(QPaintEvent *event) override;
  void paintRobot(QPainter &painter);
  void paintLaserScan(QPainter &painter);

  double determineScaleFactor();

  std::vector<float> laser_scan_;
  double max_laser_depth_{ 0. };
  float min_laser_angle_rad_{ -M_PI };
  float max_laser_angle_rad_{ M_PI };
  float laser_angle_increment_{ M_PI / 60. };
  unsigned int number_laser_returns_{ static_cast<unsigned int>(
      (max_laser_angle_rad_ - min_laser_angle_rad_) / laser_angle_increment_) };
};

#endif /* LASERSCANWIDGET_H */
