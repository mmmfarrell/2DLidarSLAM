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
  void setMinLaserAngle(double min_laser_angle);
  void setMaxLaserAngle(double max_laser_angle);
  void setLaserAngleIncrement(double laser_angle_increment);

  void updateLaserScan(std::vector<float> new_scan);

protected:
  void paintEvent(QPaintEvent *event) override;
  void paintRobot(QPainter &painter);
  void paintLaserScan(QPainter &painter);

  double determineScaleFactor();

  std::vector<float> laser_scan_;
  double max_laser_depth_{ 0. };
  double min_laser_angle_rad_{ 0 };
  double max_laser_angle_rad_{ 0 };
  double laser_angle_increment_{ 0 };
  unsigned int number_laser_returns_{ 0 };

  void recalculateNumberLaserReturns();
};

#endif /* LASERSCANWIDGET_H */
