#ifndef LASERSCANWIDGET_H
#define LASERSCANWIDGET_H

#include <QWidget>

class QPaintEvent;

class LaserScanWidget : public QWidget
{
public:
  LaserScanWidget(QWidget *parent = 0);

  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;

  void setMaxLaserDepth(double max_laser_depth);
  void updateLaserScan(double new_scan);

protected:
  void paintEvent(QPaintEvent *event) override;
  double determineScaleFactor();

  double laser_scan_;
  double max_laser_depth_{ 0. };
};

#endif /* LASERSCANWIDGET_H */
