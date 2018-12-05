#ifndef LASERSCANWIDGET_H
#define LASERSCANWIDGET_H

#include <vector>
#include <QPainter>
#include <QWidget>

#include "laserscan.h"

class QPaintEvent;

class LaserScanWidget : public QWidget
{
public:
  LaserScanWidget(QWidget *parent = 0);

  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;

  void updateLaserScan(robo::LaserScan &new_scan);

protected:
  robo::LaserScan laser_scan_;
  double max_laser_depth_{ 1. };

  void paintEvent(QPaintEvent *event) override;
  void paintRobot(QPainter &painter);
  void paintLaserScan(QPainter &painter);

  double determineScaleFactor();
};

#endif /* LASERSCANWIDGET_H */
