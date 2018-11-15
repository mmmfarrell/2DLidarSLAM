#include "laserscanwidget.h"

#include <iostream> // TODO remove
#include <QPainter>
#include <QPalette>

LaserScanWidget::LaserScanWidget(QWidget *parent) :
  QWidget(parent)
{
  this->setBackgroundRole(QPalette::Base);
  this->setAutoFillBackground(true);

  laser_scan_ = 0.0; // TODO changeme
}

QSize LaserScanWidget::minimumSizeHint() const
{
  return QSize(100, 100);
}

QSize LaserScanWidget::sizeHint() const
{
  return QSize(400, 200);
}

void LaserScanWidget::setMaxLaserDepth(double max_laser_depth)
{
  this->max_laser_depth_ = max_laser_depth;
}

void LaserScanWidget::updateLaserScan(double new_scan)
{
  laser_scan_ = new_scan;
}

void LaserScanWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  painter.save();
  painter.setPen(Qt::blue);
  painter.setBrush(Qt::blue);
  painter.translate(width() / 2., height() / 2.);
  painter.drawEllipse(0, 0, 5, 5);
  painter.restore();

  double paint_scale_factor{ this->determineScaleFactor() };

  painter.setPen(Qt::red);
  painter.setBrush(Qt::red);
  painter.save();
  painter.translate(width() / 2., height() / 2.);
  painter.drawEllipse(0, -paint_scale_factor * laser_scan_, 5, 5);
  painter.restore();
}

double LaserScanWidget::determineScaleFactor()
{
  int min_dimension{ 0 };
  if (this->width() < this->height())
    min_dimension = this->width();
  else
    min_dimension = this->height();

  double scale_factor{ min_dimension / 2. / max_laser_depth_ };

  return scale_factor;
}
