#include "laserscanwidget.h"

#include <limits>
#include <math.h>

#include <QPainter>
#include <QPalette>

LaserScanWidget::LaserScanWidget(QWidget *parent) :
  QWidget(parent)
{
  laser_scan_.resize(number_laser_returns_, 0.);

  this->setBackgroundRole(QPalette::Base);
  this->setAutoFillBackground(true);
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

void LaserScanWidget::updateLaserScan(std::vector<float> new_scan)
{
  laser_scan_ = new_scan;
  this->update();
}

void LaserScanWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  this->paintRobot(painter);
  this->paintLaserScan(painter);
}

void LaserScanWidget::paintRobot(QPainter &painter)
{
  painter.save();
  painter.setPen(Qt::blue);
  painter.setBrush(Qt::blue);
  painter.translate(width() / 2., height() / 2.);
  painter.drawEllipse(0, 0, 5, 5);
  painter.restore();
}

void LaserScanWidget::paintLaserScan(QPainter &painter)
{
  double paint_scale_factor{ this->determineScaleFactor() };

  painter.setPen(Qt::red);
  painter.setBrush(Qt::red);

  for (unsigned int i{ 0 }; i < laser_scan_.size(); i++)
  {
    if (laser_scan_[i] == std::numeric_limits<float>::max())
      continue;

    painter.save();
    painter.translate(width() / 2., height() / 2.);
    float laser_angle{ min_laser_angle_rad_ + i * laser_angle_increment_ };
    float laser_depth{ laser_scan_[i] };
    float laser_return_x{ laser_depth * cosf(laser_angle) };
    float laser_return_y{ laser_depth * sinf(laser_angle) };
    painter.drawEllipse(-paint_scale_factor * laser_return_y,
                        -paint_scale_factor * laser_return_x, 5, 5);
    painter.restore();
  }
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
