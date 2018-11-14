#include "laserscanwidget.h"

#include <QPainter>
#include <QPalette>

LaserScanWidget::LaserScanWidget(QWidget *parent) :
  QWidget(parent)
{
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

void LaserScanWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setPen(Qt::red);
  painter.setBrush(Qt::red);
  painter.setRenderHint(QPainter::Antialiasing, true);

  painter.save();
  painter.translate(width() / 2., height() / 2.);
  painter.drawEllipse(0, 0, 5, 5);
  painter.restore();
}
