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

protected:
  void paintEvent(QPaintEvent *event) override;
};

#endif /* LASERSCANWIDGET_H */
