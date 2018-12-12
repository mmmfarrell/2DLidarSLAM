#include <QLabel>
#include <QImage>
#include <QImageReader>
#include <QImageWriter>
#include <QGuiApplication>
#include <QScreen>
#include <QString>

#include "mapviewer.h"

MapViewer::MapViewer() : mapLabel(new QLabel)
{
  mapLabel->setBackgroundRole(QPalette::Base);
  mapLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  mapLabel->setScaledContents(true);

  this->setBackgroundRole(QPalette::Dark);
  this->setWidget(mapLabel);
  this->setVisible(false);
}

QSize MapViewer::minimumSizeHint() const
{
  return QSize(100, 100);
}

QSize MapViewer::sizeHint() const
{
  return QSize(400, 400);
}

void MapViewer::setImage(const QImage& map_image)
{
  mapLabel->setPixmap(QPixmap::fromImage(map_image));

  this->setVisible(true);
  this->setWidgetResizable(true);
}
