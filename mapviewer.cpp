#include <QLabel>
#include <QImage>
#include <QImageReader>
#include <QImageWriter>
#include <QGuiApplication>
#include <QScreen>
#include <QString>

#include "mapviewer.h"

MapViewer::MapViewer() : mapLabel(new QLabel), scaleFactor(1)
{
  mapLabel->setBackgroundRole(QPalette::Base);
  mapLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  mapLabel->setScaledContents(true);

  this->setBackgroundRole(QPalette::Dark);
  this->setWidget(mapLabel);
  this->setVisible(false);
}

void MapViewer::setImage(const QImage& map_image)
{
  mapLabel->setPixmap(QPixmap::fromImage(map_image));
  scaleFactor = 1.0;

  this->setVisible(true);
  this->setWidgetResizable(true);
}
