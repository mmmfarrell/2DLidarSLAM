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
  //this->setImage();
}

void MapViewer::setImage(const QImage& map_image)
{
  //QImage mapImage(800, 800, QImage::Format_Grayscale8);
  //mapImage.fill(QColor(100, 100, 100).rgb());

  //for (int i = 200; i < 600; i++)
  //{
    //for (int j = 200; j < 600; j++)
    //{
      //mapImage.setPixel(QPoint(i, j), QColor(0, 0, 0).rgb());
    //}
  //}

  mapLabel->setPixmap(QPixmap::fromImage(map_image));
  scaleFactor = 1.0;

  this->setVisible(true);
  this->setWidgetResizable(true);
}
