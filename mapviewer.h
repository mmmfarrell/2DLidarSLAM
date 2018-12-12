#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QScrollArea>
#include <QImage>

class QLabel;

class MapViewer : public QScrollArea
{
  Q_OBJECT

public:
  MapViewer();
  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;
  void setImage(const QImage& map_image);

private:

  QImage map;
  QLabel *mapLabel;
  double scaleFactor;
};

#endif
