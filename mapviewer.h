#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QScrollArea>
#include <QImage>

class QAction;
class QLabel;
class QMenu;
class QScrollBar;

class MapViewer : public QScrollArea
{
  Q_OBJECT

public:
  MapViewer();

private:
  void setImage();

  QImage map;
  QLabel *mapLabel;
  double scaleFactor;
};

#endif
