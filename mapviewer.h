#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QScrollArea>
#include <QImage>
#include <QLabel>
#include <QString>

#include <memory>

class MapViewer : public QScrollArea
{
  Q_OBJECT

public:
  MapViewer();
  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;
  void setImage(const QImage& map_image);
  bool saveMap(const QString& file_name);

private:

  QImage map;
  std::unique_ptr<QLabel> mapLabel;
};

#endif
