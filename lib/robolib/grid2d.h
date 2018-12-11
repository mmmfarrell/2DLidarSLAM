#ifndef GRID2D_H
#define GRID2D_H

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include <Eigen/Core>

namespace robo
{
class Grid2D
{
public:
  Grid2D();
  void resetMap();
  void getLogOddsMap(Eigen::MatrixXd& map) const;
  void getMapQImage(QImage &map) const;

  void setProbabilityOfWorldPoint(const QPointF &world_point,
                                  double probability);
  void addLogOdds(const QPoint &pixel_point, double value_to_add);
  double getProbabilityWorldPoint(const QPointF &world_point) const;
  double getCorrespondenceCostWorldPoint(const QPointF &world_point) const;
  double getCorrespondenceCost(const int row, const int col) const;

  QRgb getOccupiedColor() const;
  QRgb getUnknownColor() const;
  QRgb getFreeColor() const;

  int getMapPixelRows() const;
  int getMapPixelCols() const;
  double getMapResolution() const;

  QPointF getMapOrigin() const;

  QPoint worldToPixel(const QPointF &world_point) const;
  QPointF pixelToWorld(const QPoint &pixel_point) const;

  double getLogOddsOccupied() const;
  double getLogOddsNull() const;
  double getLogOddsFree() const;

  double getMinProbability() const;
  double getMaxProbability() const;
  double getMinCorrespondence() const;
  double getMaxCorrespondence() const;

protected:
  Eigen::MatrixXd map_log_odds_;

  const unsigned int map_width_meters_{ 80 };
  const unsigned int map_height_meters_{ 80 };
  const double map_resolution_meters_{ 1.0 };
  const int map_width_pixels_{ static_cast<int>(map_width_meters_ /
                                                map_resolution_meters_) };
  const int map_height_pixels_{ static_cast<int>(map_height_meters_ /
                                                 map_resolution_meters_) };

  const QRgb occupied_color_{ qRgb(0, 0, 0) };
  const QRgb unknown_color_{ qRgb(127, 127, 127) };
  const QRgb free_color_{ qRgb(255, 255, 255) };

  const double log_odds_null_{ 0. };
  const double log_odds_occupied_{ 0.2 };
  const double log_odds_free_{ -0.2 };

  const float map_origin_x_{ 70. };
  const float map_origin_y_{ 45. };
  const QPointF map_origin_{ map_origin_x_, map_origin_y_ };

  const double min_probability_{ 0.1 };
  const double max_probability_{ 1.0 - min_probability_ };
  const double min_correspondence_cost_{ 1.0 - max_probability_ };
  const double max_correspondence_cost_{ 1.0 - min_probability_ };

  double logOddsToProbability(double log_odds) const;
  double probabilityToLogOdds(double probability) const;
};
}  // namespace robo

#endif /* GRID2D_H */
