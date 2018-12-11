#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>

#include <algorithm>
#include <math.h>

#include "grid2d.h"

namespace robo
{
Grid2D::Grid2D()
{
  this->resetMap();
}

void Grid2D::resetMap()
{
  map_log_odds_.resize(map_width_pixels_, map_height_pixels_);
  map_log_odds_.setZero();
}

void Grid2D::getLogOddsMap(Eigen::MatrixXd& map) const
{
  map = map_log_odds_;
}

void Grid2D::getMapQImage(QImage &map) const
{
  int rows{ static_cast<int>(map_log_odds_.rows()) };
  int cols{ static_cast<int>(map_log_odds_.cols()) };
  map = QImage(rows, cols, QImage::Format_Grayscale8);
  map.fill(unknown_color_);

  for (int row{ 0 }; row < rows; row++)
  {
    for (int col{ 0 }; col < cols; col++)
    {
      double log_odds_map_value{ map_log_odds_(row, col) };
      double probability_occupied{ this->logOddsToProbability(
          log_odds_map_value) };

      int gray_pixel_val{ static_cast<int>(255 - 255 * probability_occupied) };
      map.setPixel(QPoint(row, col),
                   qRgb(gray_pixel_val, gray_pixel_val, gray_pixel_val));
    }
  }
}

void Grid2D::setProbabilityOfWorldPoint(const QPointF &world_point,
                                        double probability)
{
  const QPoint pixel_point{ this->worldToPixel(world_point) };
  const double log_odds_value{ this->probabilityToLogOdds(probability) };

  map_log_odds_(pixel_point.x(), pixel_point.y()) = log_odds_value;
}

void Grid2D::addLogOdds(const QPoint &pixel_point, double value_to_add)
{
  map_log_odds_(pixel_point.x(), pixel_point.y()) += value_to_add;
}

double Grid2D::getProbabilityWorldPoint(const QPointF &world_point) const
{
  const QPoint pixel_point{ this->worldToPixel(world_point) };
  const double log_odds_value{ map_log_odds_(pixel_point.x(),
                                             pixel_point.y()) };
  return this->logOddsToProbability(log_odds_value);
}

double Grid2D::getCorrespondenceCostWorldPoint(const QPointF &world_point) const
{
  const QPoint pixel_point{ this->worldToPixel(world_point) };
  const double log_odds_value{ map_log_odds_(pixel_point.x(),
                                             pixel_point.y()) };
  const double probability_value{ this->logOddsToProbability(log_odds_value) };

  return 1. - probability_value;
}

double Grid2D::getCorrespondenceCost(const int row, const int col) const
{
  const double log_odds_value{ map_log_odds_(row, col) };
  const double probability_value{ this->logOddsToProbability(log_odds_value) };

  return 1. - probability_value;
}

QRgb Grid2D::getOccupiedColor() const
{
  return this->occupied_color_;
}

QRgb Grid2D::getUnknownColor() const
{
  return this->unknown_color_;
}

QRgb Grid2D::getFreeColor() const
{
  return this->free_color_;
}

int Grid2D::getMapPixelRows() const
{
  return static_cast<unsigned int>(this->map_height_pixels_);
}

int Grid2D::getMapPixelCols() const
{
  return static_cast<unsigned int>(this->map_width_pixels_);
}

double Grid2D::getMapResolution() const
{
  return map_resolution_meters_;
}

QPointF Grid2D::getMapOrigin() const
{
  return this->map_origin_;
}

QPoint Grid2D::worldToPixel(const QPointF &world_point) const
{
  double world_x{ world_point.x() };
  double world_y{ world_point.y() };

  double map_x_diff{ map_origin_.x() - world_x };
  int map_x_pix{ static_cast<int>(map_x_diff / map_resolution_meters_) };

  double map_y_diff{ map_origin_.y() - world_y };
  int map_y_pix{ static_cast<int>(map_y_diff / map_resolution_meters_) };

  QPoint pixel_map_point(map_y_pix, map_x_pix);
  return pixel_map_point;
}

QPointF Grid2D::pixelToWorld(const QPoint &pixel_point) const
{
  int px{ pixel_point.x() };
  int py{ pixel_point.y() };

  double world_x{ map_origin_.x() - py * map_resolution_meters_ };
  double world_y{ map_origin_.y() - px * map_resolution_meters_ };

  return QPointF(world_x, world_y);
}

double Grid2D::getLogOddsOccupied() const
{
  return this->log_odds_occupied_;
}

double Grid2D::getLogOddsNull() const
{
  return this->log_odds_null_;
}

double Grid2D::getLogOddsFree() const
{
  return this->log_odds_free_;
}

double Grid2D::getMinProbability() const
{
  return this->min_probability_;
}

double Grid2D::getMaxProbability() const
{
  return this->max_probability_;
}

double Grid2D::getMinCorrespondence() const
{
  return this->min_correspondence_cost_;
}

double Grid2D::getMaxCorrespondence() const
{
  return this->max_correspondence_cost_;
}

double Grid2D::logOddsToProbability(double log_odds) const
{
  return 1. - 1. / (1. + exp(log_odds));
}

double Grid2D::probabilityToLogOdds(double probability) const
{
  return log( probability / (1. - probability));
}
}  // namespace robo
