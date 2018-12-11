#include <gtest/gtest.h>

#include <Eigen/Core>

#include "grid2d.h"

TEST(ANewGrid, askForLogOddsMap_MapAllNull)
{
  robo::Grid2D grid;
  Eigen::MatrixXd log_odds_map;
  grid.getLogOddsMap(log_odds_map);

  int true_map_width{ 80 };
  int true_map_height{ 80 };
  Eigen::MatrixXd true_new_map(true_map_width, true_map_height);
  true_new_map.setZero();

  EXPECT_EQ(log_odds_map, true_new_map);
}

class ASparseGrid : public ::testing::Test
{
public:
  robo::Grid2D grid_;
  const double min_probability_{ grid_.getMinProbability() };
  const double max_probability_{ grid_.getMaxProbability() };
  const double min_correspondence_cost_{ grid_.getMinCorrespondence() };
  const double max_correspondence_cost_{ grid_.getMaxCorrespondence() };
  ASparseGrid()
  {
    const QPointF min_prob_world_point{ 5., 5. };
    grid_.setProbabilityOfWorldPoint(min_prob_world_point, min_probability_);

    const QPointF max_prob_world_point{ 2., -2. };
    grid_.setProbabilityOfWorldPoint(max_prob_world_point, max_probability_);
  }
};

TEST_F(ASparseGrid, askForProbabiltyOfWorldPoint_ReturnsMinProbability)
{
  const QPointF world_point{ 5., 5. };
  double probability{ grid_.getProbabilityWorldPoint(world_point) };

  double abs_tolerance{ 1e-7 };
  EXPECT_NEAR(probability, this->min_probability_, abs_tolerance);
}

TEST_F(ASparseGrid, askForProbabiltyOfWorldPoint_ReturnsMaxProbability)
{
  const QPointF world_point{ 2., -2. };
  double probability{ grid_.getProbabilityWorldPoint(world_point) };

  double abs_tolerance{ 1e-7 };
  EXPECT_NEAR(probability, this->max_probability_, abs_tolerance);
}

TEST_F(ASparseGrid, askForCorrespondenceCostOfWorldPoints_ReturnsMaxCost)
{
  const QPointF world_point{ 5., 5. };
  double correspondence_cost{ grid_.getCorrespondenceCostWorldPoint(
      world_point) };

  double abs_tolerance{ 1e-7 };
  EXPECT_NEAR(correspondence_cost, this->max_correspondence_cost_,
              abs_tolerance);
}

TEST_F(ASparseGrid, askForCorrespondenceCostOfWorldPoints_ReturnsMinCost)
{
  const QPointF world_point{ 2., -2. };
  double correspondence_cost{ grid_.getCorrespondenceCostWorldPoint(
      world_point) };

  double abs_tolerance{ 1e-7 };
  EXPECT_NEAR(correspondence_cost, this->min_correspondence_cost_,
              abs_tolerance);
}
