#ifndef OCCUPIED_SPACE_COST_FUNCTION_2D_H
#define OCCUPIED_SPACE_COST_FUNCTION_2D_H

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "grid2d.h"

// Creates a cost function for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
ceres::CostFunction *CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const Eigen::MatrixXd &point_cloud,
    const robo::Grid2D &grid);

#endif /* OCCUPIED_SPACE_COST_FUNCTION_2D_H */
