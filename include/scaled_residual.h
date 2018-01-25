#include "ceres/ceres.h"
#include "ceres/rotation.h"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


    /** -----------------  YOUR CODE HERE -------------------------
     * Implement this class! The code should be very similay to SimpleResidual.
     * You will have to add the weight to the constructor (make sure to store it in the class), and scale the residual approapriately before returning.
     */


/**
 * @brief The ScaledResidual struct transforms two points from the local camera frames to the global frame using the
 * provided camera_rotations and camera_translations and computes the error. In addition to the operations performed by the
 * SimpleResidual struct, ScaledResidual also takes into account a weight associated with each correspondeces and scales the residual appropriately.
 */
//struct ScaledResidual {

//};
