#include "ceres/ceres.h"
#include "ceres/rotation.h"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

/**
 * @brief The SimpleResidual struct transforms two points from the local camera frames to the global frame using the
 * provided camera_rotations and camera_translations and computes the error.
 */
struct SimpleResidual {
  SimpleResidual(double* p1, double *p2)
  {
      memcpy(&p1_, p1, 3*sizeof(double));
      memcpy(&p2_, p2, 3*sizeof(double));
  }

  template <typename T> bool operator()(const T* const camera_rotation1,
                                        const T* const camera_translation1,
                                        const T* const camera_rotation2,
                                        const T* const camera_translation2,
                                        T* residuals) const {      

      T point_c1[3] = {T(p1_[0]), T(p1_[1]), T(p1_[2])}; /// point in the local frame of reference of camera 1
      T p_c1[3]; /// point_c1 transformed into the global frame of reference; i.e. p_c1 = camera_rotation1 * point_c1 + camera_translation1

      T point_c2[3] = {T(p2_[0]), T(p2_[1]), T(p2_[2])}; /// point in the local frame of reference of camera 2
      T p_c2[3]; /// point_c2 transformed into the global frame of reference; i.e. p_c2 = camera_rotation2 * point_c2 + camera_translation2

       /** -----------------  YOUR CODE HERE -------------------------
        * Use the Ceres methods to compute p_c1 and p_c2.
        * The residual should store:
        * residual = p_c1 - p_c2 = (camera_rotation1 * point_c1 + camera_translation1) - (camera_rotation2 * point_c2 + camera_translation2)
        */

      residuals[0] = (p_c1[0] - p_c2[0]);
      residuals[1] = (p_c1[1] - p_c2[1]);
      residuals[2] = (p_c1[2] - p_c2[2]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double* p1, double* p2) {
    return (new ceres::AutoDiffCostFunction<
            SimpleResidual, 3, 4, 3, 4, 3>(
                new SimpleResidual(p1,p2)));
  }

 private:
    double p1_[3];
    double p2_[3];
};
