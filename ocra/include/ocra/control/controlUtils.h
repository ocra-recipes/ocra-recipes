/*!
\file controlUtils.h
\brief Control utility functions.

Copyright (C) 2010 CEA/DRT/LIST/DTSI/SRCI

\author Escande Adrien
\author Evrard Paul
\date 2010/10/03

File history:
*/

#ifndef _OCRACONTROL_UTILS_H_
#define _OCRACONTROL_UTILS_H_

#include "ocra/MathTypes.h"
#include <Eigen/Lgsm>

namespace ocra
{
  class Model;

  namespace utils
  {
    /** Compute kp*\Delta H + kv * \Delta T where \Delta H = (\delta p \delta r)^T and \Delta T = Tdes-T
      * where \delta p = pdes - p and \delta r = log(Rdes^T*R)
      */
    Eigen::Matrix<double,6,1> cartesianPDCoupling(const Eigen::Matrix<double,6,1>& kp, const Eigen::Displacementd& Hdes, const Eigen::Displacementd& H,
                                  const Eigen::Matrix<double,6,1>& kv, const Eigen::Twistd& Tdes, const Eigen::Twistd& T);

    /** Compute kp*\Delta H  where \Delta H = (\delta p \delta r)^T
      * where \delta p = pdes - p and \delta r = log(Rdes^T*R)
      */
    Eigen::Matrix<double,6,1> cartesianPCoupling(const Eigen::Matrix<double,6,1>& kp, const Eigen::Displacementd& Hdes, const Eigen::Displacementd& H);

    /** Compute kv * \Delta T where \Delta T = Tdes-T
      * where \delta p = pdes - p and \delta r = log(Rdes^T*R)
      */
    Eigen::Matrix<double,6,1> cartesianDCoupling(const Eigen::Matrix<double,6,1>& kv, const Eigen::Twistd& Tdes, const Eigen::Twistd& T);

    /** Multiply a vector of gains by the diagonal of an inertia matrix and a scale.
      * \return v, where v(i) = K(i) * model.getInertiaMatrix()(i, i) * scale.
      */
    Eigen::VectorXd adaptKToInertia(const Eigen::VectorXd& K, const Model& model, double scale = 1.);

    /** \warning This function is deprecated.
      * This function is useful in scripts where Displacements were not completely bound. In such
      * contexts, you cannot write H1.inverse() * H2, so this function can do it for you.
      * [TODO] Remove this function.
      */
    Eigen::Displacementd compute_H_2_in_1(const Eigen::Displacementd& H_1_in_ref, const Eigen::Displacementd& H_2_in_ref);

    /** Computes the displacement of an ego frame wrt the world frame for a manikin.
      * The ego frame is built as follows:
      * the x direction is defined by the user, by giving its coordinates in the root frame of the robot;
      * this vector is supposed to represent the 'forward' direction of the manikin.
      * the z direction is the world frame z;
      * y = z cross x.
      */
    Eigen::Displacementd computeEgoFrame(Model& model, const Eigen::Vector3d& fwdDirection_in_phantom);

    /** Spline interpolator.
      */
    Eigen::Displacementd interpolate(const Eigen::Displacementd& Hstart, const Eigen::Displacementd& Hend, double t);

    /** project the rotation R: Rp = I + sin(theta)[n] + (1-cos(theta))[n]^2 = sin(theta)[n] + (1-cos(theta))nn^t + cos(theta)*I
      * theta minimize -tr(R^t*Rp)
      */
    Eigen::Displacementd projectRotation(const Eigen::Displacementd& H, const Eigen::Vector3d& n, double& theta);
  }
}


#endif //_OCRACONTROL_UTILS_H_

// cmake:sourcegroup=Api
