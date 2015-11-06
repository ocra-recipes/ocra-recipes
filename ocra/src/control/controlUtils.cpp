#include "controlUtils.h"
#include "ocra/control/Model.h"

namespace ocra
{
  namespace utils
  {
    Vector6d cartesianPCoupling(const Vector6d& kp, const Eigen::Displacementd& Hdes, const Eigen::Displacementd& H)
    {
      Vector6d ret;
      Eigen::Displacementd delta = H.inverse()*Hdes;
      ret.tail<3>() = delta.getTranslation();
      ret.head<3>() = delta.getRotation().log();
      ret.array() *= kp.array();
      return ret;
    }

    Vector6d cartesianDCoupling(const Vector6d& kv, const Eigen::Twistd& Tdes, const Eigen::Twistd& T)
    {
      Eigen::Twistd deltav = Tdes-T;
      return kv.asDiagonal() * deltav.get();
    }

    Vector6d cartesianPDCoupling(const Vector6d& kp, const Eigen::Displacementd& Hdes, const Eigen::Displacementd& H,
      const Vector6d& kv, const Eigen::Twistd& Tdes, const Eigen::Twistd& T)
    {
      return cartesianPCoupling(kp, Hdes, H) + cartesianDCoupling(kv, Tdes, T);
    }

    VectorXd adaptKToInertia(const VectorXd& K, const Model& model, double scale)
    {
      VectorXd k(model.nbInternalDofs());
      const MatrixXd& inertiaMatrix = model.getInertiaMatrix();

      k = inertiaMatrix.diagonal().tail(model.nbInternalDofs()).asDiagonal() * K;
      k *= scale;

      return k;
    }

    Eigen::Displacementd compute_H_2_in_1(const Eigen::Displacementd& H_1_in_ref, const Eigen::Displacementd& H_2_in_ref)
    {
      return H_1_in_ref.inverse() * H_2_in_ref;
    }

    Eigen::Displacementd computeEgoFrame(Model& model, const Vector3d& fwdDirection_in_phantom)
    {
      Matrix3d R;
      R.col(0) = model.getFreeFlyerPosition().getRotation() * fwdDirection_in_phantom;
      R.col(0).z() = 0.;
      R.col(0).normalize();
      R.col(2) = Vector3d::UnitZ();
      R.col(1) = R.col(2).cross(R.col(0));
      return Eigen::Displacementd(model.getCoMPosition(), Quaterniond(R));
    }

    Eigen::Displacementd interpolate(const Eigen::Displacementd& Hstart, const Eigen::Displacementd& Hend, double t)
    {
      SE3CubicInterpolator<double>::StdVectorDisplacement displ;
      SE3CubicInterpolator<double>::StdVectorTwist twist;
      std::vector<double> times;
      SE3CubicInterpolator<double> spline;

      displ.push_back(Hstart);
      displ.push_back(Hend);
      Eigen::Twistd td(0,0,0,0,0,0);
      twist.push_back(td);
      twist.push_back(td);
      times.push_back(0);
      times.push_back(1);
      spline.setControlPoint(displ, twist, times);

      Eigen::Displacementd H;
      Eigen::Twistd T;

      spline.Interpolate(H, T, t);
      return H;
    }

    Eigen::Displacementd projectRotation(const Eigen::Displacementd& H, const Vector3d& n, double& theta)
    {
      Eigen::Displacementd result = H;
      
      const Matrix3d& R = H.getRotation().adjoint();
      //tr(R^t*[n])
      double tr1 = (R(2,1)-R(1,2))*n[0] + (R(0,2)-R(2,0))*n[1] + (R(1,0)-R(0,1))*n[2];
      //tr(R^t*[n]^2) = tr(R^t*n*n^t) - tr(R^t) since n^2 = 1
      double tr2 =  n[0] * (R(0,0)*n[0]+R(1,0)*n[1]+R(2,0)*n[2])
        + n[1] * (R(0,1)*n[0]+R(1,1)*n[1]+R(2,1)*n[2])
        + n[2] * (R(0,2)*n[0]+R(1,2)*n[1]+R(2,2)*n[2])
        - R(0,0) - R(1,1) - R(2,2);

      double alpha = atan2(tr1, tr2);
      theta = M_PI - alpha;
      double s = sin(theta);
      double c = cos(theta);
      double ic = 1-c;
      Matrix3d Rp;
      Rp(0,0) =          + ic*n[0]*n[0] + c;
      Rp(0,1) = -s*n[2] + ic*n[0]*n[1];
      Rp(0,2) =  s*n[1] + ic*n[0]*n[2];
      Rp(1,0) =  s*n[2] + ic*n[1]*n[0];
      Rp(1,1) =          + ic*n[1]*n[1] + c;
      Rp(1,2) = -s*n[0] + ic*n[1]*n[2];
      Rp(2,0) = -s*n[1] + ic*n[2]*n[0];
      Rp(2,1) =  s*n[0] + ic*n[2]*n[1];
      Rp(2,2) =          + ic*n[2]*n[2] + c;

      result.getRotation() = Eigen::Displacementd::Rotation3D(Quaterniond(Rp));
      return result;
    }
  }
}

// cmake:sourcegroup=Api
