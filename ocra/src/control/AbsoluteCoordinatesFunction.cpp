#if 0

#include "ocra/control/AbsoluteCoordinatesFunction.h"

#include "ocra/optim/utilities.h"

using namespace xde;

#define PATCH_JACOBIAN

namespace ocra
{
  AbsoluteCoordinatesFunction::AbsoluteCoordinatesFunction(Robot* robot, cfl_size_t segment, const Vector3& v, bool isAPoint)
    :Function(*robot->getVariableJointPosition(), 3, L_UNDEFINED, C_UNDEFINED, C_INF,
    false, true),
    _robot(robot), _segment(robot->getSkeleton().getSegment(segment)), _point(v), _isAPoint(isAPoint)
  {
    J_body_scene_p_body_r_body.resize(6, _x->getSize());
  }

/*  Function(Variable& x, cfl_size_t dimension,  int linearity = L_UNDEFINED, 
              int convexity = C_UNDEFINED, int continuity = UNKNOWN, 
              bool useHessian = true, bool useGrad = true);*/



  void AbsoluteCoordinatesFunction::computeValue(void)  const
  {
    const DisplacementBase& H_seg_0 = _robot->getModel().getSegmentPosition(*_segment);
    if (_isAPoint)
      _value = H_seg_0 * _point;
    else
      _value = H_seg_0.getRotation() * _point;

    //std::cout << "[AbsoluteCoordinatesFunction::computeValue] H = " << H_seg_0 << std::endl;
    //std::cout << "[AbsoluteCoordinatesFunction::computeValue] _value = " << _value << std::endl;
  }

  void AbsoluteCoordinatesFunction::computeGradient(void)  const
  {
#ifdef PATCH_JACOBIAN
    //TODO [optimize] on peut eviter certaines multiplication
    //const Matrix& tmp = _robot->getModel().getJacobian(*_segment);
    //J_body_scene_p_body_r_body.copyValuesFrom(tmp);
    J_body_scene_p_body_r_body = _robot->getModel().getJacobian(*_segment);

    //[0, R_b_0]*Ad_H_p_b^-1 * J pour un point
    //on annule les blocs 1..3,1..3 et 4..6,4..6 de l'adjointe pour un vecteur
    Matrix66 ad_H_inv;
    Displacement(_point).compute_Ad_H_inv(ad_H_inv);
    const DisplacementBase& H_body_scene = _robot->getModel().getSegmentPosition(*_segment);
    Matrix selectedRot(3,6);
    SubMatrix R(selectedRot);
    R.rescope(3,3,0,0);
    R.setToZero();
    R.rescope(3,3,0,3);
    //R.copyValuesFrom(H_body_scene.getRotation());
    H_body_scene.getRotation().computeRotationMatrix(R);

    Matrix RAd(3,6);
    if (!_isAPoint)
    {
      SubMatrix subAd(ad_H_inv);
      subAd.rescope(3,3,3,3);
      subAd.setToZero();
    }
    CML_gemm<'n','n'>(1., selectedRot, ad_H_inv, 0, RAd);
    CML_gemm<'n','n'>(1., RAd, J_body_scene_p_body_r_body, 0, _gradient);

#else
    const Matrix& tmp = _robot->getModel().getJacobian(*_segment);
    J_body_scene_p_body_r_body.copyValuesFrom(tmp);

//    std::cout << "Raw Jacobian" << std::endl;
//    std::cout << J_body_scene_p_body_r_body << std::endl;

    Matrix66 ad_H_inv;
    Displacement(_point).compute_Ad_H_inv(ad_H_inv);
    const ocra::Displacement& H_body_scene = _robot->getModel().getSegmentPosition(*_segment);
    Matrix66 Ad_H_scene_body;
    H_body_scene.compute_Ad_H_inv(Ad_H_scene_body);
    Matrix66 Ad_toto = Ad_H_scene_body * ad_H_inv;
    SubMatrix ad_H_inv_trans(Ad_toto);
    SubMatrix sub_J(J_body_scene_p_body_r_body);

/*    //debug
    SubMatrix debug(J_body_scene_p_body_r_body);
    debug.rescope(2,J_body_scene_p_body_r_body.get_ncols(), 1, 0);
//    debug.scalarMultInPlace(-1);
    debug.rescope(2,J_body_scene_p_body_r_body.get_ncols(), 2, 0);
//    debug.scalarMultInPlace(-1);
    debug.rescope(2,J_body_scene_p_body_r_body.get_ncols(), 4, 0);
//    debug.scalarMultInPlace(-1);
    debug.rescope(1,J_body_scene_p_body_r_body.get_ncols(), 5, 0);
//    debug.scalarMultInPlace(-1);*/

    if (_isAPoint)
    {
      ad_H_inv_trans.rescope(3,6,3,0);
//      writeInFile(ad_H_inv, "gJ0.txt", true);
//      writeInFile(sub_J, "gJ0.txt", true);
    }
    else
    {
      ad_H_inv_trans.rescope(3,3,3,0);
      sub_J.rescope(3, J_body_scene_p_body_r_body.get_ncols(), 0, 0);
//      writeInFile(ad_H_inv_trans, "JH.txt");
//      writeInFile(ad_H_inv, "JH.txt", true);
//      writeInFile(sub_J, "JH.txt", true);
    }
    
    CML_gemm<'n','n'>(1., ad_H_inv_trans, sub_J, 0, _gradient);
//    writeInFile(_gradient, "gJ0.txt", true);
    #endif
    static int cpt = 0;
    //if ((++cpt)%100 == 0)
      //checkGradient(1e-6, 1e-6);
    
  }


  void AbsoluteCoordinatesFunction::computeHessian(void)  const
  {
    throw std::runtime_error("[AbsoluteCoordinatesFunction::computeHessian] No hessian computation");
  }

  void AbsoluteCoordinatesFunction::computeJdot(void) const
  {
    throw std::runtime_error("[AbsoluteCoordinatesFunction::computeJdot] Function is not implemented yet");
  }

  void AbsoluteCoordinatesFunction::computeJdotXdot(void) const
  {
    throw std::runtime_error("[AbsoluteCoordinatesFunction::computeJdotXdot] Function is not implemented yet");
  }

  void AbsoluteCoordinatesFunction::doUpdateSize(void)
  {
    J_body_scene_p_body_r_body.resize(6,_x->getSize());
  }


  void AbsoluteCoordinatesFunction::checkGradient(real delta, real threshold) const
  {
    bool error = false;
    int n = (int)_x->getSize();
//    Vector q0(n-6);
    static Vector q;
    q.resize(n-6);
//    q0.copyValuesFrom(_robot->getModel().getJointPositions());
    q.copyValuesFrom(_robot->getModel().getJointPositions());
//    Displacement r0 = _robot->getModel().get_H_root_bm();
//    Displacement r;

    static Vector v0;
    v0.resize(_dimension);
    v0.copyValuesFrom(getValues());
    //static Matrix g0;
    //g0.resize(_dimension, n);
    //g0.copyValuesFrom(_gradient);

    //std::cout << "*********G R A D I E N T    C H E C K*********" << std::endl;
    for (int i=0; i<n-6; ++i)
    {
      q[i] += delta;
      _robot->getModel().setJointPositions(q);
      computeValue();
      for (unsigned int j=0; j<_dimension; ++j)
      {
        real g0 = _gradient(j,i+6);
        real gd = (_value[j]-v0[j])/delta;
        real err = fabs(gd - g0);
        real relErr = fabs(err/g0);
        if (relErr > threshold && err>threshold)
        {
          std::cout << i << ": " << gd << "   " << g0 << "   rel=" << relErr << "   abs=" << err << std::endl;
          //_gradient(j,i+6) = gd;
          //error = true;
        }
      }
      q[i] -= delta;
    }
    _robot->getModel().setJointPositions(q);
    //if (error)
      //std::cout << std::endl;
  }
}

#endif


// cmake:sourcegroup=Functions
