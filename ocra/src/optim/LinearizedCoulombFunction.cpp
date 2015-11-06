#include "ocra/optim/LinearizedCoulombFunction.h"
#include <math.h>

#include <stdexcept>


namespace ocra
{
  const double LinearizedCoulombFunction::ANGLE_OFFSET = 0;

  LinearizedCoulombFunction::LinearizedCoulombFunction(Variable& f, double frictionCoeff, 
                                                        int numberOfFaces, double margin)
    :NamedInstance("linearized coulomb function")
    ,AbilitySet(PARTIAL_X)
    ,CoupledInputOutputSize(false)
    ,LinearFunction(f,numberOfFaces)
    , _mu(frictionCoeff)
    , _margin(margin)
    , _R_cone(Matrix3d::Identity())
  {
    if (f.getSize() != 3)
    {
      std::stringstream ss;
      ss << "[ocra::LinearizedCoulombFunction::LinearizedCoulombFunction] Size of the variable is not 3 but " << f.getSize();
      throw std::runtime_error(ss.str());
    }

    if (numberOfFaces < 3)
      throw std::runtime_error("[ocra::LinearizedCoulombFunction::LinearizedCoulombFunction] Number of faces is less than 3");
    
    checkCoeff(frictionCoeff);

    buildA();
    buildb();
  }

  
  real LinearizedCoulombFunction::getFrictionCoeff() const
  {
    return _mu;
  }

  real LinearizedCoulombFunction::getMargin() const
  {
    return _margin;
  }

  void LinearizedCoulombFunction::setFrictionCoeff(double coeff)
  {
    checkCoeff(coeff);
    _mu = coeff;
    buildA();
  }

  void LinearizedCoulombFunction::setMargin(double margin)
  {
    _margin = margin;
    buildb();
  }

  void LinearizedCoulombFunction::setConeOrientation(const Matrix3d& R)
  {
    _R_cone = R;
    buildA();
  }

  const Matrix3d& LinearizedCoulombFunction::getConeOrientation() const
  {
    return _R_cone;
  }


  // ------------------------ private methods ---------------------------------
  void LinearizedCoulombFunction::buildA()
  {
    double angle = ANGLE_OFFSET;
    double angleIncr = 2*M_PI/_dim;
    Vector3d v1, v2, n;
    v1[0] = _mu * cos(angle);               //ray of the discreatized cone
    v1[1] = _mu * sin(angle);
    v1[2] = 1;
    for (int i=0; i<_dim; ++i)
    {
      angle += angleIncr;
      v2[0] = _mu * cos(angle);             //ray of the discretized cone
      v2[1] = _mu * sin(angle);
      v2[2] = 1;

      n = v2.cross(v1);                     //normal vector to a face
      n.normalize();

      _jacobian.row(i) = n.transpose();

      v1 = v2;
    }

    _J_cache = _jacobian;
    _jacobian = _J_cache * _R_cone.transpose();
  }

  void LinearizedCoulombFunction::buildb()
  {
    _b.setConstant(_margin);     
  }

  void LinearizedCoulombFunction::checkCoeff(real mu)
  {
    if (mu<=0)
      throw std::runtime_error("[ocra::LinearizedCoulombFunction::] Invalid (negative) input for a friction coefficient");
  }
}


//test
namespace ocra
{
  void testLinearCoulomb()
  {
    BaseVariable f("f", 3);
    LinearizedCoulombFunction coul(f, 1, 4);

    Vector3d v;

    std::cout << coul.getA() << std::endl;
    std::cout << coul.getb() << std::endl;

    bool b=true;
    while (b)
    {
      std::cin >> v[0] >> v[1] >> v[2];
      f.setValue(v);
      std::cout << coul.getValue() << std::endl;
      if (v[0] == -2.71828)
        b=false;
    }
  }
}

// cmake:sourcegroup=Function

