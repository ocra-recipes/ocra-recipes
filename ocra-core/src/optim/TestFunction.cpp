#include "ocra/optim/TestFunction.h"
#include "ocra/optim/Constraint.h"
#include <math.h>


using namespace ocra;

/** The first example is a simple function \f$ R^3 \rightarrow R \f$, \f$ f(x,y,z) = 3x^2y+2\frac{\sqrt{z}}{x}\f$
  *
  * We gives an implementation that computes the value, jacbian and hessian of the function.
  */
class Function1: public Function
{
public:
  typedef Function functionType_t;

  Function1(Variable& x)
    :NamedInstance("function1")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false)
    ,Function(x,1, LINEARITY_UNDEFINED, CONVEXITY_UNDEFINED, CONTINUITY_CINF)
  {
    assert(x.getSize() == 3);
  }

protected:
  /** Compute the value of the function: \f$ f(x,y,z) = 3x^2y+2\frac{z}{x}\f$ */
  void updateValue() const
  {
    _value[0] = 3*x[0]*x[0]*x[1] + 2*sqrt(x[2])/x[0];
  }

  /** Compute the jacobian: \f$J(x,y,z) = \begin{array}[l,c,r] 6xy-2\frac{\sqrt{z}}{x^2} & 3x^2 & \frac{1}{x \sqrt{z}} \end{array} \f$*/
  void updateJacobian() const
  {
    _jacobian(0,0) = 6*x[0]*x[1] - 2*sqrt(x[2])/(x[0]*x[0]); 
    _jacobian(0,1) = 3*x[0]*x[0];
    _jacobian(0,2) = 1/(x[0]*sqrt(x[2]));
  }

  /** Compute the Hessian:
    * \f$ H(x,y,z) = \left(\begin{array}[l,c,r] 
    * 6y + 4\frac{\sqrt{z}}{x^3} & 6x & -\frac{1}{x^2 \sqrt{z}} \\
    * 6x & 0 & 0 \\
    * -\frac{1}{x^2 \sqrt{z}} & 0 & -\frac{1}{2xz \sqrt{z}}
    * \end{array}\right)
    */
  void updateHessian() const
  {
    MatrixXd& H = *IFunction<PARTIAL_XX>::_val[0];   //alias on the hessian value. There is one hessian per dimension of the function, so we take the first (and only) one
    H(0,0) = 6*x[1] + 4*sqrt(x[2])/(x[0]*x[0]*x[0]);
    H(0,1) = H(1,0) = 6*x[0];
    H(0,2) = H(2,0) = -1/(x[0]*x[0]*sqrt(x[2]));
    H(1,1) = 0;
    H(1,2) = H(2,1) = 0;
    H(2,2) = -1/(2*x[0]*x[2]*sqrt(x[2]));
  }

  /** as a precondition to resize, we check that the size of the variable didn't change */
  void doUpdateInputSizeBegin()
  {
    if (x.getSize() != 3)
      throw std::runtime_error("[Function1::doResize] Variable x size is different from 3");
  }
};


void examplef1()
{
  //Create the variable and the function
  BaseVariable xyz("xyz",3);
  Function1 f1(xyz);

  //set a value to the variable and get the value, jacobian and hessian of the function
  xyz.setValue(Vector3d(0.1,-0.2,0.45));
  std::cout << "f(0.1,-0.2,0.45) = " << f1.getValue() << std::endl << std::endl;
  std::cout << "df/dx(0.1,-0.2,0.45) = " << std::endl << f1.getJacobian() << std::endl << std::endl;
  std::cout << "d2f/dx2(0.1,-0.2,0.45) = " << std::endl << f1.get<PARTIAL_XX>(0) << std::endl << std::endl << std::endl;

  //change the value and get the results on more time
  xyz.setValue(Vector3d(0.324,0.12,0.06));
  std::cout << "f(0.324,0.12,0.06) = " << f1.getValue() << std::endl << std::endl;
  std::cout << "df/dx(0.324,0.12,0.06) = " << std::endl << f1.getJacobian() << std::endl << std::endl;
  std::cout << "d2f/dx2(0.324,0.12,0.06) = " << std::endl << f1.get<PARTIAL_XX>(0) << std::endl << std::endl;

  //xyz.resize(4); //this will trigger an exception
  
  if (f1.canCompute<PARTIAL_T>())
    std::cout << f1.get<PARTIAL_T>() << std::endl;
  else
    std::cout << "Can't compute partial_t" << std::endl;


  Constraint<Function1> cstr(&f1, true);
  std::cout << cstr.getDimension() << std::endl;
  std::cout << cstr.getValue() << std::endl;
  std::cout << cstr.get<PARTIAL_XX>(0) << std::endl;
  cstr.setB(VectorXd::Random(1));
  std::cout << cstr.isValid<FUN_VALUE>() << std::endl;
  std::cout << cstr.isRespected() << std::endl;
  //cstr.getU(); //trigger an assert
  std::cout << cstr.getType() << std::endl;
}





/** Another real-valued function with some additional properties and abilities:
  * the geometric mean function \f$ f: R^n \rightarrow R \f$ with \f$ f(x) = \left(\prod_{i=1}^n x_i\right)^(1/n) \f$.
  * This function is concave and we consider x as a function of the time. It demonstrates a more generic use of 
  * Function with a variable of generic size.
  */
class Function2: public Function
{
public:
  Function2(Variable& x)
    : NamedInstance("function2")
    , AbilitySet(PARTIAL_X, PARTIAL_T, FUN_DOT, PARTIAL_X_DOT, PARTIAL_XX, FUN_DDOT)
    , CoupledInputOutputSize(false)
    , Function(x,1, LINEARITY_UNDEFINED, CONVEXITY_CONCAVE, CONTINUITY_CINF)
  {
    //initialize the value of constant ability (partial_T)
    setZeroValues();
  }
protected:
  void updateValue() const
  {
    _value[0] = 1;
    for (int i=0; i<x.getSize(); ++i)
      _value[0] *= x[i];
    _value[0] = std::pow(std::abs(_value[0]),1./x.getSize());
//    _value[0] = 2;
  }

  void updateJacobian() const
  {
    double v = getValue(0)/x.getSize();
    for (int i=0; i<x.getSize(); ++i)
      _jacobian(0,i) = v/x[i];
  }

  void updatePartialT() const
  {
    //do nothing, \partial f/ \partial dt is 0, and the value is set when allocating the memory
  }

  /** Set to zero the _values of abilities that are constant and null. */
  void setZeroValues()
  {
     //we re-set to 0 the partial time derivative;
    IFunction<PARTIAL_T>::_val.setZero();
  }

  void updateJdot() const
  {
    ocra_assert(x.hasTimeDerivative());
    MatrixXd& Jdot = IFunction<PARTIAL_X_DOT>::_val;
    Variable& x_dot = x.getTimeDerivative();
    VectorXd tmp = getJacobian()*static_cast<VectorXd>(x_dot);
    double v = getValue(0);
    MatrixXd q = static_cast<VectorXd>(x).array().inverse().transpose();
    Jdot = tmp[0]*q; 
    Jdot.array() -= v*(static_cast<VectorXd>(x_dot)).transpose().array()*q.array()*q.array();
    Jdot *= 1./x.getSize();
  }

  void updateHessian() const
  {
    MatrixXd& H = *IFunction<PARTIAL_XX>::_val[0];
    VectorXd q = static_cast<VectorXd>(x).array().inverse();
    H = (x.getSize()*q.array()*q.array()).matrix().asDiagonal();
    H -= q*q.transpose();    //Can be optimized by taking into account the symmetry of H and qq'
    H *= -getValue(0)/(x.getSize()*x.getSize());
  }

  void doUpdateInputSizeBegin()
  {
    //do nothing: this function supports variable resizing.
  }

  /** The buffer of abilities has been changed, we need to reset the constant ones */
  void doUpdateInputSizeEnd()
  {
    setZeroValues();
  }
};


void examplef2()
{
  int size = 8;
  BaseVariable x("x",size);
  Variable& x_dot = x.getTimeDerivative();
  Variable& x_ddot = x_dot.getTimeDerivative();
  Function2 f2(x);

  VectorXd v(size), dv(size), ddv(size);
  v   << -0.997497,  0.127171, -0.613392,  0.617481, 0.170019, -0.0402539, -0.299417, 0.791925;
  dv  <<  0.64568 ,  0.49321 , -0.651784,  0.717887, 0.421003,  0.0270699, -0.39201 , -0.970031;
  ddv << -0.817194, -0.271096, -0.705374, -0.668203, 0.97705 , -0.108615 , -0.761834, -0.990661;
  x.setValue(v);
  x_dot.setValue(dv);
  x_ddot.setValue(ddv);
  std::cout << "at x=" << static_cast<VectorXd>(x).transpose() << std::endl;
  std::cout << "at dot{x}=" << static_cast<VectorXd>(x_dot).transpose() << std::endl;
  std::cout << "at dot{x}=" << static_cast<VectorXd>(x_ddot).transpose() << std::endl << std::endl;
  /* Return value for f(x)
   *  0.3065426552821116
   */
  std::cout << "f(x) = " << std::endl << f2.getValue() << std::endl << std::endl;

  /* Return value for df/dx(x)
   *  -0.03841398210747897 0.3013095116831978 -0.06246875066884464 0.06205507847247762 0.2253738223978729 -0.9519035897208457 -0.1279748040701228 0.04838568287434284
   */
  std::cout << "df/dx(x) = " << std::endl << f2.getJacobian() << std::endl << std::endl;

  /* Return value for df/dt(x)
   *  0
   */
  std::cout << "df/dt(x) = " << std::endl << f2.get<PARTIAL_T>() << std::endl << std::endl;
  
  /* Return value for dot{f}(x)
   *  0.2814173015622613
   */
  std::cout << "dot{f}(x) = " << std::endl << f2.get<FUN_DOT>() << std::endl << std::endl;
  
  /* Return value for dot{df/dx}(x)
   *  -0.06013081008007012 -0.8919620161985622 0.009030064772053717 -0.01517677697627763 -0.3511718846463573 -1.514017217677467 0.05006476001110882 0.1036875651602921
   */
  std::cout << "dot{df/dx}(x) = " << std::endl << f2.get<PARTIAL_X_DOT>() << std::endl << std::endl;
  
  /* Return value for d2f/dx2(x)
   *  -0.03369657687596465  -0.03775819772931621  0.007828187787638039 -0.00777634901063332  -0.02824241857342339  0.1192865228818791  0.01603699109748235 -0.006063387016996397
   *  -0.03775819772931621  -2.073159939945413   -0.06140231525745318   0.06099570506687612   0.221526352704108   -0.9356531655417172 -0.125790081927211    0.04755966658509294
   *   0.007828187787638039 -0.06140231525745318 -0.0891112972377192   -0.01264588519097038  -0.04592777179965521  0.1939835353495085  0.02607932693736688 -0.009860269386123156
   *  -0.00777634901063332   0.06099570506687612 -0.01264588519097038  -0.0879350031230401    0.0456236350587858  -0.1926989635553251 -0.0259066279104383   0.009794974030444426
   *  -0.02824241857342339   0.221526352704108   -0.04592777179965521   0.0456236350587858   -1.159882687218127   -0.6998508914598116 -0.0940886048545477   0.03557373210813412
   *   0.1192865228818791   -0.9356531655417172   0.1939835353495085   -0.1926989635553251   -0.6998508914598116  -20.6915514026154    0.3973987740011614  -0.1502515373489986
   *   0.01603699109748235  -0.125790081927211    0.02607932693736688  -0.0259066279104383   -0.0940886048545477   0.3973987740011614 -0.3739866258808198  -0.0201999564463369
   *  -0.006063387016996397  0.04755966658509294 -0.009860269386123156  0.009794974030444426  0.03557373210813412 -0.1502515373489986 -0.0201999564463369  -0.05346146732967134
   */
  std::cout << "d2f/dx2(x) = " << std::endl << f2.get<PARTIAL_XX>(0) << std::endl << std::endl;
  
  /* Return value for ddot{f}(x)
   *  -0.4791048960947238
   */
  std::cout << "ddot{f}(x) = " << std::endl << f2.get<FUN_DDOT>() << std::endl << std::endl;

  x.resize(3);
  x.setValue(Vector3d(-0.11,0.231,0.406));
  std::cout << "at x=" << static_cast<VectorXd>(x).transpose() << std::endl;
  std::cout << "f(x) = " << std::endl << f2.getValue() << std::endl << std::endl;
}


/** An example of function with parameters.
  * We consider the function \f$ f_A:R^3 \rightarrow R \f$ with
  * \f$ A = \left(t_0, x_0, \dot{x}_0, \ddot{x}_0, y_0, \dot{y}_0, \ddot{y}_0, z_0, \dot{z}_0, \ddot{z}_0 \right) \f$
  * and \f$ f_A(u_x,u_y,u_z) = \left(\begin{array}{c c c}
  * x_0 & \dot{x}_0 & \ddot{x}_0 \\
  * y_0 & \dot{y}_0 & \ddot{y}_0 \\
  * z_0 & \dot{z}_0 & \ddot{z}_0
  * \end{array}\right) \left(\begin{array}{c} 1 \\ (t-t_0) \\ (t-t_0)^2/2 \end{array}\right) + \frac{(t-t_0)^3}{6}
  * \left(\begin{array}{c} u_x \\ u_y \\ u_z \end{array}\right) \f$
  */
class Function3: public Function
{
private:
  static std::vector<bool> allAbilitiesUpTo(eFunctionAbility a) {return std::vector<bool>(a+1, true);}

public:
  Function3(Variable& x)
    : NamedInstance("function3")
    , AbilitySet(allAbilitiesUpTo(PARTIAL_T_DOT))
    , CoupledInputOutputSize(false)
    , Function(x,3, LINEARITY_UNDEFINED, CONVEXITY_UNDEFINED, CONTINUITY_CINF
    , true    //time dependant
    , false)  //not time separable
    , t(0), _t0(0), _stateX(Vector3d::Zero()), _stateY(Vector3d::Zero()), _stateZ(Vector3d::Zero())
  {
    ocra_assert(x.getSize() == 3);
    IFunction<PARTIAL_XX>::_val[0]->setZero();
    IFunction<PARTIAL_XX>::_val[1]->setZero();
    IFunction<PARTIAL_XX>::_val[2]->setZero();
  }

  void changeInitialState(double t0, const Vector3d& stateX, const Vector3d& stateY, const Vector3d& stateZ)
  {
    _t0 = t0;
    _stateX = stateX;
    _stateY = stateY;
    _stateZ = stateZ;
    invalidateAll();
  }

  void setTime(double t)
  {
    this->t = t;
  }

protected:
  void updateValue() const
  {
    double dt = t-_t0;
    Vector3d v(1,dt,dt*dt/2);
    double f = dt*dt*dt/6;
    _value[0] = v.dot(_stateX) + x[0]*f;
    _value[1] = v.dot(_stateY) + x[1]*f;
    _value[2] = v.dot(_stateZ) + x[2]*f;
  }

  void updateJacobian() const
  {
    double dt = t-_t0;
    double f = dt*dt*dt/6;
    _jacobian.setIdentity();
    _jacobian.diagonal() *= f;
  }

  void updatePartialT() const
  {
    double dt = t-_t0;
    double f = dt*dt/2;
    IFunction<PARTIAL_T>::_val[0] = _stateX[1] + _stateX[2]*dt + x[0]*f;
    IFunction<PARTIAL_T>::_val[1] = _stateY[1] + _stateY[2]*dt + x[1]*f;
    IFunction<PARTIAL_T>::_val[2] = _stateZ[1] + _stateZ[2]*dt + x[2]*f;
  }

  void updateJdot() const
  {
    double dt = t-_t0;
    double f = dt*dt/2;
    IFunction<PARTIAL_X_DOT>::_val.setIdentity();
    IFunction<PARTIAL_X_DOT>::_val.diagonal() *= f;
  }

  void updateHessian() const
  {
    //do nothing, it is always 0
  }

  void updatePartialTT() const
  {
    double dt = t-_t0;
    IFunction<PARTIAL_TT>::_val[0] =_stateX[2] + x[0]*dt;
    IFunction<PARTIAL_TT>::_val[1] =_stateY[2] + x[1]*dt;
    IFunction<PARTIAL_TT>::_val[2] =_stateZ[2] + x[2]*dt;
  }

  void updatePartialTX() const
  {
    IFunction<PARTIAL_TX>::_val = get<PARTIAL_X_DOT>();
  }

  void updatePartialXT() const
  {
    IFunction<PARTIAL_XT>::_val = get<PARTIAL_X_DOT>();
  }

  void updatePartialTdot() const
  {
    ocra_assert(x.hasTimeDerivative());
    Variable& x_dot = x.getTimeDerivative();
    double dt = t-_t0;
    double f = dt*dt/2;
    IFunction<PARTIAL_T_DOT>::_val = get<PARTIAL_TT>() + x_dot.getValue()*f;
  }

  void doUpdateInputSizeBegin()
  {
    if (x.getSize() != 3)
      throw std::runtime_error("[Function3::doResize] Variable x size is different from 3");
  }



private:
  double    t;
  double    _t0;
  Vector3d  _stateX;
  Vector3d  _stateY;
  Vector3d  _stateZ;
  
};



void examplef3()
{
  BaseVariable xyz("xyz",3);
  Variable& xyz_dot = xyz.getTimeDerivative();
  Variable& xyz_ddot = xyz_dot.getTimeDerivative();
  Function3 f3(xyz);
  f3.changeInitialState(0,Vector3d(0.1,0,0),Vector3d(1.05,0,0),Vector3d(0.,0.3,0));

  xyz.setValue(Vector3d(0.1,-0.2,0.45));
  xyz_dot.setValue(Vector3d(0.03,0.15,-0.307));
  xyz_ddot.setValue(Vector3d(-0.008,-0.024,-0.022));
  f3.setTime(0.1);


  std::cout << "f = " << f3.getValue() << std::endl << std::endl;
  std::cout << "df/dx = " << std::endl << f3.getJacobian() << std::endl << std::endl;
  std::cout << "df/dt = " << std::endl << f3.get<PARTIAL_T    >() << std::endl << std::endl;
  std::cout << "fdot = " << std::endl << f3.get<FUN_DOT          >() << std::endl << std::endl;
  std::cout << "dot{df/dx} = " << std::endl << f3.get<PARTIAL_X_DOT>() << std::endl << std::endl;
  std::cout << "d2f/dx2 = " << std::endl << f3.get<PARTIAL_XX   >(0) << std::endl << std::endl
    << f3.get<PARTIAL_XX   >(1) << std::endl << std::endl
    << f3.get<PARTIAL_XX   >(2) << std::endl << std::endl;
  std::cout << "fddot = " << std::endl << f3.get<FUN_DDOT         >() << std::endl << std::endl;
  std::cout << "d2f/dt2 = " << std::endl << f3.get<PARTIAL_TT   >() << std::endl << std::endl;
  std::cout << "d2f/dtdx = " << std::endl << f3.get<PARTIAL_TX   >() << std::endl << std::endl;
  std::cout << "d2f/dxdt = " << std::endl << f3.get<PARTIAL_XT   >() << std::endl << std::endl;
  std::cout << "dot{df/dt} = " << std::endl << f3.get<PARTIAL_T_DOT>() << std::endl << std::endl;

}


void exampleFunction()
{
  std::cout << "**********************************************" << std::endl;
  std::cout << "************     Example 1     ***************" << std::endl;
  std::cout << "**********************************************" << std::endl;
  examplef1();

  std::cout << std::endl << std::endl;
  std::cout << "**********************************************" << std::endl;
  std::cout << "************     Example 2     ***************" << std::endl;
  std::cout << "**********************************************" << std::endl;
  examplef2();

  std::cout << std::endl << std::endl;
  std::cout << "**********************************************" << std::endl;
  std::cout << "************     Example 3     ***************" << std::endl;
  std::cout << "**********************************************" << std::endl;
  examplef3();
}

// cmake:sourcegroup=Function
