#include "FSQPSolver.h"

namespace ocra
{
  FSQPSolver::FSQPSolver()
    :NamedInstance("FSQP Solver")
    , nf(0)
    , nfsr(0)
    , ncsrl(0)
    , ncsrn(0)
    , mode(100)
    , iprint(0)
    , miter(100)
    , bigbnd(1e10)
    , eps(1e-8)
    , epseqn(1e-8)
    , udelta(1e-8)
    , bl(0x0,0)
    , bu(0x0,0)
    , f(new double,1)
    , g(0x0,0)
    , lambda(0x0,0)
    , mesh_pts(new int, 1) 
    , _buffer(100)
    , _allObjectivesProvideAGradient(true)
  {
  }

  void FSQPSolver::addObjective(GenericObjective& obj)
  {
    ocra_assert(obj.getDimension() == 1);
    internalAddObjective(obj);
    _objectives.push_back(&obj);
    nf = 1;
    _allObjectivesProvideAGradient = _allObjectivesProvideAGradient && obj.canCompute<PARTIAL_X>();
  }

  void FSQPSolver::removeObjective(Function& obj)
  {
    for(size_t i = 0; i < _objectives.size(); ++i)
    {
      if(&_objectives[i]->getFunction() == &obj)
      {
        removeObjective(*_objectives[i]);
        break;
      }
    }
  }

  void FSQPSolver::removeObjective(GenericObjective& obj)
  {
    internalRemoveObjective(obj);
    _objectives.erase(std::find(_objectives.begin(), _objectives.end(), &obj));
    nf = std::min(1, (int)_objectives.size());
    _allObjectivesProvideAGradient = true;
    for (size_t i=0; i<_objectives.size(); ++i)
      _allObjectivesProvideAGradient = _allObjectivesProvideAGradient && obj.canCompute<PARTIAL_X>();
  }


  void FSQPSolver::addConstraint(LinearConstraint& constraint)
  {
    internalAddConstraint(constraint);
    _constraints.addConstraint(constraint);
  }

  void FSQPSolver::addConstraint(GenericConstraint& constraint)
  {
    internalAddConstraint(constraint);
    _constraints.addConstraint(constraint);
  }

  void FSQPSolver::removeConstraint(LinearConstraint& constraint)
  {
    internalRemoveConstraint(constraint);
    _constraints.removeConstraint(constraint);
  }

  void FSQPSolver::removeConstraint(GenericConstraint& constraint)
  {
    internalRemoveConstraint(constraint);
    _constraints.removeConstraint(constraint);
  }


  void FSQPSolver::addBounds(BoundConstraint& constraint)
  {
    addBounds_(constraint);
  }

  void FSQPSolver::addBounds(IdentityConstraint& constraint)
  {
    addBounds_(constraint);
  }

  void FSQPSolver::addBounds_(DiagonalLinearConstraint& constraint)
  {
    ocra_assert(constraint.isInequality() && "How queer! A bound constraint that is no inequality...");
    internalAddConstraint(constraint);
    _bounds.push_back(&constraint);
  }

  void FSQPSolver::removeBounds(BoundConstraint& constraint)
  {
    removeBounds_(constraint);
  }

  void FSQPSolver::removeBounds(IdentityConstraint& constraint)
  {
    removeBounds_(constraint);
  }

  void FSQPSolver::removeBounds_(DiagonalLinearConstraint& constraint)
  {
    ocra_assert(constraint.isInequality() && "How queer! A bound constraint that is no inequality...");
    internalRemoveConstraint(constraint);
    _bounds.erase(std::find(_bounds.begin(), _bounds.end(), &constraint));
  }

  void FSQPSolver::clearObjectives()
  {
    ocra_assert(false && "non implemented yet");
  }

  void FSQPSolver::clearConstraints()
  {
    clearEqualityConstraints();
    clearInequalityConstraints();
  }

  void FSQPSolver::clearEqualityConstraints()
  {
    clearNonLinearEqualityConstraints();
    clearLinearEqualityConstraints();
  }

  void FSQPSolver::clearNonLinearEqualityConstraints()
  {
    ocra_assert(false && "non implemented yet");
  }

  void FSQPSolver::clearLinearEqualityConstraints()
  {
    ocra_assert(false && "non implemented yet");
  }

  void FSQPSolver::clearInequalityConstraints()
  {
    clearNonLinearInequalityConstraints();
    clearLinearInequalityConstraints();
  }

  void FSQPSolver::clearNonLinearInequalityConstraints()
  {
    ocra_assert(false && "non implemented yet");
  }

  void FSQPSolver::clearLinearInequalityConstraints()
  {
    ocra_assert(false && "non implemented yet");
  }

  void FSQPSolver::clearBounds()
  {
    ocra_assert(false && "non implemented yet");
  }

  //Todo : this should be moved to Solver (or remove: same can be achieved by ref.setValue(x0) - only the checks won't be done)
  void FSQPSolver::set_x0(const VectorXd& x0, const Variable& ref)
  {
    ocra_assert(x0.size()==ref.getSize());
    const Variable& pbVar = getProblemVariable(); //this line force the evaluation of the problem variable
    if (pbVar.getSize() != ref.getSize()) 
      throw std::runtime_error("[FSQPSolver::set_x0] problem variable and reference variable don't have the same size");
    std::vector<int> mapping;
    pbVar.getRelativeMappingOf(ref, mapping);
    if (mapping.size()<pbVar.getSize()) 
      throw std::runtime_error("[FSQPSolver::set_x0] problem variable and reference variable don't have the same children");

    this->x0.resize(x0.size());
    for (int i=0; i<pbVar.getSize(); ++i)
      this->x0[mapping[i]] = x0[i];
  }


  void FSQPSolver::setA(eFsqpProblemType type)
  {
    mode = (mode/10)*10+type;
  }

  void FSQPSolver::setB(eFsqpAlgo type)
  {
    mode -= (mode/10)%10;
    mode += 10*type;
  }

  void FSQPSolver::setC(eFsqpEvaluationDomainPolicy type)
  {
    mode -= (mode/100)%100;
    mode += 100*type;
  }

  void FSQPSolver::setMode(int m)
  {
    ocra_assert(m==100 || m==101 || m==110 || m==111 || m==200 || m==201 || m==210 || m==211);
    mode = m;
  }

  void FSQPSolver::setPrintMode(eFsqpPrintOption m)
  {
    if (m<2)
      iprint = m;
    else
      iprint = (iprint/10)*10+m;
  }

  void FSQPSolver::setPrintStep(int N)
  {
    int assert_cond = iprint%10>=2;
    ocra_assert(assert_cond);
    iprint = 10*N+(iprint%10);
  }

  void FSQPSolver::setPrintOption(int option)
  {
    int assert_cond = option>=0 && option%10< 4;
    ocra_assert(assert_cond);
    iprint = option;
  }

  void FSQPSolver::setMaxIter(int n)
  {
    ocra_assert(n>0);
    miter = n;
  }

  void FSQPSolver::setInfinity(double infinity)
  {
    ocra_assert(infinity>0);
    bigbnd = infinity;
  }

  void FSQPSolver::setEps(double eps)
  {
    ocra_assert(eps>0);
    this->eps = eps;
  }

  void FSQPSolver::setEqnViol(double eps)
  {
    ocra_assert(eps>0);
    epseqn = eps;
  }

  void FSQPSolver::setUDelta(double udelta)
  {
    ocra_assert(udelta>0);
    this->udelta = udelta;
  }

  FSQPSolver::eFsqpProblemType FSQPSolver::getA() const
  {
    return static_cast<eFsqpProblemType>(mode%10);
  }

  FSQPSolver::eFsqpAlgo FSQPSolver::getB() const
  {
    return static_cast<eFsqpAlgo>((mode/10)%10);
  }

  FSQPSolver::eFsqpEvaluationDomainPolicy FSQPSolver::getC() const
  {
    return static_cast<eFsqpEvaluationDomainPolicy>((mode/100)%10);
  }

  int FSQPSolver::getMode() const
  {
    return mode;
  }

  FSQPSolver::eFsqpPrintOption FSQPSolver::getPrintMode() const
  {
    return static_cast<eFsqpPrintOption>(iprint%10);
  }

  int FSQPSolver::getPrintStep() const
  {
    return iprint/10;
  }

  int FSQPSolver::getPrintOption() const
  {
    return iprint;
  }

  int FSQPSolver::getMaxIter() const
  {
    return miter;
  }

  double FSQPSolver::getInfinity() const
  {
    return bigbnd;
  }

  double FSQPSolver::setEps() const
  {
    return eps;
  }

  double FSQPSolver::setEqnViol() const
  {
    return epseqn;
  }

  double FSQPSolver::setUDelta() const
  {
    return udelta;
  }

  void FSQPSolver::printValuesAtSolution()
  {
    ocra_assert(false && "not implemented yet");
  }

  std::string FSQPSolver::toString() const
  {
    return "";
  }


  void FSQPSolver::obj(int nparam, int j, double* x, double* fj, void* cd)
  {
    ocra_assert(j==1 && "in this implementation we only consider a unique objective function (possibly the sum of several functions)");
    checkNewX(nparam, x);
    *fj = 0;
    for (size_t i=0; i<_objectives.size(); ++i)
      *fj += _objectives[i]->getWeight() * _objectives[i]->getValue(0);
  }

  void FSQPSolver::constr(int nparam, int j, double* x, double* gj, void* cd)
  {
    checkNewX(nparam, x);
    *gj = _constraints.getValue(j-1);
  }

  void FSQPSolver::gradob(int nparam, int j, double* x, double* gradfj, void* cd)
  {
    ocra_assert(j==1 && "in this implementation we only consider a unique objective function (possibly the sum of several functions)");
    checkNewX(nparam, x);
    MatrixMap g(gradfj, 1, nparam);
    g.setZero();
    if (_allObjectivesProvideAGradient)
    {
      for (size_t i=0; i<_objectives.size(); ++i)
        utils::addCompressedByCol(_objectives[i]->getJacobian(0), g, findMapping(_objectives[i]->getVariable()), _objectives[i]->getWeight());
    }
    else
      OFSQPProblem::gradob(nparam, j, x, gradfj, cd);
  }

  void FSQPSolver::gradcn(int nparam, int j, double* x, double* gradgj, void* cd)
  {
    MatrixMap g(gradgj, 1, nparam);
    g.setZero();
    checkNewX(nparam, x);
    std::pair<GenericConstraint*, int> p = _constraints[j-1];
    if (p.first->canCompute<PARTIAL_X>())
      utils::uncompressByCol(_constraints.getGradient(p), g, findMapping(p.first->getVariable()));
    else
      OFSQPProblem::gradcn(nparam, j, x, gradgj, cd);
  }



  void FSQPSolver::doPrepare()
  {
    resize();
    updateBounds();
    if (_result.solution.size() == x0.size())
      _result.solution = x0;
    else
      _result.solution.setZero();
  }

  void FSQPSolver::doSolve()
  {
    solver.cfsqp(n(), 
                 nf, 
                 nfsr, 
                 nineqn, 
                 nineq, 
                 neqn, 
                 neq,
                 ncsrl,
                 ncsrn,
                 const_cast<int*>(mesh_pts.data()), //TODO .data should have a non const version. Check with Eigen developpers and correct
                 mode,
                 iprint,
                 miter,
                 &inform,
                 bigbnd,
                 eps,
                 epseqn,
                 udelta,
                 const_cast<double*>(bl.data()),
                 const_cast<double*>(bu.data()),
                 _result.solution.data(),
                 const_cast<double*>(f.data()),
                 const_cast<double*>(g.data()),
                 const_cast<double*>(lambda.data()),
                 *this,
                 0x0);
  }

  void FSQPSolver::doConclude()
  {
    _result.info = translateReturnInfo();
    x0 = _result.solution;
  }

  void FSQPSolver::onConstraintResize(int timestamp)
  {
    _constraints.invalidateMapping();
  }

  void FSQPSolver::onObjectiveResize(int timestamp)
  {
    //do nothing
  }

  void FSQPSolver::resize()
  {
    nparam = n();
    nineqn = _constraints.nineqn();
    nineq = _constraints.nineq();
    neqn = _constraints.neqn();
    neq = _constraints.neq();
    const int sg = std::max(1, nineq+neq);
    const int sl = nparam+1+sg;

    _buffer.resize(2*nparam+sg+sl);
    new (&g) VectorMap(_buffer.allocate(sg), sg);
    new (&lambda) VectorMap(_buffer.allocate(sl), sl); 
    new (&bl) VectorMap(_buffer.allocate(nparam), nparam);
    new (&bu) VectorMap(_buffer.allocate(nparam), nparam);

    bl.setConstant(-bigbnd);
    bu.setConstant(+bigbnd);
  }

  void FSQPSolver::updateBounds()
  {
    for (size_t i=0; i<_bounds.size(); ++i)
    {
      DiagonalLinearConstraint* cstr = _bounds[i];
      const std::vector<int>& mapping = findMapping(cstr->getVariable());
      
      utils::intersectBounds(*cstr, mapping, bl, bu);
    }
  }

  eReturnInfo FSQPSolver::translateReturnInfo() const
  {
    switch (inform)
    {
      case 0: return RETURN_SUCCESS;
      case 1: return RETURN_INFEASIBLE_PROBLEM;
      case 2: return RETURN_INFEASIBLE_PROBLEM;
      case 3: return RETURN_MAX_ITER_REACHED;
      case 4: return RETURN_NUMERICAL_ERROR;
      case 5: return RETURN_NUMERICAL_ERROR;
      case 6: return RETURN_NUMERICAL_ERROR;
      case 7: return RETURN_INCONSISTENT_PROBLEM;
      case 8: return RETURN_NUMERICAL_ERROR;
      case 9: return RETURN_NUMERICAL_ERROR;
      default: ocra_assert(false && "this should never happen");
    }
  }

  void FSQPSolver::checkNewX(int nparam, double* x)
  {
    if (isXNew())
    {
      validateX();
      setVariableValue(Map<VectorXd>(x, nparam));
    }
  }




  void testFSQPSolver01()
  {
    BaseVariable xy("xy",2);
    BaseVariable z("z",1);
    CompositeVariable T("T", xy, z);

    MatrixXd A1(1,1); A1 << 1;
    VectorXd b1(1); b1 << -3;
    LinearFunction lf1(z, A1, b1);
    LinearConstraint c1(&lf1, true);

    MatrixXd A2(1,2); A2 << 3,1 ;
    VectorXd b2(1); b2 << 0;
    LinearFunction lf2(xy, A2, b2);
    LinearConstraint c2(&lf2, true);

    MatrixXd A3(2,2); A3 << 2,1,-0.5,1 ;
    VectorXd b3(2); b3 << 0, 1;
    LinearFunction lf3(xy, A3, b3);
    LinearConstraint c3(&lf3, false);

    QuadraticFunction objFunc(T, Matrix3d::Identity(), Vector3d::Zero(), 0);
    QuadraticObjective obj(&objFunc);
    
    FSQPSolver solver;
    solver.addConstraint(c1);
    solver.addConstraint(c2);
    solver.addConstraint(c3);
    solver.addObjective(obj);

    std::cout << "sol = " << std::endl << solver.solve().solution << std::endl << std::endl;
    ocra_assert(solver.getLastResult().info == 0);


    solver.removeConstraint(c1);
    IdentityFunction id(z);
    VectorXd lz(1); lz << 1;
    VectorXd uz(1); uz << 2;
    IdentityConstraint bnd1(&id, lz, uz);
    solver.addBounds(bnd1);
    std::cout << "sol = " << std::endl << solver.solve().solution << std::endl << std::endl;
    ocra_assert(solver.getLastResult().info == 0);

    BaseVariable t("t", 2);
    VectorXd ut(2); ut << -4,-1;
    BoundFunction bf(t, ut, BOUND_TYPE_SUPERIOR);
    BoundConstraint bnd2(&bf, false);
    solver.addBounds(bnd2);

    QuadraticFunction objFunc2(t, Matrix2d::Identity(), Vector2d::Constant(2.71828),0);
    QuadraticObjective obj2(&objFunc2);
    solver.addObjective(obj2);
    std::cout << "sol = " << std::endl << solver.solve().solution << std::endl << std::endl;
    ocra_assert(solver.getLastResult().info == 0);

    Vector2d c3l(-1,-1);
    c3.setL(c3l);
    std::cout << "sol = " << std::endl << solver.solve().solution << std::endl << std::endl;
    ocra_assert(solver.getLastResult().info == 0);
  }





  class Pb114G5: public Function
  {
  public:
    Pb114G5(Variable& x, double s, double a)
      : NamedInstance("Pb114G5")
      , AbilitySet(PARTIAL_X)
      , CoupledInputOutputSize(false)
      , Function(x,1, LINEARITY_UNDEFINED, CONVEXITY_UNDEFINED, CONTINUITY_CINF)
      , s(s), a(a)
    {
      assert(x.getSize() == 3);
    }

    typedef Function  functionType_t;

  protected:
    void updateValue() const 
    {
      _value[0] = s*(1.12*x[0] + .13167*x[0]*x[2] - 0.00667*x[0]*x[2]*x[2]) + a*x[1] ;
    }
    void updateJacobian() const 
    {
      _jacobian(0,0) = s*(1.12+.13167*x[2]-0.00667*x[2]*x[2]); 
      _jacobian(0,1) = a;  
      _jacobian(0,2) = s*(.13167*x[0]-2*0.00667*x[0]*x[2]);
    }

    double a;
    double s;
  };

  class Pb114G10: public Function
  {
  public:
    Pb114G10(Variable& x)
      : NamedInstance("Pb114G10")
      , AbilitySet(PARTIAL_X)
      , CoupledInputOutputSize(false)
      , Function(x,1, LINEARITY_UNDEFINED, CONVEXITY_UNDEFINED, CONTINUITY_CINF)
    {
      assert(x.getSize() == 4);
    }

    typedef Function  functionType_t;

  protected:
    void updateValue() const 
    {
      _value[0] = 98000*x[0]/(x[1]*x[3]+1000*x[0])-x[2];
    }
    void updateJacobian() const 
    {
      double v = x[1]*x[3]+1000*x[0];
      double v2=v*v;
      _jacobian(0,0) = 98000*(v-1000*x[0])/v2; 
      _jacobian(0,1) = -98000*x[0]*x[3]/v2;
      _jacobian(0,2) = -1;  
      _jacobian(0,3) = -98000*x[0]*x[1]/v2;
    }
  };

  class Pb114G11: public Function
  {
  public:
    Pb114G11(Variable& x)
      : NamedInstance("Pb114G11")
      , AbilitySet(PARTIAL_X)
      , CoupledInputOutputSize(false)
      , Function(x,1, LINEARITY_UNDEFINED, CONVEXITY_UNDEFINED, CONTINUITY_CINF)
    {
      assert(x.getSize() == 4);
    }

    typedef Function  functionType_t;

  protected:
    void updateValue() const 
    {
      _value[0] = (x[1]+x[2])/x[0]-x[3];
    }
    void updateJacobian() const 
    {
      double x2=x[0]*x[0];
      _jacobian(0,0) = -(x[1]+x[2])/x2; 
      _jacobian(0,1) = _jacobian(0,2) = 1/x[0];  
      _jacobian(0,3) = -1;
    }
  };
}

#include "FunctionHelpers.h"

namespace ocra
{
  void testFSQPSolver02()
  {
    double a=0.99;
    double b=0.9;

    BaseVariable x1("x1",1);
    BaseVariable x2("x2",1);
    BaseVariable x3("x3",1);
    BaseVariable x4("x4",1);
    BaseVariable x5("x5",1);
    BaseVariable x6("x6",1);
    BaseVariable x7("x7",1);
    BaseVariable x8("x8",1);
    BaseVariable x9("x9",1);
    BaseVariable x0("x0",1);
    CompositeVariable X("X");
    X.add(x1).add(x2).add(x3).add(x4).add(x5).add(x6).add(x7).add(x8).add(x9).add(x0);
    CompositeVariable x70("x70", x7, x0);
    CompositeVariable x90("x90", x9, x0);
    CompositeVariable x145("x145"); x145.add(x1).add(x4).add(x5);
    CompositeVariable x148("x148"); x148.add(x1).add(x4).add(x8);
    CompositeVariable x678("x678"); x678.add(x6).add(x7).add(x8);
    CompositeVariable x1258("x1258"); x1258.add(x1).add(x2).add(x5).add(x8);
    CompositeVariable x3469("x3469"); x3469.add(x3).add(x4).add(x6).add(x9);

    MatrixXd Pf = MatrixXd::Zero(10,10); Pf(3,6)=Pf(6,3)=-0.063;
    VectorXd qf(10); qf << 5.04, 0.035, 10, 0, 3.36, 0, 0, 0, 0, 0;
    ObjectivePtr<QuadraticFunction> f(new QuadraticFunction(X, Pf, qf, 0));

    VectorXd b1(1); b1<<35.82;
    GreaterThanZeroConstraintPtr<LinearFunction> g1(new LinearFunction(x90, Vector2d(-b,-.222).transpose(), b1));

    Matrix2d A24; A24(0,0)=3;A24(1,0)=-3; A24(0,1)=-a; A24(1,1)=1/a;
    GreaterThanZeroConstraintPtr<LinearFunction> g24(new LinearFunction(x70, A24, Vector2d(-133,133)));

    VectorXd b3(1); b3<<-35.82;
    GreaterThanZeroConstraintPtr<LinearFunction> g3(new LinearFunction(x90, Vector2d(1/b,0.222).transpose(), b3));

    GreaterThanZeroConstraintPtr<Pb114G5> g5(new Pb114G5(x148,1, -a));

    Matrix3d P6 = Matrix3d::Zero(); P6(2,2)=-0.076;
    GreaterThanZeroConstraintPtr<QuadraticFunction> g6(new QuadraticFunction(x678, P6, Vector3d(.325,-a,1.098), 57.425));

    GreaterThanZeroConstraintPtr<Pb114G5> g7(new Pb114G5(x148,-1, 1/a));

    Matrix3d P8 = Matrix3d::Zero(); P8(2,2)=0.076;
    GreaterThanZeroConstraintPtr<QuadraticFunction> g8(new QuadraticFunction(x678, P8, Vector3d(-.325,1/a,-1.098), -57.425));

    VectorXd b9(1); b9<<0;
    EqualZeroConstraintPtr<LinearFunction> g9(new LinearFunction(x145, Vector3d(-1,1.22,-1).transpose(), b9));

    EqualZeroConstraintPtr<Pb114G10> g10(new Pb114G10(x3469));
    EqualZeroConstraintPtr<Pb114G11> g11(new Pb114G11(x1258));

    VectorXd l(10); l << 1.e-5,1.e-5,1.e-5,1.e-5,1.e-5,85,90,3,1.2,145;
    VectorXd u(10); u << 2000,16000,120,5000,2000,93,95,12,4,162;

    IdentityFunction Id(X);
    Constraint<IdentityFunction> bounds(&Id, l, u);

    FSQPSolver solver;
    solver.setPrintOption(1);
    solver.setB(FSQPSolver::NL);
    solver.setMaxIter(10000);
    solver.addObjective(f);
    solver.addConstraint(g1);
    solver.addConstraint(g24);
    solver.addConstraint(g3);
    solver.addConstraint(g5);
    solver.addConstraint(g6);
    solver.addConstraint(g7);
    solver.addConstraint(g8);
    solver.addConstraint(g9);
    solver.addConstraint(g10);
    solver.addConstraint(g11);
    solver.addBounds(bounds);

    VectorXd x_init(10); x_init << 1745,12000,110,3048,1974,89.2,92.8,8,3.6,145;
    VectorXd sol(10); sol << 1698.096,15818.73,54.10228,3031.226,2000,90.11537,95,10.49336,1.561636,153.53535;
    solver.set_x0(x_init, X);
    solver.solve();
    IOFormat fullFmt(FullPrecision, 0, ", ", "\n", "", "", "", "");
    std::cout << solver.getLastResult().info << std::endl;
    std::cout << solver.getLastResult().solution.transpose().format(fullFmt) << std::endl;
    std::cout << solver.getLastResult().solution.isApprox(sol,1e-5) << std::endl;
    std::cout << (solver.getLastResult().solution-sol).norm()/std::min(solver.getLastResult().solution.norm(), sol.norm()) << std::endl;
  }
}
