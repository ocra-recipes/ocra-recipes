#include "ocra/optim/QLDSolver.h"


#define SOLVER_VERBOSE 0
#if SOLVER_VERBOSE
# define IF_SOLVER_VERBOSE(x) x
#else
# define IF_SOLVER_VERBOSE(x) ((void)0);
#endif


namespace ocra
{
  QLDSolver::QLDSolver()
    : NamedInstance("QLDSolver")
    , QuadraticSolver(false)
    , _pt(0)
    , _infinity(1.e+10)
    , _C(0x0,0,0)
    , _d(0x0,0)
    , _A(0x0,0,0)
    , _b(0x0,0)
    , _xl(0x0,0)
    , _xu(0x0,0)
    , _buffer(100)
  {
    _solver = new ObjQLD();
    _eps = _solver->getEps();
  }

  void QLDSolver::setTolerance(double epsilon)
  {
    _eps = epsilon;
    _solver->setEps(epsilon);
  }

  double QLDSolver::getTolerance(void) const
  {
    return _solver->getEps();
  }


  const std::string& QLDSolver::getMoreInfo() const
  {
    static const std::string info = ""; 
    return info;
  }

  MatrixXd QLDSolver::getP() const
  {
    return _C;
  }

  VectorXd QLDSolver::getq() const
  {
    return _d;
  }

  MatrixXd QLDSolver::getA() const
  {
    return _A.topRows(_m);
  }

  VectorXd QLDSolver::getb() const
  {
    return _b.head(_m);
  }

  VectorXd QLDSolver::getbp() const
  {
    return VectorXd::Zero(_m);
  }

  MatrixXd QLDSolver::getC() const
  {
    return _A.bottomRows(_p);
  }

  VectorXd QLDSolver::getd() const
  {
    return _b.tail(_p);
  }

  VectorXd QLDSolver::getl() const
  {
    return VectorXd::Zero(_p);
  }

  VectorXd QLDSolver::getu() const
  {
    return VectorXd::Constant(_p, 1e10);
  }

  VectorXd QLDSolver::getxl() const
  {
    return _xl;
  }

  VectorXd QLDSolver::getxu() const
  {
    return _xu;
  }


  void QLDSolver::doPrepare()
  {
    resize();

    updateObjectiveEquations();
    updateEqualityEquations();
    updateInequalityEquations();
    updateBounds();

    IF_SOLVER_VERBOSE(
      //_variable.printTree(std::cout);
      std::cout << "obj" << std::endl;
      std::cout << _C << std::endl << std::endl;
      std::cout << _d << std::endl << std::endl;

      std::cout << "cstr" << std::endl;
      std::cout << _A << std::endl << std::endl;
      std::cout << _b << std::endl << std::endl;

      std::cout << "bnd" << std::endl;
      std::cout << _xl << std::endl << std::endl;
      std::cout << _xu << std::endl << std::endl;
      system("pause");
    )
  }

  void QLDSolver::doSolve()
  {
    Map<VectorXd> x(_result.solution.data(), _result.solution.size());
    info = _solver->solve(_C, _d, _A, _b, static_cast<int>(_m), x, _xl, _xu);
    translateReturnInfo(_result.info, info);
  }

  void QLDSolver::doConclude()
  {
    //do nothing
  }

  void QLDSolver::resize()
  {
    int _n = n();
    if (_invalidatedMP)
      recomputeMP();

    int mp = static_cast<int>(_m+_ps);

    _buffer.resize(_n*(_n+mp+3)+mp);
    new (&_C) MatrixMap(_buffer.allocate(_n*_n), _n, _n);
    new (&_d) VectorMap(_buffer.allocate(_n), _n);
    new (&_A) MatrixMap(_buffer.allocate(_n*mp), mp, _n);
    new (&_b) VectorMap(_buffer.allocate(mp), mp); 
    new (&_xl) VectorMap(_buffer.allocate(_n), _n);
    new (&_xu) VectorMap(_buffer.allocate(_n), _n);

    _C.setZero();
    _d.setZero();
    _A.setZero();
    _b.setZero();
    _xl.setConstant(-_infinity);
    _xu.setConstant(+_infinity);
  }


  void QLDSolver::updateEqualityEquations()
  {
    int m = 0;
    for (size_t i=0; i<_equalityConstraints.size(); ++i)
    {
      LinearConstraint* cstr = _equalityConstraints[i];
      int dim = cstr->getDimension();
      
      // XXX http://eigen.tuxfamily.org/api/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing
      Eigen::Block<MatrixMap> _A_block = _A.block(m, 0, dim, n());
      Eigen::DenseBase<VectorMap>::SegmentReturnType _b_segment = _b.segment(m, dim);
      Eigen::VectorXd v;
      utils::convert(*cstr, findMapping(cstr->getVariable()), CSTR_PLUS_EQUAL, _A_block, _b_segment, v);

      m += dim;
    }
  }

  void QLDSolver::updateInequalityEquations()
  {
    int p = static_cast<int>(_m);
    for (size_t i=0; i<_inequalityConstraints.size(); ++i)
    {
      LinearConstraint* cstr = _inequalityConstraints[i];
      int dim = cstr->getDimension();

      if(cstr->getType() == CSTR_LOWER_AND_GREATER)
        dim *= 2;

      // XXX http://eigen.tuxfamily.org/api/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing
      Eigen::Block<Eigen::Map<Eigen::MatrixXd> > _A_block = _A.block(p, 0, dim, n());
      Eigen::VectorBlock<Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> > > _b_segment = _b.segment(p, dim);
      Eigen::VectorXd v;
      utils::convert(*cstr, findMapping(cstr->getVariable()), CSTR_PLUS_GREATER, _A_block, _b_segment, v);

      p += dim;
    }
  }

  void QLDSolver::updateObjectiveEquations()
  {
    for (size_t i=0; i<_objectives.size(); ++i)
    {
      QuadraticFunction& obj = _objectives[i]->getFunction();
      utils::addCompressed2d(obj.getPi(), _C, findMapping(obj.getVariable()), _objectives[i]->getWeight());
      utils::addCompressedByRow(obj.getqi(), _d, findMapping(obj.getVariable()), _objectives[i]->getWeight());
    }
  }

  void QLDSolver::updateBounds()
  {
    for (size_t i=0; i<_bounds.size(); ++i)
    {
      DiagonalLinearConstraint* cstr = _bounds[i];
      const std::vector<int>& mapping = findMapping(cstr->getVariable());
      
      utils::intersectBounds(*cstr, mapping, _xl, _xu);
    }
  }


  void QLDSolver::translateReturnInfo(eReturnInfo& ocraInfo, int qldInfo)
  {
    switch (qldInfo)
    {
      case 0: ocraInfo = RETURN_SUCCESS;           return;
      case 1: ocraInfo = RETURN_MAX_ITER_REACHED;  return;
      case 2: ocraInfo = RETURN_NUMERICAL_ERROR;   return;
      case 5: ocraInfo = RETURN_MEMORY_ERROR;      return;
      default:
        if (qldInfo >10) 
          {ocraInfo = RETURN_INFEASIBLE_PROBLEM; return;}
        else 
          throw std::runtime_error("[QLDSolver::translateReturnInfo] invalid info number.");
    }
  }

#undef SOLVER_VERBOSE
#undef IF_SOLVER_VERBOSE


  /*
  void QLDSolver::updateInequalityEquations(std::vector<cfl_size_t>& workingMapping)
  {
    SubMatrix sC(_A);
    sC.rescope(_p,_n,_m,0);
    //sC.setToZero(); //TODO [mineur] : be more subtle : elements that will be replaced don't need to be put to 0
    SubVector sd(_b);
    int p = 0;
    _pt = 0;
    for (unsigned int i=0; i<_inequalityConstraints.size(); ++i)
    {
      Constraint<LinearFunction>* cstr = _inequalityConstraints[i];
      cfl_size_t dim = cstr->getDimension();
      sC.rescope(dim, _n, p+_m, 0);
      sC.setToZero();
      sd.rescope(dim, p+_m);

      workingMapping.resize(cstr->getVariable().getSize());
      uncompressMatrixWithScaling(*_variable, cstr->getVariable(), cstr->getGradients(), sC, workingMapping, -1);
      sd.copyValuesFrom(cstr->getFunction()->getb());
      CML_scal(-1, sd);

      if (cstr->isSlacked())
      {
        //we need to put -1. * -I in front of the slack variable
        _variable->getRelativeMappingOf(*cstr->getSlackVariable(), workingMapping);
        for (cfl_size_t j=0; j<dim; ++j)
          sC(j, workingMapping[j]) = 1;
      }

      //std::cout << sC << std::endl;

      int k = transferSimpleInequalityToBounds(sC, sd);
      
      p+=dim-k;
      _pt += k;
    }
  }


  void QLDSolver::updateObjectiveEquations(std::vector<cfl_size_t>& workingMapping)
  {
    //TODO [mineur] : verify this reset is needed
    _C.setToZero();
    _d.setToZero();

    //SubMatrix sP(_C);
    //SubVector sq(_d);

    //int nt = 0;
    for (unsigned int i=0; i<_objectives.size(); ++i)
    {
      //int ni = (*_variable)(i).getSize();
      Variable& v = _objectives[i].objective->getVariable();
      //sP.rescope(ni, ni, nt, nt);
      //sq.rescope(ni, nt);

      //sP.setToZero();
      //sP.copyValuesFrom(_objectives[i]->getPi());
      
      //sq.copyValuesFrom(_objectives[i]->getqi());
      workingMapping.resize(v.getSize());
      addCompressedMatrix(*_variable, v, _objectives[i].objective->getPi(), _C, workingMapping, _objectives[i].weight);
      addCompressedVector(_objectives[i].objective->getqi(), _d, workingMapping, _objectives[i].weight);

      //nt += ni;
    }
    //CML_scal(.5, _d);  //to comply with QLDSolver definition
  }


  unsigned int QLDSolver::transferSimpleInequalityToBounds(MatrixBase& C, VectorBase& d)
  {
    std::vector<cfl_size_t> linesToBeChanged;

    // 1 - detect bound-like rows and copy the constraints into the corresponding bounds
    for (cfl_size_t r=0; r<C.get_nrows(); ++r)
    {
      cfl_size_t c;
      if (isASingleComponentRow(C, r, c))
      {
        linesToBeChanged.push_back(r);
        if (C(r,c)>0)   //lower bound
        {
          _xl[c] = std::max(-d[r]/C(r,c), _xl[c]);
        }
        else      //upper bound
        {
          _xu[c] = std::min(-d[r]/C(r,c), _xu[c]);
        }
      }
    }

    // 2 - suppress the rows from C and d by moving up the remaining rows
    linesToBeChanged.push_back(C.get_nrows());
    for (unsigned int i=0; i<linesToBeChanged.size()-1; ++i)
    {
      for (unsigned int j=linesToBeChanged[i]+1; j<linesToBeChanged[i+1]; ++j)
      {
        d[j-i-1] = d[j];
        for (cfl_size_t c=0; c<C.get_ncols(); ++c)
          C(j-i-1,c) = C(j,c);
      }
    }
    return (unsigned int)(linesToBeChanged.size()-1);
  }

  bool QLDSolver::isASingleComponentRow(const MatrixBase& C, const cfl_size_t rowIndex, cfl_size_t& returnColIndex)
  {
    int i=0;
    for (cfl_size_t c=0; c<C.get_ncols() && i<2; ++c)
    {
      if (fabs(C(rowIndex, c))>_eps)
      {
        if (i==0)
        {
          returnColIndex = c;
        }
        ++i;
      }
    }

    if (i==1)
      return true;
    else
      return false;
  }*/

  void testQLDSolver()
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
    
    QLDSolver solver;
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
}

// cmake:sourcegroup=Solvers
