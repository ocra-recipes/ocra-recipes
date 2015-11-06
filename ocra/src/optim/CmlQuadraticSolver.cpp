#if 0

#include "ocra/optim/CmlQuadraticSolver.h"

#include "cfl/Timer.h"

#include <algorithm>
#include <sstream>

#include <fstream>
#include <stdexcept>

using namespace xde;

namespace ocra
{
  CmlQuadraticSolver::CmlQuadraticSolver(int type)
    :QuadraticSolver("CmlQuadraticSolver"), _solverType(type)
  {
    switch (type)
    {
    case CMLQPSOLVER_LEMKE:
      _solver = cmlQPSolver::New(cmlQPSolver::LEMKE_SOLVER);
      break;
    case CMLQPSOLVER_GAUSS_SEIDEL:
      _solver = cmlQPSolver::New(cmlQPSolver::GAUSS_SEIDEL_SOLVER);
      break;
    default:
      throw std::runtime_error("[CmlQuadraticSolver::CmlQuadraticSolver] Invalid solver type");
    }
  }

  void CmlQuadraticSolver::setTolerance(double epsilon)
  {
    _solver->setTolerance(epsilon);
  }

  void CmlQuadraticSolver::setMaxIteration(cfl_size_t maxIter)
  {
    switch (_solverType)
    {
    case CMLQPSOLVER_LEMKE:
      _solver->setNumberOfLemkeLoopsMax(maxIter);
      break;
    case CMLQPSOLVER_GAUSS_SEIDEL:
      throw std::runtime_error("[CmlQuadraticSolver::setMaxIteration] There is no method to set the maximum iteration number with this type of solver");
    }
  }

  /*  const double CmlQuadraticSolver::getTolerance(void) const
  {
  _solver->getTolerance();
  }*/

  cfl_size_t CmlQuadraticSolver::getMaxIteration(void) const
  {
    switch (_solverType)
    {
    case CMLQPSOLVER_LEMKE:
      return _solver->getNumberOfLemkeLoops();
    case CMLQPSOLVER_GAUSS_SEIDEL:
      throw std::runtime_error("[CmlQuadraticSolver::getMaxIteration] There is no method to get the maximum iteration number with this type of solver");
    }
    return 0;
  }

  const std::string& CmlQuadraticSolver::getMoreInfo(void) const
  {
    //TODO [todo] 
    return "";
  }


  void CmlQuadraticSolver::addLinearEqualityConstraint(LinearConstraint* constraint)
  {
    if (constraint->isInequality())
      throw std::runtime_error("[CmlQuadraticSolver::addLinearEqualityConstraint] added Constraint is not an equality" );

    _equalityConstraints.push_back(constraint);
    _m += constraint->getDimension();
    constraint->attach(*this);
    addVariable(constraint->getVariable()); //invalidate _isPrepared
  }

  void CmlQuadraticSolver::addLinearInequalityConstraint(LinearConstraint* constraint)
  {
    if (constraint->isEquality())
      throw std::runtime_error("[CmlQuadraticSolver::addLinearEqualityConstraint] added Constraint is not an inequality" );
    
    _inequalityConstraints.push_back(constraint);
    _p += constraint->getDimension();
    constraint->attach(*this);
    addVariable(constraint->getVariable()); //invalidate _isPrepared
  }

  void CmlQuadraticSolver::removeConstraint(LinearConstraint* constraint)
  {
    std::vector<LinearConstraint*>* v;
    if (constraint->isEquality())
      v = &_equalityConstraints;
    else
      v = &_inequalityConstraints;

    std::vector<LinearConstraint*>::iterator it = std::find(v->begin(), v->end(), constraint);
    if (it != v->end())
    {
      removeVariable((*it)->getVariable()); //invalidate _isPrepared
      v->erase(it);
      constraint->completelyDetach(*this);
      if (constraint->isEquality())
        _m -= constraint->getDimension();
      else
        _p -= constraint->getDimension();
    }
  }


  void CmlQuadraticSolver::setObjective(QuadraticFunction* obj, real weight)
  {
    //TODO [todo] : attach to function
    if (obj->getDimension() != 1)
      throw std::runtime_error("[QLDSolver::setObjective] dimension of objective function is not 1. Multidimensionnal objectives are not handled by this solver");

    _objectives.clear();
    WeightedObjective wo;
    wo.objective = obj;
    wo.weight = weight;
    _objectives.push_back(wo);
    obj->attach(*this);

    addVariable(obj->getVariable());
  }

  void CmlQuadraticSolver::addObjective(QuadraticFunction* obj, real weight)
  {
    if (obj->getDimension() != 1)
      throw std::runtime_error("[CmlQuadraticSolver::setObjective] dimension of objective function is not 1. Multidimensionnal objectives are not handled by this solver");


    if (_objectives.size() == 0)
    {
      setObjective(obj, weight);
    }
    else
    {
      addVariable(obj->getVariable());
      WeightedObjective wo;
      wo.objective = obj;
      wo.weight = weight;
      _objectives.push_back(wo);
      obj->attach(*this);
    }
  }


  void CmlQuadraticSolver::removeObjective(QuadraticFunction* obj)
  {
    std::vector<WeightedObjective>::iterator it; //  = std::find(_objectives.begin(), _objectives.end(), obj);

    for (it = _objectives.begin(); it != _objectives.end(); ++it)
    {
      if (it->objective == obj)
        break;
    }

    if (it != _objectives.end())
    {
      obj->completelyDetach(*this);
      _objectives.erase(it);
    }
  }


  void CmlQuadraticSolver::printValuesAtSolution(void)
  {
    _variable->setValue(_result.solution);
    std::cout << "objective(s):" <<std::endl;
    for (unsigned int i=0; i<_objectives.size(); ++i)
      std::cout << _objectives[i].objective->getValues() << std::endl;
    std::cout << "equalities:" <<std::endl;
    for (unsigned int i=0; i<_equalityConstraints.size(); ++i)
      std::cout << _equalityConstraints[i]->getValues() << std::endl;
    std::cout << "inequalities:" <<std::endl;
    for (unsigned int i=0; i<_inequalityConstraints.size(); ++i)
      std::cout << _inequalityConstraints[i]->getValues() << std::endl;
  }


  bool CmlQuadraticSolver::checkConstraints(void)
  {
    _variable->setValue(_result.solution);
    bool b = true;
    for (unsigned int i=0; i<_equalityConstraints.size(); ++i)
    {
      const Vector& v =_equalityConstraints[i]->getValues();
      for (cfl_size_t j=0; j<v.getSize(); ++j)
      {
        if (fabs(v[j]) > 1.e-8)
        {
          if (b)
          {
            std::cout << "equalities : " << std::endl;
            b=false;
          }
          std::cout << "(" << i << "," << j << "): " << v[j] << std::endl;
        }
      }
    }
    bool c = true;
    for (unsigned int i=0; i<_inequalityConstraints.size(); ++i)
    {
      const Vector& v =_inequalityConstraints[i]->getValues();
      for (cfl_size_t j=0; j<v.getSize(); ++j)
      {
        if (v[j] > 1.e-7)
        {
          if (c)
          {
            std::cout << "inequalities : " << std::endl;
            c=false;
          }
          std::cout << "(" << i << "," << j << "): " << v[j] << std::endl;
        }
      }
    }
    return b&&c;
  }



  const MatrixBase& CmlQuadraticSolver::getP(void) const
  {
    return _Q;
  }

  const MatrixBase& CmlQuadraticSolver::getA(void) const
  {
    return _H;
  }

  const MatrixBase& CmlQuadraticSolver::getC(void) const
  {
    _minus_G.resize(_G.get_nrows(), _G.get_ncols());
    _minus_G.setToZero();
    CML_axpy(-1,_G, _minus_G);
    return _minus_G;
  }

  const VectorBase& CmlQuadraticSolver::getq(void) const
  {
    _minus_k.resize(_k.getSize());
    _minus_k.copyValuesFrom(_k);
    _minus_k *= -1;
    return _minus_k;
  }

  const VectorBase& CmlQuadraticSolver::getb(void) const
  {
    return _a;
  }

  const VectorBase& CmlQuadraticSolver::getd(void) const
  {
    _minus_c.resize(_c.getSize());
    _minus_c.copyValuesFrom(_c);
    _minus_c *= -1;
    return _minus_c;
  }

  const VectorBase& CmlQuadraticSolver::getu(void) const
  {
    _u.resize(_k.getSize());
    _u.fill(1e30);
    return _u;
  }

  const VectorBase& CmlQuadraticSolver::getl(void) const
  {
    _l.resize(_k.getSize());
    _l.fill(-1e30);
    return _l;
  }




  const Solver::Result& CmlQuadraticSolver::doSolve(void)
  {
    static int cpt =0;
/*    cflTimer t1,t2;
    t1.reset();
    t1.start();*/
    prepare();
#ifdef OCRA_REAL_IS_DOUBLE
    cmlDDenseVector& res = _result.solution;
#else
    cmlDDenseVector r;
    cmlDDenseVector& res = &r;
#endif
    updateMatrices();
//    t1.stop();
    //res.resize(_n);
    //Debug
/*    SubMatrix subH(_H);
    subH.rescope(_m-6, _n, 0, 0);
    Matrix Debug(_m-6, _n);
    Debug.copyValuesFrom(subH);
    SubVector suba(_a);
    suba.rescope(_m-6, 0);
    Vector debuga(_m-6);
    debuga.copyValuesFrom(suba);*/
//    t2.reset();
//    t2.start();
    int info = _solver->solveQP(_Q, _k, /*Debug*/ _H, /*suba*/_a, _G, _c, res);
//    t2.stop();
    //TODO [todo] : change _result.inform
//    std::cout << info << " x_star " << "new " << _result.solution << std::endl;
    //printSolution(res, *_variable, 10);
    //printValuesAtSolution();
//    if (!checkConstraints() && cpt%10  == 0)
//    {
//      printEquation(_G, _c, false, 9);
//      printSolution(res, *_variable, 10);
//    }
    //printSolution(res, *_variable, 10);
//    ++cpt;
//    bool toto = false;
//    if(toto)
    //{
      //printEquation(_H, _a, false, 9);
      //printEquation(_G, _c, false, 9);
      //printEquation(_Q, _k, false, 9);
      //printSolution(res, *_variable, 15,8);
    //}
    //system("pause");
//    std::cout << "prepare " << t1.getTime() << std::endl;
//    std::cout << "solve " << t2.getTime() << std::endl;
    //checkConstraints();
    _result.inform = info;
    return _result;
  }


  void CmlQuadraticSolver::doPrepare(void)
  {
    updateMatrixDimension();
  }


  void CmlQuadraticSolver::recomputeVariable(void)
  {
    _variable->clear();
    for (unsigned int i=0; i<_objectives.size(); ++i)
    {
      addVariable(_objectives[i].objective->getVariable());
    }
    for (unsigned int i=0; i<_equalityConstraints.size(); ++i)
    {
      addVariable(_equalityConstraints[i]->getVariable());
    }
    for (unsigned int i=0; i<_inequalityConstraints.size(); ++i)
    {
      addVariable(_inequalityConstraints[i]->getVariable());
    }
    _isPrepared = false;
  }



  void CmlQuadraticSolver::updateMatrixDimension(void)
  {
    //TODO [todo] : correctly
    //we assume _n, _m and _p have been correctly updated

    _Q.resize(_n, _n);
    _Q.setToZero();
    _k.resize(_n);
    _k.setToZero();
    _H.resize(_m, _n);
    _H.setToZero();
    _a.resize(_m);
    _a.setToZero();
    _G.resize(_p, _n);
    _G.setToZero();
    _c.resize(_p);
    _c.setToZero();
  }

  void CmlQuadraticSolver::updateSize(void)
  {
    //TODO [mineur] : be more subtle
    //this is called when a function change its dimension
    _m=0;
    _p=0;
    for (unsigned int i=0; i<_equalityConstraints.size(); ++i)
      _m += _equalityConstraints[i]->getDimension();
    for (unsigned int i=0; i<_inequalityConstraints.size(); ++i)
      _p += _inequalityConstraints[i]->getDimension();

    _isPrepared = false;
    //updateMatrixDimension();
  }

  void CmlQuadraticSolver::updateMatrices(void)
  {
    //updateSize(); //TODO [???]: should not be needed
    //std::vector<cfl_size_t> workingMapping(_n);
   
    updateEqualityEquations(_workingMapping);
    updateInequalityEquations(_workingMapping);
    updateObjectiveEquations(_workingMapping);

/*    std::ofstream aofA("A.txt");
    if (aofA.is_open())
    {
      for (cfl_size_t i=0; i<_m; ++i)
      {
        for (cfl_size_t j=0; j<_n; ++j)
          aofA << _H(i,j) << " ";
        aofA << std::endl;
      }
      aofA.close();
    }

    std::ofstream aofb("b.txt");
    if (aofb.is_open())
    {
      aofb << std::endl;
      for (cfl_size_t i=0; i<_m; ++i)
      {
        aofb << _a[i] << " ";
      }
      aofb << std::endl;
      aofb.close();
    }

    std::ofstream aofC("C.txt");
    if (aofC.is_open())
    {
      for (cfl_size_t i=0; i<_p; ++i)
      {
        for (cfl_size_t j=0; j<_n; ++j)
          aofC << _G(i,j) << " ";
        aofC << std::endl;
      }
      aofC << std::endl;
      aofC.close();
    }

    std::ofstream aofd("d.txt");
    if (aofd.is_open())
    {
      for (cfl_size_t i=0; i<_p; ++i)
      {
        aofd << _c[i] << " ";
      }
      aofd << std::endl;
      aofd.close();
    }
*/
/*    std::ofstream aofP("P.txt");
    if (aofP.is_open())
    {
       for (cfl_size_t i=0; i<_n; ++i)
      {
        for (cfl_size_t j=0; j<_n; ++j)
          aofP << _Q(i,j) << " ";
        aofP << std::endl;
      }
      aofP << std::endl;
      aofP.close();
    }
*/
/*    std::ofstream aofq("q.txt");
    if (aofq.is_open())
    {
      for (cfl_size_t i=0; i<_n; ++i)
      {
        aofq << _k[i] << " ";
      }
      aofq.close();
    }
*/
//    std::cout.precision(3);
//    std::cout << "A " <<_H << std::endl;
//    std::cout << "b " <<_a << std::endl;

//    std::cout << "C " <<_G << std::endl;
//    std::cout << "d " <<_c << std::endl;

//    std::cout << "P " <<_Q << std::endl;
//    std::cout << "q " <<_k << std::endl;

    bool toto = false;
    if(toto)
    {
      //printEquation(_H, _a, false, 9);
      printEquation(_G, _c, false, 9);
      //printEquation(_Q, _k, false, 9);
    }
//    system("pause");
  }

  void CmlQuadraticSolver::updateEqualityEquations(std::vector<cfl_size_t>& workingMapping)
  {
    _H.setToZero(); //TODO [mineur] : be more subtle : elements that will be replaced don't need to be put to 0
    SubMatrix sA(_H);
    SubVector sb(_a);
    cfl_size_t m = 0;
    for (unsigned int i=0; i<_equalityConstraints.size(); ++i)
    {
      Constraint<LinearFunction>* cstr = _equalityConstraints[i];
      cfl_size_t dim = cstr->getDimension();
      sA.rescope(dim, _n, m, 0);
      sb.rescope(dim, m);

      workingMapping.resize(cstr->getVariable().getSize());
      uncompressMatrix(*_variable, cstr->getVariable(), cstr->getGradients(), sA, workingMapping);
//      sb.copyValuesFrom(((LinearFunction)*cstr).getb());
      sb.copyValuesFrom(cstr->getFunction()->getb());

      m+=dim;
    }


/*    //debug
    SubMatrix M(_H);
    M.rescope(6,_n,_m-6,0);
    Matrix MtM(_n, _n);
    CML_gemm<'t','n'>(1., M, M, 0., MtM);
    std::ofstream aofM("MtM.txt");
    if (aofM.is_open())
    {
       for (cfl_size_t i=0; i<_n; ++i)
      {
        for (cfl_size_t j=0; j<_n; ++j)
          aofM << MtM(i,j) << " ";
        aofM << std::endl;
      }
      aofM << std::endl;
      aofM.close();
    }

    SubVector v(_a);
    v.rescope(6,_m-6);
    Vector Mtv(_n);
    CML_gemv<'t'>(1., M, v, 0., Mtv);
    std::ofstream aofv("Mtv.txt");
    if (aofv.is_open())
    {
       for (cfl_size_t i=0; i<_n; ++i)
      {
          aofv << Mtv[i] << " ";
      }
      aofv << std::endl;
      aofv.close();
    }*/
  }


  void CmlQuadraticSolver::updateInequalityEquations(std::vector<cfl_size_t>& workingMapping)
  {
    _G.setToZero(); //TODO [mineur] : be more subtle : elements that will be replaced don't need to be put to 0
    SubMatrix sC(_G);
    SubVector sd(_c);
    int m = 0;
    for (unsigned int i=0; i<_inequalityConstraints.size(); ++i)
    {
      Constraint<LinearFunction>* cstr = _inequalityConstraints[i];
      cfl_size_t dim = cstr->getDimension();
      sC.rescope(dim, _n, m, 0);
      sd.rescope(dim, m);

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
      
      m+=dim;
    }
  }


  void CmlQuadraticSolver::updateObjectiveEquations(std::vector<cfl_size_t>& workingMapping)
  {
    //TODO [mineur] : verify this reset is needed
    _Q.setToZero();
    _k.setToZero();

    int nt = 0;
    for (unsigned int i=0; i<_objectives.size(); ++i)
    {
      /*int ni = (*_variable)(i).getSize();
      sP.rescope(ni, ni, nt, nt);
      sq.rescope(ni, nt);

      sP.setToZero();
      sP.copyValuesFrom(_objectives[i]->getPi());
      sq.copyValuesFrom(_objectives[i]->getqi());

      nt += ni;
      */
      Variable& v = _objectives[i].objective->getVariable();
      workingMapping.resize(v.getSize());
      addCompressedMatrix(*_variable, v, _objectives[i].objective->getPi(), _Q, workingMapping, _objectives[i].weight);
      addCompressedVector(_objectives[i].objective->getqi(), _k, workingMapping, _objectives[i].weight);
    }
    CML_scal(-1., _k);  //to comply with cmlQPSolver definition
  }

}

#include "ocra/optim/BaseVariable.h"
#include "ocra/optim/CompositeVariable.h"

namespace ocra 
{
  void testSolveCmlQP(void)
  {
    VariableManager m;
    BaseVariable x("x", 1, m);
    BaseVariable y("y", 1, m);
    BaseVariable z("z", 1, m);

    CompositeVariable xy("xy", m);
      xy.add(x).add(y);
    CompositeVariable xz("xz", m);
      xz.add(x).add(z);
    CompositeVariable yz("yz", m);
      yz.add(y).add(z);
    CompositeVariable X("X", m);
      X.add(x).add(y).add(z);

    Matrix A1(1,2);
      A1(0,0) = 1;
      A1(0,1) = 1;
    Vector b1(1);
      b1[0] = -1;
    LinearFunction f1(xy, A1, b1);
    LinearConstraint e(&f1, true);

    Matrix A2(2,3);
      A2(0,0) = 1; A2(0,1) = -1; A2(0,2) = 1;
      A2(1,0) = 2; A2(1,1) = 1; A2(1,2) = 1;
    Vector b2(2);
      b2[0] = -1; b2[1] = 2;
    LinearFunction f2(X, A2, b2);
    LinearConstraint i(&f2, false);

    //objectives
    Matrix P1(1,1);
      P1(0,0) = 1;
    Vector q1(1);
      q1[0] = 0;
    QuadraticFunction o1(x, P1, q1, 0);

    Matrix P2(2,2);
      P2(0,0) = 1; P2(0,1) = 0;
      P2(1,0) = 0; P2(1,1) = 1;
    Vector q2(2);
      q2[0] = 0; q2[1] = 0;
    QuadraticFunction o2(yz, P2, q2, 0);


    CmlQuadraticSolver solver;
    solver.setObjective(&o1);
    solver.addObjective(&o2);
    solver.addLinearEqualityConstraint(&e);
    solver.addLinearInequalityConstraint(&i);
    std::cout << solver.solve().solution << std::endl;
  }

  void testSolveCmlQPWithNonFeasiblePb(void)
  {
    VariableManager m;
    BaseVariable x("x", 1, m);

    Matrix P1(1,1);
      P1(0,0) = 1;
    Vector q1(1);
      q1[0] = 0;
    QuadraticFunction o1(x, P1, q1, 0);

    Matrix A1(2,1);
      A1(0,0) = 1;
      A1(1,0) = -1;
    Vector b1(2);
      b1[0] = 1;
      b1[1] = 1;
    LinearFunction f(x, A1, b1);
    LinearConstraint inequality(&f, false);

    CmlQuadraticSolver solver;
    solver.addObjective(&o1);
    solver.addLinearInequalityConstraint(&inequality);
    solver.solve();
  }
}

#endif

// cmake:sourcegroup=toBeUpdated
