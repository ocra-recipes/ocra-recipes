#if 0

#include "ocra/optim/CascadeQPSolver.h"

#include <algorithm>
#include "ocra/optim/utilities.h"

namespace ocra
{

  CascadeQPSolver::CascadeQPSolver()
    :Solver("CascadeQPSolver")
  {
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                           get More Info                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////
  
  const std::string& CascadeQPSolver::getMoreInfo(void) const
  {
    return "";
  }


/////////////////////////////////////////////////////////////////////////////////////////////////
//                                       add Linear Constraint                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////

  void CascadeQPSolver::addLinearConstraint(LinearConstraint* constraint, size_t hierarchyLevel)
  {
    //increasing _hierarchyLevels size if necessary
    if (hierarchyLevel >= _hierarchyLevels.size())
    {
      for (size_t i=_hierarchyLevels.size(); i<=hierarchyLevel; ++i)
      {
        HierarchyLevelConstraints* h = new HierarchyLevelConstraints();
        h->_m = 0;
        h->_p = 0;
        _hierarchyLevels.push_back(h);
        _hierarchyInput.push_back(new HierarchyLevel());
      }
    }

    //adding constraints
    if (constraint->isEquality())
    {
      _hierarchyLevels[hierarchyLevel]->_equalities.push_back(constraint);
      _hierarchyLevels[hierarchyLevel]->_m += constraint->getDimension();
    }
    else
    {
      _hierarchyLevels[hierarchyLevel]->_inequalities.push_back(constraint);
      _hierarchyLevels[hierarchyLevel]->_p += constraint->getDimension();
    }

    //update variable
    constraint->attach(*this);
    addVariable(constraint->getVariable());
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                      remove Linear Constraint                               //
/////////////////////////////////////////////////////////////////////////////////////////////////

  bool CascadeQPSolver::removeLinearConstraint(LinearConstraint* constraint, size_t hierarchyLevel)
  {
    std::vector<LinearConstraint*>* v;
    if (constraint->isEquality())
      v = &_hierarchyLevels[hierarchyLevel]->_equalities;
    else
      v = &_hierarchyLevels[hierarchyLevel]->_inequalities;

    std::vector<LinearConstraint*>::iterator it = std::find(v->begin(), v->end(), constraint);
    if (it != v->end())
    {
      removeVariable((*it)->getVariable()); //invalidate _isPrepared
      v->erase(it);
      constraint->completelyDetach(*this);
      if (constraint->isEquality())
        _hierarchyLevels[hierarchyLevel]->_m -= constraint->getDimension();
      else
        _hierarchyLevels[hierarchyLevel]->_p -= constraint->getDimension();
      return true;
    }
    return false;
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                      remove Linear Constraint                               //
/////////////////////////////////////////////////////////////////////////////////////////////////

  void CascadeQPSolver::removeLinearConstraint(LinearConstraint* constraint)
  {
    for (unsigned int i=0; i<_hierarchyLevels.size(); ++i)
      removeLinearConstraint(constraint, i);
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                            do Solve                                         //
/////////////////////////////////////////////////////////////////////////////////////////////////

  const Solver::Result& CascadeQPSolver::doSolve(void)
  {
    prepare();
    updateLevels();
    _qp.clear();
    _qp.addHierarchyLevel(_hierarchyInput);
    const FinalSolution& sol = _qp.solveCascadeQP();
    _result.inform = sol.r;
    memcpy(_result.solution.getDataArray(), sol.yf.getDataArray(), _result.solution.getSize()*sizeof(double));
    //memmove(_result.solution.getDataArray(), sol.yf.getDataArray(), _result.solution.getSize()*sizeof(double)); 
    //printSolution(sol.yf, *_variable, 10);
    return _result;
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                           do Prepare                                        //
/////////////////////////////////////////////////////////////////////////////////////////////////
  
  void CascadeQPSolver::doPrepare(void)
  {
    updateMatrixSize();
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                       recompute Variable                                    //
/////////////////////////////////////////////////////////////////////////////////////////////////

  void CascadeQPSolver::recomputeVariable(void)
  {
    _variable->clear();

    for (unsigned int i=0; i<_hierarchyLevels.size(); ++i)
    { 
      for (unsigned int k=0; k<_hierarchyLevels[i]->_equalities.size(); ++k)
      {
        addVariable(_hierarchyLevels[i]->_equalities[k]->getVariable());
      } 
      for (unsigned int j=0; j<_hierarchyLevels[i]->_inequalities.size(); ++j)
      {
        addVariable(_hierarchyLevels[i]->_inequalities[j]->getVariable());
      }














      //_variable->printTree();

      //for (unsigned int j=0, k=0; j<_hierarchyLevels[i]->_equalities.size(), k<_hierarchyLevels[i]->_inequalities.size(); ++j, k++)
      //{
      //  addVariable(_hierarchyLevels[i]->_equalities[j]->getVariable());
      //  addVariable(_hierarchyLevels[i]->_inequalities[k]->getVariable());
      //}
    }

    _isPrepared = false;
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                       update Matrix Size                                    //
/////////////////////////////////////////////////////////////////////////////////////////////////

  void CascadeQPSolver::updateMatrixSize(void)
  {
    for (unsigned int i=0; i<_hierarchyInput.size(); ++i)
    {
      _hierarchyInput[i]->A.resize((cfl_size_t)_hierarchyLevels[i]->_m, _n);
      _hierarchyInput[i]->b.resize((cfl_size_t)_hierarchyLevels[i]->_m);
      _hierarchyInput[i]->C.resize((cfl_size_t)_hierarchyLevels[i]->_p, _n);
      _hierarchyInput[i]->d.resize((cfl_size_t)_hierarchyLevels[i]->_p);
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                           update Levels                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////

  void CascadeQPSolver::updateLevels(void)
  {
    updateSize();
    for (unsigned int i=0; i<_hierarchyLevels.size(); ++i)
    {
      cfl_size_t m=0;
      cfl_size_t p=0;
      _hierarchyInput[i]->A.setToZero();
      _hierarchyInput[i]->C.setToZero();
      SubMatrix sA(_hierarchyInput[i]->A);
      SubVector sb(_hierarchyInput[i]->b);
      SubMatrix sC(_hierarchyInput[i]->C);
      SubVector sd(_hierarchyInput[i]->d);
      for (unsigned int j=0; j<_hierarchyLevels[i]->_equalities.size(); ++j)
      {
        LinearConstraint* cstr = _hierarchyLevels[i]->_equalities[j];
        cfl_size_t dim = cstr->getDimension();
        sA.rescope(dim, _n, m, 0);
        sb.rescope(dim, m);

        _workingMapping.resize(cstr->getVariable().getSize());
        uncompressMatrix(*_variable, cstr->getVariable(), cstr->getGradients(), sA, _workingMapping);
        sb.copyValuesFrom(cstr->getFunction()->getb());
        CML_scal(-1, sb);
        //sA.scalarMultInPlace(-1);

        m+=dim;
      }
      //writeInFile(_hierarchyInput[i]->A, "A_CASCADE_QP.txt", true);
      //writeInFile(_hierarchyInput[i]->b, "b_CASCADE_QP.txt", true);

      for (unsigned int j=0; j<_hierarchyLevels[i]->_inequalities.size(); ++j)
      {
        Constraint<LinearFunction>* cstr = _hierarchyLevels[i]->_inequalities[j];
        cfl_size_t dim = cstr->getDimension();
        sC.rescope(dim, _n, p, 0);
        sd.rescope(dim, p);

        _workingMapping.resize(cstr->getVariable().getSize());
        uncompressMatrix(*_variable, cstr->getVariable(), cstr->getGradients(), sC, _workingMapping);
        sd.copyValuesFrom(cstr->getFunction()->getb());
        CML_scal(-1, sd);
        //sC.scalarMultInPlace(-1);
      
        p+=dim;
      }
      //writeInFile(_hierarchyInput[i]->C, "C_CASCADE_QP.txt", true);
      //writeInFile(_hierarchyInput[i]->d, "d_CASCADE_QP.txt", true);
    }
  }


  void CascadeQPSolver::updateSize(void)
  {
    //TODO [mineur] : be more subtle
    //this is called when a function changes its dimension
    for ( size_t i = 0 ; i < _hierarchyLevels.size()  ; i++ )
    {
      _hierarchyLevels[i]->_m=0;
      _hierarchyLevels[i]->_p=0;
      for (unsigned int j=0; j < _hierarchyLevels[i]->_equalities.size(); ++j)
      {
        _hierarchyLevels[i]->_m += _hierarchyLevels[i]->_equalities[j]->getDimension();
      }
      for (unsigned int k=0; k < _hierarchyLevels[i]->_inequalities.size(); ++k)
      {
        _hierarchyLevels[i]->_p += _hierarchyLevels[i]->_inequalities[k]->getDimension();
      }

    _isPrepared = false;
    //updateMatrixDimension();
    }
  }
}

#endif

// cmake:sourcegroup=toBeUpdated
