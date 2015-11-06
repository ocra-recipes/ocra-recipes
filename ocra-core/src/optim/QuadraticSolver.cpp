#include "ocra/optim/QuadraticSolver.h"

namespace
{
  template<class T, class Y>
  void eraseFunctionFromSolver
    (std::vector<T*>& v, void (ocra::Solver::*internalRemove)(const Y&), ocra::Solver* solver)
  {
    while(v.size())
    {
      (solver->*internalRemove)(*v.back());
      v.pop_back();
    }
  }
}

namespace ocra
{
  void QuadraticSolver::addObjective(QuadraticObjective& obj)
  {
    internalAddObjective(obj);
    _objectives.push_back(&obj);
  }

  void QuadraticSolver::removeObjective(QuadraticFunction& obj)
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

  void QuadraticSolver::removeObjective(QuadraticObjective& obj)
  {
    internalRemoveObjective(obj);
    _objectives.erase(std::find(_objectives.begin(), _objectives.end(), &obj));
  }

  void QuadraticSolver::addConstraint(LinearConstraint& constraint)
  {
    internalAddConstraint(constraint);
    if(constraint.isEquality())
      _equalityConstraints.push_back(&constraint);
    else
      _inequalityConstraints.push_back(&constraint);
    _invalidatedMP = true;
  }

  void QuadraticSolver::removeConstraint(LinearConstraint& constraint)
  {
    internalRemoveConstraint(constraint);
    if(constraint.isEquality())
      _equalityConstraints.erase(std::find(_equalityConstraints.begin(), _equalityConstraints.end(), &constraint));
    else
      _inequalityConstraints.erase(std::find(_inequalityConstraints.begin(), _inequalityConstraints.end(), &constraint));
    _invalidatedMP = true;
  }

  
  void QuadraticSolver::addBounds(BoundConstraint& constraint)
  {
    addBounds_(constraint);
  }

  void QuadraticSolver::addBounds(IdentityConstraint& constraint)
  {
    addBounds_(constraint);
  }

  void QuadraticSolver::addBounds_(DiagonalLinearConstraint& constraint)
  {
    ocra_assert(constraint.isInequality() && "How queer! A bound constraint that is no inequality...");
    if (_boundsAsConstraints)
      addConstraint(constraint);
    else
    {
      internalAddConstraint(constraint);
      _bounds.push_back(&constraint);
    }
  }


  void QuadraticSolver::removeBounds(BoundConstraint& constraint)
  {
    removeBounds_(constraint);
  }

  void QuadraticSolver::removeBounds(IdentityConstraint& constraint)
  {
    removeBounds_(constraint);
  }

  void QuadraticSolver::removeBounds_(DiagonalLinearConstraint& constraint)
  {
    ocra_assert(constraint.isInequality() && "How queer! A bound constraint that is no inequality...");
    if (_boundsAsConstraints)
      removeConstraint(constraint);
    else
    { 
      internalRemoveConstraint(constraint);
      _bounds.erase(std::find(_bounds.begin(), _bounds.end(), &constraint));
    }
  }


  void QuadraticSolver::clearObjectives()
  {
    eraseFunctionFromSolver(_objectives, &QuadraticSolver::internalRemoveObjective, this);
  }

  void QuadraticSolver::clearConstraints()
  {
    clearEqualityConstraints();
    clearInequalityConstraints();
  }

  void QuadraticSolver::clearEqualityConstraints()
  {
    eraseFunctionFromSolver(_equalityConstraints, &QuadraticSolver::internalRemoveConstraint, this);
  }

  void QuadraticSolver::clearInequalityConstraints()
  {
    eraseFunctionFromSolver(_inequalityConstraints, &QuadraticSolver::internalRemoveConstraint, this);
  }

  void QuadraticSolver::clearBounds()
  {
    eraseFunctionFromSolver(_bounds, &QuadraticSolver::internalRemoveConstraint, this);
  }


  void QuadraticSolver::printValuesAtSolution()
  {
    std::cout << "objective(s):" <<std::endl;
    for (unsigned int i=0; i<_objectives.size(); ++i)
      std::cout << _objectives[i]->getValue() << std::endl;
    std::cout << "equalities:" <<std::endl;
    for (unsigned int i=0; i<_equalityConstraints.size(); ++i)
      std::cout << _equalityConstraints[i]->getValue() << std::endl;
    std::cout << "inequalities:" <<std::endl;
    for (unsigned int i=0; i<_inequalityConstraints.size(); ++i)
      std::cout << _inequalityConstraints[i]->getValue() << std::endl;
  }

  void QuadraticSolver::onConstraintResize(int timestamp)
  {
    _invalidatedMP = true;
  }

  void QuadraticSolver::recomputeMP()
  {
    _m = 0;
    _p = 0;
    _ps = 0;
    for (size_t i=0; i<_equalityConstraints.size(); ++i)
      _m += _equalityConstraints[i]->getDimension();

    for (size_t i=0; i<_inequalityConstraints.size(); ++i)
    {
      _p += _inequalityConstraints[i]->getDimension();
      if (_inequalityConstraints[i]->getType() == CSTR_LOWER_AND_GREATER)
        _ps += 2*_inequalityConstraints[i]->getDimension();
      else
        _ps += _inequalityConstraints[i]->getDimension();
    }

    _invalidatedMP = false;
  }

  std::string QuadraticSolver::toString() const
  {
    std::stringstream ss;

    ss << "P = "  << std::endl << getP()  << std::endl << std::endl;
    ss << "q = "  << std::endl << getq()  << std::endl << std::endl;
    ss << "A = "  << std::endl << getA()  << std::endl << std::endl;
    ss << "b = "  << std::endl << getb()  << std::endl << std::endl;
    ss << "b' = " << std::endl << getbp() << std::endl << std::endl;
    ss << "C = "  << std::endl << getC()  << std::endl << std::endl;
    ss << "d = "  << std::endl << getd()  << std::endl << std::endl;
    ss << "l = "  << std::endl << getl()  << std::endl << std::endl;
    ss << "u = "  << std::endl << getu()  << std::endl << std::endl;
    ss << "xl = " << std::endl << getxl() << std::endl << std::endl;
    ss << "xu = " << std::endl << getxu() << std::endl << std::endl;

    return ss.str();
  }
}

// cmake:sourcegroup=Solvers
