#include "FSQPConstraintManager.h"
#include <algorithm>

namespace
{
  int getConstraintOnesidedDimension(const ocra::GenericConstraint* c)
  {
    if (c->getType() == ocra::CSTR_LOWER_AND_GREATER)
      return 2*c->getDimension();
    else return c->getDimension();
  }
}


namespace ocra
{
  bool operator< (const FSQPConstraintManager::Mapping& m1, const FSQPConstraintManager::Mapping& m2)
  {
    return m1.length < m2.length;
  }

  FSQPConstraintManager::FSQPConstraintManager()
    : _invalidatedIndex(0)
    , _nineqn(0)
    , _nineq(0)
    , _neqn(0)
    , _neq(0)
  {
    _mappings.push_back(Mapping());
  }

  void FSQPConstraintManager::addConstraint(LinearConstraint& constraint)
  {
    const int ni = (int)(_nonLinearIneq.size()+_linearIneq.size());
    if (constraint.isEquality())
    {
      _invalidatedIndex = std::min(_invalidatedIndex, ni+(int)(_nonLinearEq.size()+_linearEq.size()));
      _linearEq.push_back(&constraint);
    }
    else
    {
      _invalidatedIndex = std::min(_invalidatedIndex, ni);
      _linearIneq.push_back(&constraint);
    }
  }

  void FSQPConstraintManager::addConstraint(GenericConstraint& constraint)
  {
    if (constraint.isEquality())
    {
      _invalidatedIndex = std::min(_invalidatedIndex, (int)(_nonLinearIneq.size()+_linearIneq.size()+_nonLinearIneq.size()));
      _nonLinearEq.push_back(&constraint);
    }
    else
    {
      _invalidatedIndex = std::min(_invalidatedIndex, (int)_nonLinearIneq.size());
      _nonLinearIneq.push_back(&constraint);
    }
  }

  void FSQPConstraintManager::removeConstraint(LinearConstraint& constraint)
  {
    if (constraint.isEquality())
    {
      std::vector<LinearConstraint*>::iterator it = std::find(_linearEq.begin(), _linearEq.end(), &constraint);
      if (it != _linearEq.end())
      {
        _invalidatedIndex = std::min(_invalidatedIndex, (int)(_nonLinearIneq.size()+_linearIneq.size()+_nonLinearEq.size())+(int)(it-_linearEq.begin()));
        _linearEq.erase(it);
      }
    }
    else
    {
      std::vector<LinearConstraint*>::iterator it = std::find(_linearIneq.begin(), _linearIneq.end(), &constraint);
      if (it != _linearIneq.end())
      {
        _invalidatedIndex = std::min(_invalidatedIndex, (int)_nonLinearIneq.size()+(int)(it-_linearIneq.begin()));
        _linearIneq.erase(it);
      }
    }
  }

  void FSQPConstraintManager::removeConstraint(GenericConstraint& constraint)
  {
    if (constraint.isEquality())
    {
      std::vector<GenericConstraint*>::iterator it = std::find(_nonLinearEq.begin(), _nonLinearEq.end(), &constraint);
      if (it != _nonLinearEq.end())
      {
        _invalidatedIndex = std::min(_invalidatedIndex, (int)(_nonLinearIneq.size()+_linearIneq.size())+(int)(it-_nonLinearEq.begin()));
        _nonLinearEq.erase(it);
      }
    }
    else
    {
      std::vector<GenericConstraint*>::iterator it = std::find(_nonLinearIneq.begin(), _nonLinearIneq.end(), &constraint);
      if (it != _nonLinearIneq.end())
      {
        _invalidatedIndex = std::min(_invalidatedIndex, (int)(it-_nonLinearIneq.begin()));
        _nonLinearIneq.erase(it);
      }
    }
  }

  void FSQPConstraintManager::updateMapping() const
  {
    if (_invalidatedIndex == std::numeric_limits<int>::max())
      return ;

    const size_t n1 = _nonLinearIneq.size();
    const size_t n2 = _linearIneq.size() + n1;
    const size_t n3 = _nonLinearEq.size() + n2;
    const size_t n4 = _linearEq.size() + n3;
    const size_t n = n4+1;
    
    // 1. Resize the vector if needed
    if (n<_mappings.size())
      _mappings.resize(n);
    else
    {
      for (size_t i=_mappings.size(); i<n; ++i)
        _mappings.push_back(Mapping(static_cast<int>(i)));
    }

    // 2. Update the counting values
    size_t i = _invalidatedIndex;
    for (;i<n1; ++i)
      _mappings[i+1].length = _mappings[i].length + getConstraintOnesidedDimension(_nonLinearIneq[i]);
    _nineqn = _mappings[i].length;
    for (;i<n2; ++i)
      _mappings[i+1].length = _mappings[i].length + getConstraintOnesidedDimension(_linearIneq[i-n1]);
    _nineq = _mappings[i].length;
    for (;i<n3; ++i)
      _mappings[i+1].length = _mappings[i].length + getConstraintOnesidedDimension(_nonLinearEq[i-n2]);
    _neqn = _mappings[i].length - _nineq;
    for (;i<n4; ++i)
      _mappings[i+1].length = _mappings[i].length + getConstraintOnesidedDimension(_linearEq[i-n3]);
    _neq = _mappings[i].length - _nineq;

    _invalidatedIndex = std::numeric_limits<int>::max();
  }

  void FSQPConstraintManager::invalidateMapping()
  {
    _invalidatedIndex = 0;
  }

  std::pair<GenericConstraint*, int> FSQPConstraintManager::operator[] (int i) const
  {
    if (_invalidatedIndex != std::numeric_limits<int>::max())
      updateMapping();

    std::vector<Mapping>::iterator it = std::upper_bound(_mappings.begin(), _mappings.end(), Mapping(i,0));
    assert(it != _mappings.end());
    --it;
    size_t j = it->index;
    if (j<_nonLinearIneq.size())
      return std::make_pair(_nonLinearIneq[j], i-it->length);
    
    j -= _nonLinearIneq.size();
    if (j<_linearIneq.size())
      return std::make_pair(_linearIneq[j], i-it->length);

    j -= _linearIneq.size();
    if (j<_nonLinearEq.size())
      return std::make_pair(_nonLinearEq[j], i-it->length);

    j -= _nonLinearEq.size();
      return std::make_pair(_linearEq[j], i-it->length);
  }

  double FSQPConstraintManager::getValue(int i) const
  {
    std::pair<const GenericConstraint*, int> p = (*this)[i];
    
    switch (p.first->getType())
    {
      case CSTR_EQUAL_ZERO:         return  p.first->getValue(p.second);
      case CSTR_EQUAL_B:            return  p.first->getValue(p.second) - p.first->getB()[p.second];
      case CSTR_LOWER_ZERO:         return  p.first->getValue(p.second);
      case CSTR_LOWER_U:            return  p.first->getValue(p.second) - p.first->getU()[p.second];
      case CSTR_GREATER_ZERO:       return -p.first->getValue(p.second);
      case CSTR_GREATER_L:          return -p.first->getValue(p.second) + p.first->getL()[p.second];
      case CSTR_LOWER_AND_GREATER:
        if (p.second<p.first->getDimension())
          return -p.first->getValue(p.second) + p.first->getL()[p.second];
        else
        {
          const int n = p.second - p.first->getDimension();
          return p.first->getValue(n) - p.first->getU()[n];
        }
      default: 
        ocra_assert(false && "this should never happen");
    }

    return -1; // should never happen, just to avoid warnings
  }

  const FSQPConstraintManager::ScalarMultMatrixXdRow FSQPConstraintManager::getGradient(int i) const
  {
    std::pair<GenericConstraint*, int> p = (*this)[i];
    return getGradient(p);
  }

  const FSQPConstraintManager::ScalarMultMatrixXdRow FSQPConstraintManager::getGradient(const std::pair<GenericConstraint*, int>& p) const
  {
    ocra_assert(p.first->canCompute<PARTIAL_X>());
    ocra_assert(p.second < p.first->getDimension() || (p.first->getType() == CSTR_LOWER_AND_GREATER && p.second < 2*p.first->getDimension()));
    
    switch (p.first->getType())
    {
      case CSTR_EQUAL_ZERO:         return  1.*p.first->getJacobian(p.second);
      case CSTR_EQUAL_B:            return  1.*p.first->getJacobian(p.second);
      case CSTR_LOWER_ZERO:         return  1.*p.first->getJacobian(p.second);
      case CSTR_LOWER_U:            return  1.*p.first->getJacobian(p.second);
      case CSTR_GREATER_ZERO:       return -1.*p.first->getJacobian(p.second);
      case CSTR_GREATER_L:          return -1.*p.first->getJacobian(p.second);
      case CSTR_LOWER_AND_GREATER:
        if (p.second<p.first->getDimension())
          return -1.*p.first->getJacobian(p.second);
        else
          return 1.*p.first->getJacobian(p.second - p.first->getDimension());
      default:
        ocra_assert(false && "this should never happen");
    }
  }
}

#include "LinearFunction.h"
#include "QuadraticFunction.h"
#include "FunctionHelpers.h"

namespace ocra
{
  void testFsqpMapping()
  {
    const int n=5;
    BaseVariable x("x", n);
    EqualZeroConstraintPtr<LinearFunction> el1(new LinearFunction(x, MatrixXd::Random(3,n), VectorXd::Random(3)));
    EqualZeroConstraintPtr<LinearFunction> el2(new LinearFunction(x, MatrixXd::Random(4,n), VectorXd::Random(4)));
    EqualZeroConstraintPtr<LinearFunction> el3(new LinearFunction(x, MatrixXd::Random(2,n), VectorXd::Random(2)));

    LessThanZeroConstraintPtr<LinearFunction> il1(new LinearFunction(x, MatrixXd::Random(2,n), VectorXd::Random(2)));
    LessThanZeroConstraintPtr<LinearFunction> il2(new LinearFunction(x, MatrixXd::Random(1,n), VectorXd::Random(1)));
    LessThanZeroConstraintPtr<LinearFunction> il3(new LinearFunction(x, MatrixXd::Random(2,n), VectorXd::Random(2)));

    MatrixXd P = MatrixXd::Random(n,n);
    P = P.transpose()*P;

    EqualZeroConstraintPtr<QuadraticFunction> eq1(new QuadraticFunction(x, P, VectorXd::Random(n), 0.));
    EqualZeroConstraintPtr<QuadraticFunction> eq2(new QuadraticFunction(x, P, VectorXd::Random(n), 0.));

    LessThanZeroConstraintPtr<QuadraticFunction> iq1(new QuadraticFunction(x, P, VectorXd::Random(n), 0.));
    LessThanZeroConstraintPtr<QuadraticFunction> iq2(new QuadraticFunction(x, P, VectorXd::Random(n), 0.));


    FSQPConstraintManager m;
    m.addConstraint(el1);
    m.updateMapping();
    m.addConstraint(iq1);
    m.updateMapping();
    m.addConstraint(eq1);
    m.updateMapping();
    m.addConstraint(il1);
    m.updateMapping();

    m[2];
    m[5];

    m.removeConstraint(il1);
    m.updateMapping();
  }
}