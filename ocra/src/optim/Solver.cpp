#include "ocra/optim/Solver.h"
#include "ocra/optim/VariableMapping.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <stdexcept>
#include <fstream>

namespace ocra
{
  // --- Restricted access to auto_map ------------------------------

  const std::vector<int>& Solver::findMapping(Variable& var)
  {
    VariableMapping* result = _problemVariable.find(&var);
    ocra_assert(
      result &&
      "Attempt to find a variable, means you have a pointer on it, which means it has been added through a function, which means it should have been registered..."
      );
    return result->getMapping();
  }


  // --- Solver -----------------------------------------------------

  Solver::Solver()
    :NamedInstance("solver")
    ,_result()
    ,_problemVariable("SolverVariable")
    ,_objectives()
    ,_constraints()
    ,_memory()
    ,_autodumpFile()
    ,_autodump(false)
  {
  }

  const OptimizationResult& Solver::solve()
  {
    _result.solution.resize(_problemVariable.getVariable().getSize());
    doPrepare();

    if (_memory.capacity() > 0)
      _memory.push_back(toString());
    doSolve();
    if (_autodump && _result.info!=RETURN_SUCCESS)
      dump(_autodumpFile);

    doConclude();
    _problemVariable.setValue(_result.solution);

    return _result;
  }

  const OptimizationResult& Solver::getLastResult() const
  {
    return _result;
  }

  void Solver::printStatus(std::ostream& os) const
  {
    //TODO
    os << "Solver status:" << std::endl;
    os << "\tSize: " << n() << std::endl;
  }

  const std::string& Solver::getMoreInfo() const
  {
    static const std::string info = "Abstract solver";
    return info;
  }

  void Solver::internalAddObjective(const GenericObjective& objective)
  {
    if (std::find(_objectives.begin(), _objectives.end(), &objective) != _objectives.end())
    {
      std::stringstream ss;
      ss << "[ocra::Solver::internalAddObjective] Objective was already added in the solver, so it cannot be added again\n";
      ss << "\tObjective is named: " << objective.getName() << std::endl;
      throw std::runtime_error(ss.str());
    }

    objective.getFunction().connect<EVT_RESIZE>(*this, &Solver::onObjectiveResize);
    _objectives.push_back(&objective);
    _problemVariable.insert(&const_cast<GenericObjective*>(&objective)->getVariable());
  }

  void Solver::internalAddConstraint(const GenericConstraint& constraint)
  {
    if (std::find(_constraints.begin(), _constraints.end(), &constraint) != _constraints.end())
    {
      std::stringstream ss;
      ss << "[ocra::Solver::internalAddConstraint] Constraint was already added in the solver, so it cannot be added again\n";
      ss << "\tConstraint is named: " << constraint.getName() << std::endl;
      throw std::runtime_error(ss.str());
    }
    constraint.getFunction().connect<EVT_RESIZE>(*this, &Solver::onConstraintResize);
    constraint.connect(*this, &Solver::onConstraintResize);
    _constraints.push_back(&constraint);
    _problemVariable.insert(&const_cast<GenericConstraint*>(&constraint)->getVariable());
  }

  void Solver::internalRemoveObjective(const GenericObjective& objective)
  {
    std::vector<const GenericObjective*>::iterator it = std::find(_objectives.begin(), _objectives.end(), &objective);
    if(it == _objectives.end())
    {
      std::stringstream ss;
      ss << "[ocra::Solver::internalRemoveObjective] Objective was not added in the solver, so it cannot be removed\n";
      ss << "\tObjective is named: " << objective.getName() << std::endl;
      throw std::runtime_error(ss.str());
    }

    _objectives.erase(it);
    objective.getFunction().disconnect<EVT_RESIZE>(*this, &Solver::onObjectiveResize);
    _problemVariable.remove(&const_cast<GenericObjective*>(&objective)->getVariable());
  }

  void Solver::internalRemoveConstraint(const GenericConstraint& constraint)
  {
    std::vector<const GenericConstraint*>::iterator it = std::find(_constraints.begin(), _constraints.end(), &constraint);
    if(it == _constraints.end())
    {
      std::stringstream ss;
      ss << "[ocra::Solver::internalRemoveConstraint] Constraint was not added in the solver, or has already been removed, so it cannot be removed\n";
      ss << "\tConstraint is named: " << constraint.getName() << std::endl;
      throw std::runtime_error(ss.str());
    }

    _constraints.erase(it);
    constraint.disconnect(*this, &Solver::onConstraintResize);
    constraint.getFunction().disconnect<EVT_RESIZE>(*this, &Solver::onConstraintResize);
    _problemVariable.remove(&const_cast<GenericConstraint*>(&constraint)->getVariable());
  }

  void Solver::onConstraintResize(int timestamp)
  {
    //do nothing
  }

  void Solver::onObjectiveResize(int timestamp)
  {
    //do nothing
  }

  void Solver::setMemoryLevel(int level)
  {
    _memory.set_capacity(level);
  }

  void Solver::setAutoDumpFile(const std::string& file)
  {
    if(file.empty())
      throw std::runtime_error("[Solver::setAutoDumpFile] Invalid (empty) filename!");

    _autodumpFile = file;
  }

  void Solver::activateAutoDump()
  {
    _autodump = true;
  }

  void Solver::deactivateAutoDump()
  {
    _autodump = false;
  }

  void Solver::dump(const std::string& file) const
  {
    if(file.empty())
      std::copy(_memory.begin(), _memory.end(), std::ostream_iterator<std::string>(std::cout, "\r\n\r\n----------------\r\n"));
    else
    {
      std::ofstream os(file.c_str());
      std::copy(_memory.begin(), _memory.end(), std::ostream_iterator<std::string>(os, "\r\n\r\n----------------\r\n"));
    }
  }
}

// cmake:sourcegroup=Solvers
