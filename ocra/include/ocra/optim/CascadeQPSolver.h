/** @file CascadeQPSolver.h
  * @brief Declaration file of the CascadeQPSolver class.
  *
  *   Copyright (C) 2016 ISIR/CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Antoine Hoarau
  *	@date 11/04/2016
  */

#ifndef _OCRABASE_CASCADE_QP_SOLVER_H_
#define _OCRABASE_CASCADE_QP_SOLVER_H_

// includes
#include "ocra/optim/Solver.h"
#include "ocra/optim/OneLevelSolver.h"
#include "ocra/optim/Constraint.h"
#include "ocra/optim/QuadraticFunction.h"
#include "ocra/optim/CascadeQP.h"
#include "ocra/optim/CascadeQPStructures.h"
#include <ocra/control/Tasks/Task.h>

#include <memory>
#include <map>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace.
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems.
  */
namespace ocra
{
/** @class CascadeQPSolver
  *	@brief %CascadeQPSolver class.
  *
  * Hierarchical solver based on CascadeQP
  */
class CascadeQPSolver : public ocra::Solver
{
public:
    DEFINE_CLASS_POINTER_TYPEDEFS(CascadeQPSolver)

    CascadeQPSolver();

    void addTask(Task::Ptr task);
    void addSolver(OneLevelSolver::Ptr solver,int level);
    virtual std::string toString() const;
protected:
    virtual void doSolve(void);
    virtual void doPrepare(void);
    virtual void doConclude();
    virtual void printValuesAtSolution();
    
    
    std::map<int,OneLevelSolver::Ptr > solvermap;
    CascadeQP cqp;
    std::map<int,std::vector<Task::Ptr> > taskmap;
};
}

#endif	//_OCRABASE_CASCADE_QP_SOLVER_H_
