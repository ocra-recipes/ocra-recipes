/** @file CascadeQPSolver.h
  * @brief Declaration file of the CascadeQPSolver class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  * @author Yacoubi Salim
  *	@date 09/07/24
  */  

#ifndef _OCRABASE_CASCADE_QP_SOLVER_H_
#define _OCRABASE_CASCADE_QP_SOLVER_H_

// includes
#include "ocra/optim/Solver.h"
#include "ocra/optim/Constraint.h"
#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/QuadraticFunction.h"
#include "ocra/optim/CascadeQP.h"
#include <memory>

namespace ocra{
    typedef std::shared_ptr<LinearConstraint> LinearConstraintPtr;
}
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
    struct HierarchyLevelConstraints
    {
      size_t _m = 0;
      size_t _p = 0;
      std::vector<LinearConstraintPtr> _equalities;
      std::vector<LinearConstraintPtr> _inequalities;
    };

    CascadeQPSolver();
    virtual const std::string& getMoreInfo(void) const;

    void addLinearConstraint(LinearConstraintPtr constraint, size_t hierarchyLevel);
    bool removeLinearConstraint(LinearConstraintPtr constraint, size_t hierarchyLevel);
    void removeLinearConstraint(LinearConstraintPtr constraint);


    void updateSize(void);

  protected:
    virtual void doSolve(void);
    virtual void doPrepare(void);       
    virtual void recomputeVariable(void);
    std::vector<std::shared_ptr<HierarchyLevelConstraints> > _hierarchyLevels;
    std::vector<std::shared_ptr<HierarchyLevel> > _hierarchyInput;
    CascadeQP _qp;
    void updateMatrixSize(void);
    void updateLevels(void);
private:
    bool _isPrepared;
  };
}

#endif	//_OCRABASE_CASCADE_QP_SOLVER_H_

