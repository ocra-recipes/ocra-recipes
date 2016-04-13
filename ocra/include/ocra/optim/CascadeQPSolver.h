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
#include <memory>

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
        DEFINE_CLASS_POINTER_TYPEDEFS(HierarchyLevelConstraints)
        size_t _m = 0;
        size_t _p = 0;
        std::vector<LinearConstraint::Ptr> _equalities;
        std::vector<LinearConstraint::Ptr> _inequalities;
    };

    CascadeQPSolver();
    virtual const std::string& getMoreInfo(void) const;

    void addLinearConstraint(LinearConstraint::Ptr constraint, size_t hierarchyLevel);
    bool removeLinearConstraint(LinearConstraint::Ptr constraint, size_t hierarchyLevel);
    void removeLinearConstraint(LinearConstraint::Ptr constraint);


    void updateSize(void);

protected:
    virtual void doSolve(void);
    virtual void doPrepare(void);
    virtual void doConclude();
    virtual void recomputeVariable(void);
    std::vector<HierarchyLevelConstraints::Ptr > _hierarchyLevels;
    std::vector<HierarchyLevel::Ptr > _hierarchyInput;
    std::vector<OneLevelSolver> allSolvers;
    CascadeQP _qp;
    void updateMatrixSize(void);
    void updateLevels(void);
private:
    bool _isPrepared;
};
}

#endif	//_OCRABASE_CASCADE_QP_SOLVER_H_
