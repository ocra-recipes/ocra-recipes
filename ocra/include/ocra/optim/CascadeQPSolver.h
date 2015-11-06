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

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class CascadeQPSolver
    *	@brief %CascadeQPSolver class.
    *	@warning None
    *  
    * Hierarchical solver based on CascadeQP
    */
  class CascadeQPSolver : public Solver
  {
    // ------------------------ structures --------------------------------------
  public:
    struct HierarchyLevelConstraints
    {
      size_t _m;
      size_t _p;
      std::vector<LinearConstraint*> _equalities;
      std::vector<LinearConstraint*> _inequalities;
    };
  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
  protected:
  public:
    CascadeQPSolver();

    // ------------------------ public interface --------------------------------
  public:
    virtual const std::string& getMoreInfo(void) const;

    void addLinearConstraint(LinearConstraint* constraint, size_t hierarchyLevel);
    bool removeLinearConstraint(LinearConstraint* constraint, size_t hierarchyLevel);
    void removeLinearConstraint(LinearConstraint* constraint);

    // ------------------------ public methods ----------------------------------
  public:
    void updateSize(void);

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual const Solver::Result& doSolve(void);
    virtual void doPrepare(void);       

    virtual void recomputeVariable(void);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:
    void updateMatrixSize(void);
    void updateLevels(void);
    //void updateSize(void);

    // ------------------------ private static methods --------------------------
  private:
 
    // ------------------------ protected members -------------------------------
  protected:
    std::vector<HierarchyLevelConstraints*> _hierarchyLevels;   //sets of ocra constraints
    std::vector<HierarchyLevel*> _hierarchyInput;               //input vector for CascadeQP
    CascadeQP _qp;

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRABASE_CASCADE_QP_SOLVER_H_

// cmake:sourcegroup=toBeUpdated
