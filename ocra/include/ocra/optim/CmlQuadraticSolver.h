/** @file CmlQuadraticSolver.h
  * @brief Declaration file of the CmlQuadraticSolver class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/04/29
  */

#ifndef _OCRABASE_CML_QUADRATIC_SOLVER_H_
#define _OCRABASE_CML_QUADRATIC_SOLVER_H_

//ocra includes
#include "ocra/optim/CompositeVariable.h"
#include "ocra/optim/QuadraticSolver.h"

//cml includes
#include "cml/QPSolver.h"

//std includes
#include <vector>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class CmlQuadraticSolver
    *	@brief %CmlQuadraticSolver class.
    *	@warning None
    *  
    * Wrapping of the QP solver from cml
    * 
    */
  class CmlQuadraticSolver : public QuadraticSolver
  {
    // ------------------------ structures --------------------------------------
  public:
    enum eCmlQPSolverType
    {
      CMLQPSOLVER_LEMKE,
      CMLQPSOLVER_GAUSS_SEIDEL
    };
    //TODO: A discuter : garder CML ? Oui on le garde, 
    //      mais ca signifie qu'il faut sortirCmlQuadraticSolver de base-optim.
    //      -> Où mettre CmlQuadraticSolver ?
    //      Simplifier la gestion actuelle avec deux enum ocra::cmlQPSolverType / xde::cmlQPSolver ?
  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
    CmlQuadraticSolver(CmlQuadraticSolver&);
  protected:
  public:
    CmlQuadraticSolver(int type = xde::cmlQPSolver::LEMKE_SOLVER);

    // ------------------------ public interface --------------------------------
  public:
    void setTolerance(double epsilon);
    void setMaxIteration(cfl_size_t maxIter);

//    const double getTolerance(void) const;
    cfl_size_t getMaxIteration(void) const;

    virtual const std::string& getMoreInfo(void) const;

    virtual void addLinearEqualityConstraint(LinearConstraint* constraint);
    virtual void addLinearInequalityConstraint(LinearConstraint* constraint);
    virtual void removeConstraint(LinearConstraint* constraint);
    virtual void setObjective(QuadraticFunction* obj, real weight=1.);
    virtual void addObjective(QuadraticFunction* obj, real weight=1.);
    virtual void removeObjective(QuadraticFunction* obj);
//    void setVariable(Variable& var);

    virtual void printValuesAtSolution(void);
    bool checkConstraints(void);


    virtual const MatrixBase& getP(void) const;
    virtual const MatrixBase& getA(void) const;
    virtual const MatrixBase& getC(void) const;
    virtual const VectorBase& getq(void) const;
    virtual const VectorBase& getb(void) const;
    virtual const VectorBase& getd(void) const;
    virtual const VectorBase& getu(void) const;
    virtual const VectorBase& getl(void) const;

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual const Solver::Result& doSolve(void);
    virtual void  doPrepare(void);
    virtual void  recomputeVariable(void);
    

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:
    /*void updateN(void);
    void updateM(void);
    void updateP(void);
    void updateNM(void);
    void updateNP(void);
    void updateMNP(void);*/
    void updateMatrixDimension(void);
    void updateSize(void);

    void updateMatrices(void);
    void updateEqualityEquations(std::vector<cfl_size_t>& workingMapping);
    void updateInequalityEquations(std::vector<cfl_size_t>& workingMapping);
    void updateObjectiveEquations(std::vector<cfl_size_t>& workingMapping);

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:
    int           _solverType;
    xde::cmlQPSolver*  _solver;

    Matrix _Q;   //< P (nxn)
    Vector _k;   //< q (n)
    Matrix _H;   //< A (mxn)
    Vector _a;   //< b (m)
    Matrix _G;   //< C (pxn)
    Vector _c;   //< d (p)

    //for output
    mutable Vector _minus_k;
    mutable Matrix _minus_G;
    mutable Vector _minus_c;
    mutable Vector _l;
    mutable Vector _u;

    
    /*bool _nChanged;
    bool _mChanged;
    bool _pChanged;*/

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };

  void testSolveCmlQP(void);
  void testSolveCmlQPWithNonFeasiblePb(void);
}

#endif	//_OCRABASE_CML_QUADRATIC_SOLVER_H_

// cmake:sourcegroup=toBeUpdated

