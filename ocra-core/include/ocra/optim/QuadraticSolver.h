/** @file QuadraticSolver.h
  * @brief Declaration file of the QuadraticSolver class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/06/11
  *
  * File history:
  *  - 10/06/23: Escande Adrien, Evrard Paul - Adaptation to the new Function interface and switch to Eigen.
  *  - 10/06/30: Escande Adrien - Documentation.
  */

#ifndef _OCRABASE_QUADRATIC_SOLVER_H_
#define _OCRABASE_QUADRATIC_SOLVER_H_

// includes
#include "ocra/optim/Solver.h"
#include "ocra/optim/Constraint.h"
#include "ocra/optim/Objective.h"
#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/QuadraticFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class QuadraticSolver
    *	@brief %QuadraticSolver class.
    *	@warning None
    *  
    * Base class for a quadratic solver
    * Objective functions and constraints given as input assume the following form:
    * Minimize 1/2 x^t P x + q^t x
    * Subj. to Ax + b  = b'
    *          l <= Cx + d <= u
    *         (xl <= x <= xu)
    *
    * This class gives functionalities to maintain the right count in the number of equality or inequality constraints.
    * Bounds can be either considered separatly as bounds or as 'normal' constraints, together with the other linear
    * constraints.
    */
  class QuadraticSolver: public Solver
  {
    // ------------------------ constructors ------------------------------------
  private:
    /** This class is non copyable*/
    QuadraticSolver(const QuadraticSolver&);
    QuadraticSolver& operator= (const QuadraticSolver&);

  protected:
    /** Constructor to be used by derived class
      *
      * \param[in] boundsAsConstraints. If true, the bounds will be considered as normal constraints
      */
    QuadraticSolver(bool boundsAsConstraints = false)
      : NamedInstance("quadratic solver")
      , _m(0), _p(0), _ps(0), _boundsAsConstraints(boundsAsConstraints)
    {
    }

    // ------------------------ public interface --------------------------------
  public:
    /** Add/remove an objective.
      * Will throw a runtime_error in case \a obj already exist and is added again, or if one tries to remove \a obj 
      * and \a obj does not appear in the problem.
      */
    //@{
    void addObjective(QuadraticObjective& obj);
    void removeObjective(QuadraticFunction& obj);
    void removeObjective(QuadraticObjective& obj);
    //@}

    /** Add/remove a constraint.
      * Will throw a runtime_error for similar reasons as add/removeObjective.
      */
    //@{
    void addConstraint(LinearConstraint& constraint);
    void removeConstraint(LinearConstraint& constraint);
    //@}
    
     /** Add/remove a bound.
      * Will throw a runtime_error for similar reasons as add/removeObjective.
      */
    //@{
    void addBounds(BoundConstraint& constraint);
    void addBounds(IdentityConstraint& constraint);
    void removeBounds(BoundConstraint& constraint);
    void removeBounds(IdentityConstraint& constraint);
    //@}

    /** Clear all the corresponding data of the problem. Equivalent to call remove() one by one for each of them */
    //@{
    void clearObjectives();
    void clearConstraints();
    void clearEqualityConstraints();
    void clearInequalityConstraints();
    void clearBounds();
    //@}

    /** Print the value of each objective and constraint at the previously found solution
      * 
      * \warning This assumes the value of the problem variable is still the value of the solution. It does not set the 
      * value of the variable to the solution again. Thus, if the variable's value has been changed (directly or 
      * through one of its sub-variable) between the last call to solve() and the call to this methods, values 
      * displayed will not be the value at the solution.
      */
    void printValuesAtSolution();

    /** Get debug info. Don't use these functions in critical loops.
      */
    //@{
    std::string toString() const;
    virtual MatrixXd getP() const = 0;
    virtual VectorXd getq() const = 0;
    virtual MatrixXd getA() const = 0;
    virtual VectorXd getb() const = 0;
    virtual VectorXd getbp() const = 0;
    virtual MatrixXd getC() const = 0;
    virtual VectorXd getd() const = 0;
    virtual VectorXd getl() const = 0;
    virtual VectorXd getu() const = 0;
    virtual VectorXd getxl() const = 0;
    virtual VectorXd getxu() const = 0;
    //@}

    // ------------------------ protected methods ---------------------------------
  protected:
    /** Overload of the callback defined in Solver.*/
    void onConstraintResize(int timestamp);

    /** Compute the value of _m, _p and _ps for the actual set of constraints (and bounds if _boundsAsConstraints
      * is true), and put back _invalidatedMP to false.
      */
    void recomputeMP();

    // ------------------------ private methods ---------------------------------
  private:
    /** \internal sub-methods for addBounds and removeBounds*/
    //@{
    void addBounds_(DiagonalLinearConstraint& constraint);
    void removeBounds_(DiagonalLinearConstraint& constraint);
    //@}

    // ------------------------ protected members -------------------------------
  protected:
    std::vector<LinearConstraint*>              _equalityConstraints;   //< all the equality constraints in the problem 
    std::vector<LinearConstraint*>              _inequalityConstraints; //< all the inequality constraints in the problem 
    std::vector<QuadraticObjective*>            _objectives;            //< all the objectives in the problem 
    std::vector<DiagonalLinearConstraint*>      _bounds;                //< all the bounds in the problem 

    size_t _m;    //< number of unidimensional equality constraints (i.e. sum of dimension of equality constraints)
    size_t _p;    //< number of unidimensional inequality constraints (i.e. sum of dimension of inequality constraints)
    size_t _ps;   //< number of simple unidimensional inequality constraints (i.e. of the type Ax+b<=b' or Ax+b>=b')

    const bool  _boundsAsConstraints; //< true if the bounds should be treated as generic constraints
    bool        _invalidatedMP;       //< true when _m, _p and _ps are not up-to-date. 
  };
}

#endif	//_OCRABASE_QUADRATIC_SOLVER_H_

// cmake:sourcegroup=Solvers
