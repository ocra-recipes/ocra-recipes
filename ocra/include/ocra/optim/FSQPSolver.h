/** @file FSQPSolver.h
  * @brief Declaration file of the FSQPSolver class.
  *
  *   Copyright (C) 2011 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 11/04/08
  *
  */

#ifndef _OCRABASE_FSQP_SOLVER_H_
#define _OCRABASE_FSQP_SOLVER_H_

// includes
#include "Solver.h"
#include "OFSQP.h"
#include "FSQPConstraintManager.h"
#include "ocra/optim/Buffer.h"


namespace ocra
{
  /** @class FSQPSolver
    *	@brief FSQPSolver class.
    *	@warning None
    *  
    * This class offers an OCRA interface for a simple subset of the FSQP solver possiblities. The problem solved here
    * is:
    *   min. sum f_i
    *   s.t. g_j<=0
    *        h_k=0
    *       l<=x<=u
    * where the function f_i are real-valued functions while the functions g_j and h_k can have an output space of
    * dimension greater than one. The sets of indices for i, j and k might be empty. In particular, having an objective
    * function is not mandatory.
    * All functions need to be at least C^1 (at least close to the optimum), but they do not need to provide their
    * gradient. In case they don't, the gradient will be evaluate by finite differences. This approximation can however
    * be costly and poor, so it is advised that each function provides its gradient.
    *
    * Compared to all the possibilities described in fsqp manual, this implementation does not handle:
    *   - minimizing the max of several objectives
    *   - related objective or constraints function
    *
    */
  class FSQPSolver : public Solver, public OFSQPProblem
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef Map<VectorXd> VectorMap;
    typedef Map<MatrixXd> MatrixMap;


    enum eFsqpProblemType
    {
      NORMAL_PB=0,      //A=0: min sum f_i
      INFINITE_PB       //A=1: min |sum f_i|
    };

    enum eFsqpAlgo
    {
      AL=0,             //B=0: algorithm FSQP-AL, decrease of the (modified) objectif function at each step
      NL                //B=1: algorithm FSQP-NL, decrease of the (modified) objectif function after at most 4 steps
    };

    enum eFsqpEvaluationDomainPolicy
    {
      LOOSE=1,          //C=1: objectives and constraints can be evaluated outside the feasible space
      STRICT            //C=2: objectives and constraints can not be evaluated outside the feasible space
    };

    enum eFsqpPrintOption
    {
      NONE,
      FINAL,
      EACH,
      FULL
    };

  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
    FSQPSolver(const FSQPSolver&);
    FSQPSolver& operator=(const FSQPSolver&);
  protected:
  public:
    FSQPSolver();

    // ------------------------ public interface --------------------------------
  public:
    /** Add/remove an objective.
      * Will throw a runtime_error in case \a obj already exist and is added again, or if one tries to remove \a obj 
      * and \a obj does not appear in the problem.
      */
    //@{
    void addObjective(GenericObjective& obj);
    void removeObjective(Function& obj);
    void removeObjective(GenericObjective& obj);
    //@}

    /** Add/remove a constraint.
      * Will throw a runtime_error for similar reasons as add/removeObjective.
      */
    //@{
    void addConstraint(LinearConstraint& constraint);
    void addConstraint(GenericConstraint& constraint);
    void removeConstraint(LinearConstraint& constraint);
    void removeConstraint(GenericConstraint& constraint);
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
    void clearNonLinearEqualityConstraints();
    void clearLinearEqualityConstraints();
    void clearInequalityConstraints();
    void clearNonLinearInequalityConstraints();
    void clearLinearInequalityConstraints();
    void clearBounds();
    //@}

    /** Set the initial guess. The order of \a x0 is done w.r.t. to the order of the Variable \a ref */
    void set_x0(const VectorXd& x0, const Variable& ref);

    /** Set options*/
    //@{
    void setA(eFsqpProblemType type);
    void setB(eFsqpAlgo type);
    void setC(eFsqpEvaluationDomainPolicy type);
    void setMode(int m);
    void setPrintMode(eFsqpPrintOption m);
    void setPrintStep(int N);
    void setPrintOption(int option);
    void setMaxIter(int n);
    void setInfinity(double infinity);
    void setEps(double eps);
    void setEqnViol(double eps);
    void setUDelta(double udelta);
    //@}

    eFsqpProblemType            getA()            const;
    eFsqpAlgo                   getB()            const;
    eFsqpEvaluationDomainPolicy getC()            const;
    int                         getMode()         const;
    eFsqpPrintOption            getPrintMode()    const;
    int                         getPrintStep()    const;
    int                         getPrintOption()  const;
    int                         getMaxIter()      const;
    double                      getInfinity()     const;
    double                      setEps()          const;
    double                      setEqnViol()      const;
    double                      setUDelta()       const;


    void printValuesAtSolution();

    /** Returns the state of the solver (e.g. matrices) as a string. */
    std::string toString() const;

    // ------------------------ public methods ----------------------------------
  public:
    void obj(int nparam, int j, double* x, double* fj, void* cd);
    void constr(int nparam, int j, double* x, double* gj, void* cd);
    void gradob(int nparam, int j, double* x, double* gradfj, void* cd);
    void gradcn(int nparam, int j, double* x, double* gradgj, void* cd);

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    void doPrepare();
    void doSolve();
    void doConclude();

    /** \internal Callback method to be invoked when the function of a constraint indicates an EVT_RESIZE, or the
      * constraint itself triggers a EVT_CHANGE_BOUNDS_NUMBER event.
      */
    virtual void onConstraintResize(int timestamp);

    /** \internal Callback method to be invoked when the function of an objective triggers a EVT_RESIZE event. */
    virtual void onObjectiveResize(int timestamp);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:
    void resize();
    void updateBounds();
    eReturnInfo translateReturnInfo() const;

    void addBounds_(DiagonalLinearConstraint& constraint);
    void removeBounds_(DiagonalLinearConstraint& constraint);
    void checkNewX(int nparam, double* x);

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:
    OFSQP     solver;

    int       nparam;
    int       nf;
    int       nfsr;
    int       nineqn; 
    int       nineq; 
    int       neqn; 
    int       neq;
    int       ncsrl;
    int       ncsrn;
    int       mode;
    int       iprint;
    int       miter;
    int       inform;
    double    bigbnd;
    double    eps;
    double    epseqn;
    double    udelta;

    VectorXd  x0;
    VectorMap bl;
    VectorMap bu;
    VectorMap f;
    VectorMap g;
    VectorMap lambda;

    Map<VectorXi>   mesh_pts;

    Buffer<double>  _buffer;

    bool _allObjectivesProvideAGradient;
    std::vector<GenericObjective*>          _objectives;
    std::vector<DiagonalLinearConstraint*>  _bounds;
    FSQPConstraintManager                   _constraints;

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };

  void testFSQPSolver01();
  void testFSQPSolver02();
}

#endif	//_OCRABASE_FSQP_SOLVER_H_
