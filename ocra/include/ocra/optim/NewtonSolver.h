/** @file NewtonSolver.h
  * @brief Declaration file of the NewtonSolver class.
  *
  *   Copyright (C) 2011 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 11/01/31
  */

#ifndef _OCRABASE_NEWTON_SOLVER_H_
#define _OCRABASE_NEWTON_SOLVER_H_

// includes
#include "Solver.h"
#include "ocra/optim/Buffer.h"

namespace ocra
{
  /** @class NewtonSolver
    *	@brief %NewtonSolver class.
    *	@warning None
    *  
    * This class implements a Newton Method to optimize a weighted sum of squared functions:
    * \f$ \mbox{min.} \dfrac{1}{2} \sum_i{ w_i \left\| f_i \right\|^2_2} \f$
    * The functions \f$ f_i \f$ must provide their jacobian. Additionnaly if they can provides their hessian, it will
    * be used for a complete Newton Method, unless the user choosed to use a quasi-Newton approach. If a function can't
    * produce its hessian, or a quasi Newton method is asked, the hessian will be approximated by \f$ J_i^t J_i \f$
    * with \f$ J_i \f$ the jacobian matrix of \f$ f_i \f$.
    *
    * The solver is taking itself the squared norm of each function, so that the ocra Objective instance has to contain
    * \f$ f_i \f$, not \left\| f_i \right\|^2_2.
    *
    *
    * Let us define \f$ F(x) = \dfrac{1}{2} \sum_i{ w_i \left\| f_i(x) \right\|^2_2} \f$, 
    * and \f$ H(x)=\frac{\partial^2 F}{\partial x^2}(x)\f$.
    * Starting from an initial guess \f$ x_0 \f$, the algorithm performs iterations 
    * \f$ x_{n+1} = x_n - \alpha H(x_n)^{-1} \nabla F(x_n) \f$ until the residual \f$ \left\|x_{n+1} - x_n\right\|_2\f$
    * is smaller than \f$ \epsilon \f$ or the maximal number of iterations has been reached.
    *
    * The following parameters and options can be changed by the user:
    * - \f$ \alpha \f$ is by default adaptative, using a heuristic. \f$ \alpha \f$ can be chosen to be constant and 
    * equal to 1,
    * - \f$ \epsilon \f$ can be changed,
    * - the maximal number of iterations \a maxIter,
    * - the initial guess \f$ x_0 \f$.
    *
    * In case the initial guess has not the good size, or was not initialized, the algorithm starts from \f$ x_0=0 \f$.
    * In case the solver is called repeatedly and the user does not provide an initial guess, the last \f$ x_n \f$ of
    * the previous run is used (unless the problem changed its size, in which case 0 will be used).
    */
  class NewtonSolver: public Solver
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef Eigen::Map<MatrixXd> MatrixMap;
    typedef Eigen::Map<VectorXd> VectorMap;

    struct eNewtonInfo
    {
      bool    success;
      bool    nonPositiveH;
      bool    usedDefaultGuess;
      bool    smallAlpha;
      int     iter;
      double  residual;
    };

    // ------------------------ constructors ------------------------------------
  private:
    NewtonSolver(const NewtonSolver&);
    NewtonSolver& operator=(const NewtonSolver&);

  public:
    NewtonSolver(bool fullNewton=true);

    // ------------------------ public interface --------------------------------
  public:
    /** Add/remove an objective.
      * Will throw a runtime_error in case \a obj already exist and is added again, or if one tries to remove \a obj 
      * and \a obj does not appear in the problem.
      */
    //@{
    void  addObjective(GenericObjective& obj);
    void  removeObjective(Function& obj);
    void  removeObjective(GenericObjective& obj);
    //@}

    void    setAdaptativeAlpha(bool adapt);
    bool    getAdaptativeAlpha()            const;
    void    setEpsilon(double eps);
    double  getEpsilon()                    const;
    void    setMaxIter(int maxIter);
    int     getMaxIter()                    const;
    void    set_x0(const VectorXd& x0);
    const   VectorXd& get_x0()              const;

    void printValuesAtSolution();

    /** Returns the state of the solver (e.g. matrices) as a string. */
    std::string toString() const;

    // ------------------------ protected methods -------------------------------
  protected:
    void doPrepare();
    void doSolve();
    void doConclude();

    // ------------------------ private methods ---------------------------------
  private:
    void    initInfo();
    void    newtonSolve();
    double  compute_alpha();
    void    compute_H();
    void    compute_g();
    void    translateReturnInfo(eReturnInfo& orcInfo);

    // ------------------------ private members ---------------------------------
  private:
    bool                            _completeMethod;
    bool                            _adaptativeAlpha;
    int                             _maxIter;
    double                          _alpha;
    double                          _epsilonSqr;
    //double                          _r;
    std::vector<GenericObjective*>  _objectives;          //< set of objectives

    eNewtonInfo                     _info;

    VectorXd                        _x0;
    VectorMap                       _x;
    MatrixMap                       _H;
    double*                         _tmpHbuf;
    VectorMap                       _g;
    double*                         _tmpgbuf;
    VectorMap                       _p;
    Buffer<double>                  _buffer;

    Eigen::LDLT<MatrixXd>           _ldlt;

  };

  void testNewtonSolver01();
  void testNewtonSolver02();
}

#endif	//_OCRABASE_NEWTON_SOLVER_H_

// cmake:sourcegroup=Solvers
