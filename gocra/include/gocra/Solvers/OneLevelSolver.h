/**
 * \file OneLevelSolver.h
 * \author Joseph Salini
 *
 * \brief Define concrete solvers class for \b one-level LQP solver.
 */

#ifndef __ONELEVELSOLVER_H__
#define __ONELEVELSOLVER_H__

#include "gocra/Solvers/gOcraSolver.h"

#include "quadprog/QuadProg++.h"

#include "ocra/optim/ObjQLD.h"


namespace gocra
{

/** \addtogroup solver
 * \{
 */

/** \brief Abstract solver class that only consider one level of importance for all tasks.
 *
 * Concrete instances are gocra::OneLevelSolverWithQuadProg and gocra::OneLevelSolverWithQLD .
 */
class OneLevelSolver: public gOcraSolver
{
public:
    OneLevelSolver();
    virtual ~OneLevelSolver();

    virtual void setObjectiveLevel(ocra::QuadraticObjective& obj, int level);

    virtual std::string toString() const;

protected:

    virtual void doSolve() = 0;

protected:

    Eigen::MatrixXd _C; // Quadratic Matrix of the task (n,n)
    Eigen::VectorXd _d; // Quadratic vector of the task (n)

};




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Solver class that only consider one level of importance for all tasks using Quadprog++.
 *
 * It uses a linear quadratic program which comes from http://quadprog.sourceforge.net/ .
 *
 * Quadprog++ solve the following problem:
 *
 * \f{align*}{
 *     \argmin{\x} &: \;  \frac{1}{2} \x\tp G \x + \vec{g}_0\tp \x \\
 *     & CE \x + \vec{ce}_0 =    \vec{0} \\
 *     & CI \x + \vec{ci}_0 \geq \vec{0}
 * \f}
 */
class OneLevelSolverWithQuadProg: public OneLevelSolver
{
public:
    OneLevelSolverWithQuadProg();
    virtual ~OneLevelSolverWithQuadProg();

protected:

    // function in doPrepare
    virtual void doSolve();
    virtual void updateObjectiveEquations();
    virtual void updateConstraintEquations();

};





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Solver class that only consider one level of importance for all tasks using QLD.
 *
 * It uses a linear quadratic program which is included in the xde framework.
 *
 * QLD solve the following problem:
 *
 * \f{align*}{
 *     \argmin{\x} &: \;  \frac{1}{2} \x\tp P \x + \vec{q}\tp \x \\
 *     & \A \x + \b \geqq \vec{0} \\
 *     & \x_{min} \leq \x \leq \x_{max}
 * \f}
 *
 * with
 * \f{align*}{
 *     \A \x + \b &\geqq \vec{0} & &\Leftrightarrow
 *     &\begin{bmatrix} CE \\ CI \end{bmatrix} \x + \begin{bmatrix} \vec{ce}_0 \\ \vec{ci}_0 \end{bmatrix}
 *          & \quad \begin{matrix} = \\ \geq \end{matrix} \quad \begin{bmatrix} \vec{0} \\ \vec{0} \end{bmatrix}
 * \f}
 */
class OneLevelSolverWithQLD: public OneLevelSolver
{
public:
    OneLevelSolverWithQLD();
    virtual ~OneLevelSolverWithQLD();


protected:

    // function in doPrepare
    virtual void doSolve();
    virtual void updateObjectiveEquations();
    virtual void updateConstraintEquations();


protected:
    Eigen::VectorXd _xl;
    Eigen::VectorXd _xu;

    Eigen::MatrixXd AandG;
    Eigen::VectorXd bandh;

    ocra::ObjQLD* _QLDsolver;


    MatrixMap MapP;
    VectorMap Mapq;

    MatrixMap MapAandG;
    VectorMap Mapbandh;

    VectorMap MapXsol;
    VectorMap MapXl;
    VectorMap MapXu;

};


/** \} */ // end group solver

}// namespace gocra

#endif
