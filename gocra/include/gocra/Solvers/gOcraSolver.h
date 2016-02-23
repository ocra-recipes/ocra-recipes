/**
 * \file gOcraSolver.h
 * \author Joseph Salini
 *
 * \brief Define the internal solver class that can be used in the GHCJT controller.
 *
 * Here, only an abstract class is defined.
 */

#ifndef __GOCRASOLVER_H__
#define __GOCRASOLVER_H__

#include "ocra/optim/Solver.h"
#include "ocra/optim/Objective.h"
#include "ocra/optim/QuadraticFunction.h"
#include "ocra/optim/SquaredLinearFunction.h"

#include "ocra/control/Model.h"

#include <string>


#include "gocra/Performances.h"


namespace gocra
{

/** \addtogroup solver
 * \{
 */

//typedef ocra::Objective<ocra::SquaredLinearFunction>  SquaredLinearObjective;
//typedef ocra::Objective<ocra::QuadraticFunction>  QuadraticObjective;

typedef Eigen::Map<Eigen::MatrixXd> MatrixMap;
typedef Eigen::Map<Eigen::VectorXd> VectorMap;

/** \brief A generic abstract class the solvers that can be used in the GHCJT Controller.
 *
 * It is based on quadratic solvers.
 *
 * To get a concrete implementation of gOcra solvers, you should call:
 *
 *      - gocra::OneLevelSolver if we consider that all the tasks registered have the same level of importance (but not necessarily the same weights)
 *      - gocra::HierarchySolver if we consider that tasks registered can have different level of importance
 */
class gOcraSolver: public ocra::Solver
{
public:
    gOcraSolver();
    virtual ~gOcraSolver();

    virtual void printValuesAtSolution();
    virtual std::string toString() const;

//    virtual void addObjective(SquaredLinearObjective& obj);
//    virtual void removeObjective(SquaredLinearObjective& obj);
//    virtual void setObjectiveLevel(SquaredLinearObjective& obj, int level)  = 0;

    virtual void addObjective(ocra::QuadraticObjective& obj);
    virtual void removeObjective(ocra::QuadraticObjective& obj);
    virtual void setObjectiveLevel(ocra::QuadraticObjective& obj, int level)  = 0;

    void addConstraint(ocra::LinearConstraint& constraint);
    void removeConstraint(ocra::LinearConstraint& constraint);

    void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd);


protected:

    virtual void doPrepare();
    virtual void doSolve()  = 0;
    virtual void doConclude();

    // function in doPrepare
    virtual void prepareMatrices();
    virtual void updateObjectiveEquations()  = 0;
    virtual void updateConstraintEquations() = 0;

    //function in doSolve
    void reduceConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::MatrixXd& Ar, Eigen::VectorXd& br, double tolerance=1e-6);

protected:

    //std::vector<SquaredLinearObjective*>    _objectives;
    std::vector<ocra::QuadraticObjective*>    _objectives;

    std::vector<ocra::LinearConstraint*> _equalityConstraints;         //< set of constraints
    Eigen::MatrixXd _A;
    Eigen::VectorXd _b;
    // for equality constraint, which will grow over levels, or when they are reduced
    Eigen::MatrixXd _Atotal;
    Eigen::VectorXd _btotal;
    int ne;

    std::vector<ocra::LinearConstraint*> _inequalityConstraints;         //< set of constraints
    Eigen::MatrixXd _G;
    Eigen::VectorXd _h;
    int ni;

    Eigen::VectorXd Xsolution;


    PerformanceRecorder prepareRecorder;
    PerformanceRecorder solveRecorder;

};


/** \} */ // end group solver

}// namespace gocra

#endif
