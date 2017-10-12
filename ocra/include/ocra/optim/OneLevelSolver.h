/**
 * \file OneLevelSolver.h
 * \author Joseph Salini
 *
 * \brief Define the internal solver class that can be used in the wOcra controller.
 *
 * Here, only an abstract class is defined.
 */

#ifndef __ONE_LEVEL_SOLVER_H__
#define __ONE_LEVEL_SOLVER_H__

#include "ocra/optim/Solver.h"
#include "ocra/optim/Objective.h"
#include "ocra/optim/QuadraticFunction.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/MergedVariable.h"
#include "ocra/optim/ObjQLD.h"
#include "ocra/control/ControlEnum.h"
#include "ocra/control/Model.h"
#include <ocra/util/ErrorsHelper.h>

#include "ocra/optim/QuadProg++.h"

#include <string>
#include <Eigen/SVD>

//#ifdef USE_QPOASES

#include <qpOASES.hpp>
//#endif

namespace ocra
{

/** \addtogroup solver
 * \{
 */

//typedef ocra::Objective<ocra::SquaredLinearFunction>  SquaredLinearObjective;
//typedef ocra::Objective<ocra::QuadraticFunction>  SquaredLinearObjective;

typedef Eigen::Map<Eigen::MatrixXd> MatrixMap;
typedef Eigen::Map<Eigen::VectorXd> VectorMap;

typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdRm;

/** \brief A generic abstract class the solvers that can be used in the wOcra Controller.
 *
 * It is based on quadratic solvers.
 *
 * To get a concrete implementation of wOcra solvers, you should call:
 *
 *      - OneLevelSolver if we consider that all the tasks registered have the same level of importance (but not necessarily the same weights)
 *      - HierarchySolver if we consider that tasks registered can have different level of importance
 */
class OneLevelSolver: public ocra::Solver
{
    DEFINE_CLASS_POINTER_TYPEDEFS(OneLevelSolver)
    public: using ObjectiveType = ocra::QuadraticObjective;
public:
    OneLevelSolver();
    virtual ~OneLevelSolver();

    virtual void printValuesAtSolution();

//    virtual void addObjective(SquaredLinearObjective& obj);
//    virtual void removeObjective(SquaredLinearObjective& obj);
//    virtual void setObjectiveLevel(SquaredLinearObjective& obj, int level)  = 0;

    void addObjective(ObjectiveType& obj)
    {
        internalAddObjective(obj);
        _objectives.push_back(&obj);
    }
    void removeObjective(ObjectiveType& obj)
    {
        internalRemoveObjective(obj);
        _objectives.erase(std::find(_objectives.begin(), _objectives.end(), &obj));
    }

    void addConstraint(ocra::LinearConstraint& constraint);
    void removeConstraint(ocra::LinearConstraint& constraint);

    void writePerformanceInStream(std::ostream& myOstream, bool addCommaAtEnd);
    virtual void setObjectiveLevel(ocra::GenericObjective& obj, int level){};

    virtual std::string toString();
    /** retuns a pointer to a new instance, very usefull in Hocra */
    virtual OneLevelSolver::Ptr clone() const = 0;
    const std::vector<ObjectiveType*>& getObjectives(){return _objectives;}
    const Eigen::MatrixXd& getQuadraticMatrix(){return _C;}
    const Eigen::VectorXd& getQuadraticVector(){return _d;}

    void saveOptimizationData(const std::string& dir_path);

protected:

    virtual void doPrepare();
    virtual void doConclude();
    virtual void doSolve() = 0;

    // function in doPrepare
    virtual void prepareMatrices();
    virtual void updateObjectiveEquations()  = 0;
    virtual void updateConstraintEquations() = 0;

    //function in doSolve
    void reduceConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::MatrixXd& Ar, Eigen::VectorXd& br, double tolerance=1e-6);


    std::vector<ObjectiveType*>    _objectives;
//     std::vector<ocra::QuadraticObjective*>    _objectives;

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
    DEFINE_CLASS_POINTER_TYPEDEFS(OneLevelSolverWithQuadProg)
public:
    virtual OneLevelSolver::Ptr clone() const{ return std::make_shared<OneLevelSolverWithQuadProg>();}
    OneLevelSolverWithQuadProg();
    virtual ~OneLevelSolverWithQuadProg();

protected:

    // function in doPrepare
    virtual void doSolve();
    virtual void updateObjectiveEquations();
    virtual void updateConstraintEquations();

};

//#ifdef USE_QPOASES
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Solver class that only consider one level of importance for all tasks using QPOASES.
 *
 * It uses a linear quadratic program which comes from https://projects.coin-or.org/qpOASES .
 *
 * QPOASES solve the following problem:
 *
 * \f{align*}{
 *     \argmin{\x} &: \;  \frac{1}{2} \x\tp G \x + \vec{g}_0\tp \x \\
 *     & CE \x + \vec{ce}_0 =    \vec{0} \\
 *     & CI \x + \vec{ci}_0 \geq \vec{0}
 * \f}
 */
class OneLevelSolverWithQPOASES: public OneLevelSolver
{
    DEFINE_CLASS_POINTER_TYPEDEFS(OneLevelSolverWithQPOASES)
public:
    virtual OneLevelSolver::Ptr clone() const{ return std::make_shared<OneLevelSolverWithQPOASES>();}
public:
    OneLevelSolverWithQPOASES();
    virtual ~OneLevelSolverWithQPOASES(){};

protected:
    std::unique_ptr<qpOASES::SQProblem> sqp_prob;
    qpOASES::Options sqp_options;
    //struct qp{
        std::vector<qpOASES::real_t> H;
        qpOASES::real_t* g;
        qpOASES::real_t* lb;
        qpOASES::real_t* ub;
        qpOASES::real_t* A;
        qpOASES::real_t* lbA;
        qpOASES::real_t* ubA;
    //};

    Eigen::VectorXd _xl;
    Eigen::VectorXd _xu;

    MatrixXdRm _AandG,_C_row_major;
    Eigen::MatrixXd _RegTerm;
    Eigen::VectorXd _lbAandG,_ubAandG;
    Eigen::VectorXd _lbA,_lbG,_ubA,_ubG;

    int _nWSR_every_run,nWSR;
    // function in doPrepare
    virtual void doSolve();
    virtual void updateObjectiveEquations();
    virtual void updateConstraintEquations();
    static ocra::eReturnInfo toOcraRetValue(const qpOASES::returnValue& ret);

};
//#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Solver class that only consider one level of importance for all tasks using QLD.
 *
 * It uses a linear quadratic program which is included in the ocra framework.
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
    DEFINE_CLASS_POINTER_TYPEDEFS(OneLevelSolverWithQLD)
public:
    virtual OneLevelSolver::Ptr clone() const{ return std::make_shared<OneLevelSolverWithQLD>();}
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

}// namespace ocra

#endif
