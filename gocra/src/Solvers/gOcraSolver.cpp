/**
 * \file gOcraSolver.cpp
 * \author Joseph Salini
 *
 * \brief Implement the internal solver class that can be used in the gOcra controller.
 */

#include "gocra/Solvers/gOcraSolver.h"

#include <Eigen/SVD>

#include "ocra/control/ControlEnum.h"

#include "ocra/optim/MergedVariable.h"

using namespace gocra;



/** Initialize an abstract gOcra Solver.
 *
 */
gOcraSolver::gOcraSolver()
    : ocra::Solver()
    , ocra::NamedInstance("gOcra Solver")
{

}

/** Destructor
 *
 */
gOcraSolver::~gOcraSolver()
{

}

/** I don't really know, I suppose to print internal values when solution is found.
 *
 * Actually does nothing.
 */
void gOcraSolver::printValuesAtSolution()
{
    std::cout<<"TODO: 'printValuesAtSolution' method not implemented yet!"<<std::endl;
}

/** I don't really know, I support to write the problem as a string.
 *
 * Actually does nothing.
 */
std::string gOcraSolver::toString() const
{
    return "TODO: 'toString' method not implemented yet!";
}

/** Add a quadratic objective to the controller.
 *
 * \param obj The quadratic objective to add
 *
 * This objective is saved in an internal vector of objectives.
 */
void gOcraSolver::addObjective(ocra::QuadraticObjective& obj)
{
    internalAddObjective(obj);
    _objectives.push_back(&obj);
}

/** Remove a quadratic objective to the controller.
 *
 * \param obj The quadratic objective to remove
 *
 * This objective is removed from the internal vector of objectives.
 */
void gOcraSolver::removeObjective(ocra::QuadraticObjective& obj)
{
    internalRemoveObjective(obj);
    _objectives.erase(std::find(_objectives.begin(), _objectives.end(), &obj));

}

/** Add a linear constraint to the controller.
 *
 * \param constraint The linear constraint to add
 *
 * This constraint is saved in an internal vector of equality or inequality constraints, depending on the type of \a constraint.
 */
void gOcraSolver::addConstraint(ocra::LinearConstraint& constraint)
{
    internalAddConstraint(constraint);
    if(constraint.isEquality())
        _equalityConstraints.push_back(&constraint);
    else
        _inequalityConstraints.push_back(&constraint);
}

/** Removea linear constraint to the controller.
 *
 * \param constraint The linear constraint to remove
 */
void gOcraSolver::removeConstraint(ocra::LinearConstraint& constraint)
{
    internalRemoveConstraint(constraint);
    if(constraint.isEquality())
        _equalityConstraints.erase(std::find(_equalityConstraints.begin(), _equalityConstraints.end(), &constraint));
    else
        _inequalityConstraints.erase(std::find(_inequalityConstraints.begin(), _inequalityConstraints.end(), &constraint));
}




/** Prepare the basic matrices, which can be considered as the task \f$ T_0 \f$  because it minimizes the whole problem vector with very low coefficient.
 *
 */
void gOcraSolver::prepareMatrices()
{
    Xsolution.resize(n());
}

/** Do prepare, meaning prepare matrices before solving.
 *
 * It calls in this order:
 *
 *      - #prepareMatrices()
 *      - #updateObjectiveEquations()
 *      - #updateConstraintEquations()
 */
void gOcraSolver::doPrepare()
{
    prepareRecorder.initializeTime();

    prepareMatrices();
    updateObjectiveEquations();
    updateConstraintEquations();

    prepareRecorder.saveRelativeTime();
}

/** Do conclude, meaning what to do after solving.
 *
 * Actually does nothing.
 */
void gOcraSolver::doConclude()
{

}


/** Write information about solver performances in a string stream.
 *
 * \param outstream the output stream where to write the performances information
 * \param addCommaAtEnd If true, add a comma at the end of the stream. If false, it means that this is the end of the json file, nothing will be added after that, no comma is added.
 *
 * See gocra::GHCJTController::getPerformances() to know more. Here it saves:
 *
 *  - solver_prepare
 *  - solver_solve
 */
void gOcraSolver::writePerformanceInStream(std::ostream& outstream, bool addCommaAtEnd)
{
    prepareRecorder.writeInStream("solver_prepare", outstream, true);
    solveRecorder.writeInStream("solver_solve", outstream, addCommaAtEnd);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Reduce the set of equality constraints if it is not full rank.
 *
 * \param A The matrix associated to the set of equality constraints
 * \param b The vector associated to the set of equality constraints
 * \param Ar The matrix instance where will be saved the reduced version of \a A
 * \param br The vector instance where will be saved the reduced version of \a b
 * \param tolerance The tolerance of reduction
 *
 * To reduce the set of equality constraints, we make a SVD decomposition \f$ \A = U . S . V\tp \f$.
 * Then we check the dimension (denoted by \f$ r \f$) of the diagonal \f$ S \f$ where value are greater than the tolerance.
 *
 * The reduced set of task is then (noted in a python way):
 *
 * \f{align*}{
 *      \A_r &= S[0:r, :] V[:, 0:r] \tp
 *      & &
 *      & \b_r &= U[:, 0:r] \tp \b
 * \f}
 *
 */
void gOcraSolver::reduceConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::MatrixXd& Ar, Eigen::VectorXd& br, double tolerance)
{
    if (A.rows()>0)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfMat(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd S  = svdOfMat.singularValues();
        Eigen::MatrixXd Ur = svdOfMat.matrixU();
        Eigen::MatrixXd Vr = svdOfMat.matrixV();

        int r;
        for (r = 0; r<S.size(); r++)
        {
            if (S[r] <= tolerance)
                break;
        }

        Eigen::VectorXd Sr = S.segment(0,r);
        Ur.conservativeResize(Eigen::NoChange, r);
        Vr.conservativeResize(Eigen::NoChange, r);

        Ar = Sr.asDiagonal() * Vr.transpose();
        br = Ur.transpose() * b;
    }
    else
    {
        Ar = A;
        br = b;
    }
}
