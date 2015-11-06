/**
 * \file OneLevelSolver.cpp
 * \author Joseph Salini
 *
 * \brief Implement concrete solvers class for \b one-level LQP solver.
 */

#include "gocra/Solvers/OneLevelSolver.h"

#include <Eigen/SVD>

#include "ocra/control/ControlEnum.h"

#include "ocra/optim/MergedVariable.h"

using namespace gocra;

/** Constructor of the abstract one level solver.
 *
 * \param m The Model of the robot
 */
OneLevelSolver::OneLevelSolver()
    : gOcraSolver()
    , ocra::NamedInstance("One Level Solver")
{

}

/** Destructor
 *
 */
OneLevelSolver::~OneLevelSolver()
{

}

/** Set the level of a particular objective registered  in the solver.
 *
 * \param obj The objective instance with a new level
 * \param level the new level
 *
 * Actually, as this solver considers that all tasks have the same level, this function does nothing.
 */
void OneLevelSolver::setObjectiveLevel(ocra::QuadraticObjective& obj, int level)
{
    //std::cout<<"[gocra::OneLevelSolver::setObjectiveLevel] Warning: solver doesn't take into account level for task.\n";
}



//#include <sstream>

/** I don't really know, I support to write the problem as a string.
 *
 * Actually does nothing.
 */
std::string OneLevelSolver::toString() const
{
    std::stringstream ss;
    ss<<"-----------------------------------------------\n";
    ss<<"probvar:\n";
    getProblemVariable().printSubTree(3,ss);
    ss<<"C:\n"<<_C<<"\n";
    ss<<"d:\n"<<-_d.transpose()<<"\n";
    ss<<"\n";
    ss<<"A:\n"<<_Atotal<<"\n";
    ss<<"b:\n"<<-_btotal.transpose()<<"\n";
    ss<<"\n";
    ss<<"G:\n"<<_G<<"\n";
    ss<<"h:\n"<<-_h.transpose()<<"\n";

    return ss.str();

}




///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Hierarchy Solver with QuadProg sub-Solver
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Instanciate a concrete one level solver with Quadprog++.
 *
 * \param m The Model of the robot
 */
OneLevelSolverWithQuadProg::OneLevelSolverWithQuadProg()
    : ocra::NamedInstance("One Level Solver with QuadProg++ subSolver")
    , OneLevelSolver()
{

}

/** Destructor
 *
 */
OneLevelSolverWithQuadProg::~OneLevelSolverWithQuadProg()
{
    std::cout<<getProblemVariable().getSize()<<std::endl;
}

/** Update the objective function, by considering all the active registered tasks.
 *
 */
void OneLevelSolverWithQuadProg::updateObjectiveEquations()
{
//    _C = _Pbase;
    _C.setZero(n(), n());
    _d.setZero(n());

    for(int i=0; i<_objectives.size(); i++)
    {

        ocra::QuadraticFunction& obj     = _objectives[i]->getFunction();
        double weight                   = _objectives[i]->getWeight();
        const std::vector<int>& objMap  = findMapping(obj.getVariable());

        ocra::utils::addCompressed2d(   obj.getPi(), _C, objMap, weight);
        ocra::utils::addCompressedByRow(obj.getqi(), _d, objMap, weight);

    }

}

/** Update the constraint set, by considering all the active ones.
 *
 */
void OneLevelSolverWithQuadProg::updateConstraintEquations()
{
    // UPDATE EQUALITY CONSTRAINTS
    int ne = 0;
    for (int i=0; i<_equalityConstraints.size(); ++i)
    {
        ne += _equalityConstraints[i]->getDimension();
    }

    _A.setZero(ne, n());
    _b.setZero(ne);

    int idx = 0;
    for (int i=0; i<_equalityConstraints.size(); ++i)
    {
        ocra::LinearConstraint* cstr = _equalityConstraints[i];
        int dim = cstr->getDimension();

        if (dim > 0)
        {
            Eigen::Block<Eigen::MatrixXd> _A_block = _A.block(idx, 0, dim, n());
            Eigen::VectorBlock<Eigen::VectorXd> _b_segment = _b.segment(idx, dim);

            Eigen::VectorXd v; // ?? is it useless ??
            ocra::utils::convert(*cstr, findMapping(cstr->getVariable()), ocra::CSTR_PLUS_EQUAL, _A_block, _b_segment, v);

            idx += dim;
        }

    }

    reduceConstraints(_A, _b, _Atotal, _btotal);



    // UPDATE INEQUALITY CONSTRAINTS
    int ni = 0;
    for (int i=0; i<_inequalityConstraints.size(); ++i)
    {
        ni += _inequalityConstraints[i]->getDimension();
    }

    _G.setZero(ni, n());
    _h.setZero(ni);

    idx = 0;
    for (int i=0; i<_inequalityConstraints.size(); ++i)
    {
        ocra::LinearConstraint* cstr = _inequalityConstraints[i];
        int dim = cstr->getDimension();

        if (dim > 0)
        {
            Eigen::Block<Eigen::MatrixXd > _G_block = _G.block(idx, 0, dim, n());
            Eigen::VectorBlock<Eigen::VectorXd> _h_segment = _h.segment(idx, dim);
            Eigen::VectorXd v;  // ?? is it useless ??
            ocra::utils::convert(*cstr, findMapping(cstr->getVariable()), ocra::CSTR_PLUS_GREATER, _G_block, _h_segment, v);

            idx += dim;
        }

    }
}


/** Do the call Quadprog++ to find a solution of the problem.
 *
 */
void OneLevelSolverWithQuadProg::doSolve()
{
    solveRecorder.initializeTime();

    QuadProgPP::solve_quadprog(_C, -_d, _Atotal, _btotal, _G, _h, Xsolution);
    _result.solution = Xsolution;
    _result.info = ocra::RETURN_SUCCESS; //TODO: should be dfined through the result of the QuadProgPP::solve_quadprog

    solveRecorder.saveRelativeTime();
}









///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Instanciate a concrete one level solver with QLD.
 *
 * \param m The Model of the robot
 */
OneLevelSolverWithQLD::OneLevelSolverWithQLD()
    : ocra::NamedInstance("One Level Solver with QLD subSolver")
    , OneLevelSolver()
    , _QLDsolver( new ocra::ObjQLD() )
    , MapP(NULL, 0,0)
    , Mapq(NULL,0)
    , MapAandG(NULL,0,0)
    , Mapbandh(NULL,0)
    , MapXsol(NULL,0)
    , MapXl(NULL,0)
    , MapXu(NULL,0)
{

}

/** Destructor
 *
 */
OneLevelSolverWithQLD::~OneLevelSolverWithQLD()
{

}



/** Update the objective function, by considering all the active registered tasks.
 *
 */
void OneLevelSolverWithQLD::updateObjectiveEquations()
{
//    _C = _Pbase;
    _C.setZero(n(), n());
    _d.setZero(n());

    if (MapP.size() != _C.size())
    {
        new (&MapP) MatrixMap(_C.data(), _C.rows(), _C.cols());
        new (&Mapq) VectorMap(_d.data(), _d.size());
    }

    for(int i=0; i<_objectives.size(); i++)
    {

        ocra::QuadraticFunction& obj     = _objectives[i]->getFunction();
        double weight                   = _objectives[i]->getWeight();
        const std::vector<int>& objMap  = findMapping(obj.getVariable());

        ocra::utils::addCompressed2d(   obj.getPi(), _C, objMap, weight);
        ocra::utils::addCompressedByRow(obj.getqi(), _d, objMap, weight);

    }

}

/** Update the constraint set, by considering all the active ones.
 *
 */
void OneLevelSolverWithQLD::updateConstraintEquations()
{
    // UPDATE EQUALITY CONSTRAINTS
    int ne = 0;
    for (int i=0; i<_equalityConstraints.size(); ++i)
    {
        ne += _equalityConstraints[i]->getDimension();
    }

    _A.setZero(ne, n());
    _b.setZero(ne);

    int idx = 0;
    for (int i=0; i<_equalityConstraints.size(); ++i)
    {
        ocra::LinearConstraint* cstr = _equalityConstraints[i];
        int dim = cstr->getDimension();

        if (dim > 0)
        {
            Eigen::Block<Eigen::MatrixXd> _A_block = _A.block(idx, 0, dim, n());
            Eigen::VectorBlock<Eigen::VectorXd> _b_segment = _b.segment(idx, dim);

            Eigen::VectorXd v; // ?? is it useless ??
            ocra::utils::convert(*cstr, findMapping(cstr->getVariable()), ocra::CSTR_PLUS_EQUAL, _A_block, _b_segment, v);

            idx += dim;
        }

    }

    reduceConstraints(_A, _b, _Atotal, _btotal);



    // UPDATE INEQUALITY CONSTRAINTS
    int ni = 0;
    for (int i=0; i<_inequalityConstraints.size(); ++i)
    {
        ni += _inequalityConstraints[i]->getDimension();
    }

    _G.setZero(ni, n());
    _h.setZero(ni);

    idx = 0;
    for (int i=0; i<_inequalityConstraints.size(); ++i)
    {
        ocra::LinearConstraint* cstr = _inequalityConstraints[i];
        int dim = cstr->getDimension();

        if (dim > 0)
        {
            Eigen::Block<Eigen::MatrixXd > _G_block = _G.block(idx, 0, dim, n());
            Eigen::VectorBlock<Eigen::VectorXd> _h_segment = _h.segment(idx, dim);
            Eigen::VectorXd v;  // ?? is it useless ??
            ocra::utils::convert(*cstr, findMapping(cstr->getVariable()), ocra::CSTR_PLUS_GREATER, _G_block, _h_segment, v);

            idx += dim;
        }

    }

    if ( ( AandG.rows() != _Atotal.rows()+_G.rows() ) || ( AandG.cols() != _Atotal.cols() ) )
    {
        AandG.resize(_Atotal.rows()+_G.rows(), n());
        bandh.resize(_btotal.size()+_h.size());
        new (&MapAandG) MatrixMap(AandG.data(), AandG.rows(), AandG.cols());
        new (&Mapbandh) VectorMap(bandh.data(), bandh.size());
    }
    AandG.topRows(_Atotal.rows())   = _Atotal;
    AandG.bottomRows(_G.rows())     = _G;
    bandh.head(_btotal.size())      = _btotal;
    bandh.tail(_h.size())           = _h;

    if (_xl.size() != n())
    {
        _xl = - 1e10 * Eigen::VectorXd::Ones(n()); // X lower bound
        _xu =   1e10 * Eigen::VectorXd::Ones(n()); // X upper bound
        new (&MapXl)    VectorMap(_xl.data(), _xl.size());
        new (&MapXu)    VectorMap(_xu.data(), _xu.size());
        new (&MapXsol)  VectorMap(Xsolution.data(), Xsolution.size()); // if bounds have changed, so has Xsolution
    }
}


/** Do the call QLD to find a solution of the problem.
 *
 */
void OneLevelSolverWithQLD::doSolve()
{
    solveRecorder.initializeTime();


//    if (MapXsol.size() != Xsolution.size())                               // this test is done in #updateConstraintEquations()
//        new (&MapXsol) VectorMap(Xsolution.data(), Xsolution.size());

    _d = -_d; // this is to set -Mapq

    int res = _QLDsolver->solve(MapP, Mapq, MapAandG, Mapbandh, _Atotal.rows(), MapXsol, MapXl, MapXu, false);

    _result.solution = MapXsol;

    _result.info = ocra::RETURN_SUCCESS; //TODO: should be dfined through the result of the QuadProgPP::solve_quadprog

    solveRecorder.saveRelativeTime();
}

