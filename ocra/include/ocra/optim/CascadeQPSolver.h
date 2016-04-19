/** @file CascadeQPSolver.h
  * @brief Declaration file of the CascadeQPSolver class.
  *
  *   Copyright (C) 2016 ISIR/CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Antoine Hoarau
  *	@date 11/04/2016
  */

#ifndef _OCRABASE_CASCADE_QP_SOLVER_H_
#define _OCRABASE_CASCADE_QP_SOLVER_H_

// includes
#include "ocra/optim/Solver.h"
#include "ocra/optim/OneLevelSolver.h"
#include "ocra/optim/Constraint.h"
#include "ocra/optim/QuadraticFunction.h"
#include "ocra/optim/CascadeQPStructures.h"
#include <ocra/control/Tasks/Task.h>
#include <ocra/optim/SumOfLinearFunctions.h>
#include <ocra/optim/FunctionHelpers.h>
#include <ocra/optim/LinearFunction.h>
#include <ocra/control/ControlConstraint.h>
#include <ocra/control/Tasks/OneLevelTask.h>
#include <ocra/control/FullDynamicEquationFunction.h>

#include <memory>
#include <map>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace.
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems.
  */
namespace ocra
{
    
/* 
struct LevelConstraints
{
    std::vector<Function*> _exclusion_constraints;
    std::map<Function*,EqualZeroConstraintPtr<LinearFunction> > _allowed_constraints;
};
    
class HierarchyConstraints
{
    HierarchyConstraints(int level):_level(level){}
    std::map<int,LevelConstraints> _level_constraints;
private:
    const int _level;
};
*/
/** @class CascadeQPSolver
  *	@brief %CascadeQPSolver class.
  *
  * Hierarchical solver based on CascadeQP
  */
class CascadeQPSolver : public ocra::Solver
{
    static bool has_element(std::vector<const ocra::Function*>& v,const ocra::Function* e)
    {
        return std::find(v.begin(),v.end(),e) != v.end();
    }
    
public:
    DEFINE_CLASS_POINTER_TYPEDEFS(CascadeQPSolver)

    CascadeQPSolver(const std::string& _ctrlName,
                                 Model::Ptr _innerModel,
                                 OneLevelSolver::Ptr _levelSolver,
                                 bool _useReducedProblem);

    void addTask(Task::Ptr task);
    void addSolver(OneLevelSolver::Ptr solver,int level);
    OneLevelSolver::Ptr getSolver(int level);
    virtual std::string toString();
    const std::map<int,OneLevelSolver::Ptr >& getSolvers();
    void updateHierarchicalContraints(int level);
    int getNumberOfLevelsAbove(int current_level);
protected:
    virtual void doSolve(void);
    virtual void doPrepare(void);
    virtual void doConclude();
    virtual void printValuesAtSolution();
    void excludeObjective(int at_level, const ocra::GenericObjective& obj);
    struct StandardObjectivesAndConstraints;
    std::shared_ptr<StandardObjectivesAndConstraints> own_obj;
    std::vector<int> solverInitialized;
    std::map<int,std::shared_ptr<StandardObjectivesAndConstraints> > std_obj;
    OneLevelSolver::Ptr levelSolver;
    bool useReducedProblem;
    Model::Ptr innerModel;
    
private:
    // NOTE : For each level, it contains a list of the n-1 * m constraints ( Each level can have m tasks, so we need to add m constraints from that level)
    std::map<int, std::map<int,std::map<Function*,EqualZeroConstraintPtr<LinearFunction> > > > levelConstraints;
    std::map<int, std::vector<const Function*> > _exclusion_constraints;
    std::map<int,OneLevelSolver::Ptr > solvermap;
    std::map<int,std::vector<Task::Ptr> > taskmap;
    int highest_hierarchy_level;
};
}

#endif	//_OCRABASE_CASCADE_QP_SOLVER_H_
