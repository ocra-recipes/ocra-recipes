#include "ocra/optim/CascadeQPSolver.h"

#include <algorithm>
#include "ocra/utilities.h"
// #include <boost/range/adaptor/reversed.hpp>

namespace ocra
{
struct CascadeQPSolver::StandardObjectivesAndConstraints
{
    Model::Ptr          innerModel;
    // EQUALITY CONSTRAINT OF THE DYNAMIC EQUATION
    ocra::EqualZeroConstraintPtr< ocra::FullDynamicEquationFunction >      dynamicEquation;

    // MINIMIZATION TASK FOR WHOLE VARIABLE MINIMIZATION
    ocra::QuadraticFunction*     minDdqFunction;
    ocra::QuadraticFunction*     minTauFunction;
    ocra::FcQuadraticFunction*        minFcFunction;

    ObjectivePtr<ocra::QuadraticFunction>     minDdqObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minTauObjective;
    ObjectivePtr<ocra::QuadraticFunction>     minFcObjective;

    StandardObjectivesAndConstraints(Model::Ptr m)

        : innerModel(m)
        , dynamicEquation( new ocra::FullDynamicEquationFunction(*m) )

        , minDdqFunction(  new ocra::QuadraticFunction(m->getAccelerationVariable(), Eigen::MatrixXd::Identity(m->nbDofs(), m->nbDofs()), Eigen::VectorXd::Zero(m->nbDofs()), 0) )

        , minTauFunction(  new ocra::QuadraticFunction(m->getJointTorqueVariable(), Eigen::MatrixXd::Identity(m->getJointTorqueVariable().getSize(), m->getJointTorqueVariable().getSize()), Eigen::VectorXd::Zero(m->getJointTorqueVariable().getSize()), 0) )

         , minFcFunction(   new FcQuadraticFunction(m->getModelContacts().getContactForcesVariable()) )

    {
        minDdqObjective.set(minDdqFunction);
        minTauObjective.set(minTauFunction);
        minFcObjective.set(minFcFunction);
        minDdqObjective.getObjective().setWeight(1e-7);
        minTauObjective.getObjective().setWeight(1e-8);
        minFcObjective.getObjective().setWeight(1e-9);
        dynamicEquation.getFunction().takeIntoAccountGravity(true);

    }

    ~StandardObjectivesAndConstraints()
    {
    }

};
    
    
    
CascadeQPSolver::CascadeQPSolver(const std::string& _ctrlName,
                                 Model::Ptr _innerModel,
                                 OneLevelSolver::Ptr _levelSolver,
                                 bool _useReducedProblem):
    ocra::Solver(),
    ocra::NamedInstance("CascadeQPSolver"),
    levelSolver(_levelSolver),
    innerModel(_innerModel),
    useReducedProblem(_useReducedProblem),
    own_obj(new StandardObjectivesAndConstraints(_innerModel))
{
    
    internalAddConstraint(own_obj->dynamicEquation.getConstraint());
    internalRemoveObjective(own_obj->minDdqObjective);
    internalRemoveObjective(own_obj->minTauObjective);
    internalRemoveObjective(own_obj->minFcObjective);
    //std::cout << "CascadeQPSolver constructor" << std::endl;
}

void CascadeQPSolver::updateHierarchicalContraints(int level)
{
    //std::cout << "[[[[[[--------- CascadeQPSolver Update for level "<<level<<" -------- ]]]] "<< std::endl;

    for(auto m : solvermap)
    {
        //std::cout << "[---- Level "<<m.first<<" ----]"<< std::endl;
        if(level == m.first) return;
        auto solver = m.second;
        Eigen::VectorXd solution = solver->getLastResult().solution;
        //std::cout << "-Processing level "<<m.first<<" variable size : "<<solver->n()<<"\n\n last solution : "<<solution.transpose()<< std::endl;
        _P.setZero(solver->n(), solver->n());
        _q.setZero(solver->n());
        solver->getProblemVariable();

        for(auto& obj : solver->getObjectives())
        {
            auto& f = obj->getFunction();
            ocra::utils::addCompressed2d(f.getPi(), _P, solver->findMapping(obj->getVariable()), obj->getWeight());
        }

        auto & f = levelConstraints[level][m.first].getFunction();
        //std::cout << "-_P * solution:\n\n"<<-_P * solution<<std::endl;
        //std::cout << "P:\n\n"<<_P<<std::endl;
        f.changeb( -_P * solution );
        f.changeA(  _P );
    }
    
}
int CascadeQPSolver::getNumberOfLevelsAbove(int current_level)
{
    int n = 0;
    for(auto m : solvermap)
        if(m.first == current_level)
            break;
        else
            n++;
    return n;
}

void CascadeQPSolver::addHierarchicalContraintsToEachLevels()
{
    //std::cout <<"CascadeQPSolver::addHierarchicalContraintsToEachLevels("<<")"<<std::endl;


    for(auto m : solvermap)
    {
        auto current_solver = m.second;
        int current_level = m.first;

        int n_levels_above = getNumberOfLevelsAbove(current_level);

        //std::cout << "At level "<<current_level<<" we have "<<n_levels_above<<" levels above, so we need the same amount of constraints, we now have "<<levelConstraints[current_level].size()<<std::endl;

        for(auto a : solvermap)
        {
            auto solver_above = a.second;
            int solver_above_idx = a.first;

            if(solver_above_idx == current_level)
                break;

            auto& var = solver_above->getProblemVariable();

            Eigen::MatrixXd P_tmp(var.getSize(),var.getSize());
            Eigen::VectorXd q_tmp(var.getSize());

            P_tmp.setZero();
            q_tmp.setZero();
            try {
                if(levelConstraints[current_level].size() != n_levels_above)
                {
                    //std::cout <<"[lc size:"<<levelConstraints[current_level].size()<<"] At level "<<current_level<<", I'm adding constrains form level "<<solver_above_idx<<"; constraint of size "<<var.getSize()<<"-"<<solver_above->n()<<std::endl;
                    levelConstraints[current_level].push_back(EqualZeroConstraintPtr<LinearFunction>(new LinearFunction(var,P_tmp,q_tmp)));
                    getSolver(current_level)->addConstraint(levelConstraints[current_level][levelConstraints[current_level].size() -1].getConstraint());
                } else {
                    //std::cout << "levelConstraints[current_level].size() == n_levels_above not adding new constraints"<<std::endl;
                }
            }
            catch(const std::runtime_error & e) {
                //std::cout <<"Constraint already in use"<<std::endl;
            }
        }

        for(const auto& lc : levelConstraints)
        {
            //std::cout << "Level "<<lc.first<<" , we have "<<lc.second.size()<<" constraints registered"<<std::endl;
        }
    }

}

void CascadeQPSolver::doSolve(void)
{
    //std::cout << "CascadeQPSolver::doSolve" << std::endl;
    int i=0;
    std::vector<OptimizationResult> solution;
    for(auto& m : solvermap)
    {
        updateHierarchicalContraints(m.first);
        //std::cout << "Solving" << std::endl;
        _result = m.second->solve();
        //std::cout << "RESULT "<<m.first<<" : "<< _result.solution.transpose() << std::endl;
        //std::cout << "Solved" << std::endl;
    }
    //std::cout << "FINAL RESULT : "<< _result.solution.transpose() << std::endl;
}


void CascadeQPSolver::doPrepare(void)
{
    //std::cout << "CascadeQPSolver::doPrepare" << std::endl;
}


void CascadeQPSolver::doConclude()
{
    //std::cout << "CascadeQPSolver::doConclude" << std::endl;
}


std::string CascadeQPSolver::toString()
{

}
const std::map< int, OneLevelSolver::Ptr >& CascadeQPSolver::getSolvers()
{
    return this->solvermap;
}

void CascadeQPSolver::addSolver(OneLevelSolver::Ptr solver, int level)
{

    if(solvermap.find(level) == solvermap.end())
    {
        bool solverexists = false;
        for( auto sm : solvermap)
            if(sm.second.get() == solver.get())
                solverexists = true;
        if(solverexists)
        {
            //std::cout << "[Warning] You are tying to add al already existing solver, I'm cloning for you a new " << solver->getName() << std::endl;
            this->solvermap[level] = solver->clone();
        } else {
            //std::cout << "Adding solver " << solver->getName() << " at level "<<level<< std::endl;
            this->solvermap[level] = solver;
        }
    } else {
        //std::cout << "A Solver is already present at level " << level<< " : "<<solvermap[level]->getName() << std::endl;
    }
}

OneLevelSolver::Ptr CascadeQPSolver::getSolver(int level)
{
    return solvermap.at(level);
}

void CascadeQPSolver::addTask(Task::Ptr task)
{
    this->taskmap[task->getHierarchyLevel()].push_back(task);
        
    if(std::find(solverInitialized.begin(),solverInitialized.end(),task->getHierarchyLevel()) == solverInitialized.end())
    {
        solverInitialized.push_back(task->getHierarchyLevel());
        std_obj[task->getHierarchyLevel()] = std::make_shared<StandardObjectivesAndConstraints>(innerModel);
        auto new_solver = levelSolver->clone();
        new_solver->addConstraint(std_obj[task->getHierarchyLevel()]->dynamicEquation.getConstraint());
        new_solver->addObjective(std_obj[task->getHierarchyLevel()]->minDdqObjective);
        new_solver->addObjective(std_obj[task->getHierarchyLevel()]->minTauObjective);
        new_solver->addObjective(std_obj[task->getHierarchyLevel()]->minFcObjective);  
        
        this->addSolver(new_solver,task->getHierarchyLevel());
    }
    try {
        auto ctask = std::dynamic_pointer_cast<ocra::OneLevelTask>(task);
        ctask->connectToController(this->getSolver(task->getHierarchyLevel()), 
                                   std_obj[task->getHierarchyLevel()]->dynamicEquation, 
                                   useReducedProblem);
        this->addHierarchicalContraintsToEachLevels();
        
    }
    catch(const std::exception & e) {
        std::cerr << e.what() ;
        throw std::runtime_error("[HocraController::doAddTask] cannot add task to controller (wrong type)");
    }
}

void CascadeQPSolver::printValuesAtSolution()
{

}


} // Namespace ocra


