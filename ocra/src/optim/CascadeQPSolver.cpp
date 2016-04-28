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

        , minDdqFunction(  new ocra::SquaredLinearFunction( new LinearFunction(m->getAccelerationVariable(), Eigen::MatrixXd::Identity(m->nbDofs(), m->nbDofs()), Eigen::VectorXd::Zero(m->nbDofs()))))
        , minTauFunction(  new ocra::SquaredLinearFunction(new LinearFunction(m->getJointTorqueVariable(), 
                                                                              Eigen::MatrixXd::Identity(m->getJointTorqueVariable().getSize(), 
                                                                                                        m->getJointTorqueVariable().getSize()), 
                                                                              Eigen::VectorXd::Zero(m->getJointTorqueVariable().getSize())) ))

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
    internalAddObjective(own_obj->minDdqObjective);
    internalAddObjective(own_obj->minTauObjective);
//     internalAddObjective(own_obj->minFcObjective);
    
    //std::cout  << "CascadeQPSolver constructor WITH "<<this->getProblemVariable().getSize()<<" VARIABLES " << std::endl;
}

void CascadeQPSolver::updateHierarchicalContraints(int level)
{
    //std::cout  << "[[[[[[--------- CascadeQPSolver Update for level "<<level<<" -------- ]]]] "<< std::endl;

    for(auto m : solvermap)
    {
        if(level == m.first) return;
        //std::cout  << "---------\n Updating constraints from level "<<m.first<< std::endl;
        auto solver = m.second;
        Eigen::VectorXd full_solution = solver->getLastResult().solution;
        //std::cout  << "full_solution:"<<full_solution.transpose()<<std::endl;
        
        for(auto obj : solver->getObjectives())
        {

            auto& f_obj = static_cast<SquaredLinearObjective*>(obj)->getFunction().getFunction();
            
            if(has_element(_exclusion_constraints[m.first],&obj->getFunction()))
            {
                //std::cout  << "Function "<<&obj->getFunction()<<" is excluded, passing.."<<std::endl;
                continue;
            }
            
            auto& var = f_obj.getVariable();
            
            //std::cout  << "f_obj.getA():\n\n"<<f_obj.getA()<<std::endl;
            
            const std::vector<int>& mapping = solver->findMapping(obj->getVariable());
            
            //std::cout  << "Var size :"<<var.getSize()<<std::endl;
            
            Eigen::VectorXd minus_solution(var.getSize());
            for(int i=0;i<mapping.size();i++)
                minus_solution[i] = - full_solution[mapping[i]];
            
            
            //std::cout  << "     solution:"<<minus_solution.transpose()<<std::endl;
            
            if(levelConstraints[level][m.first].find(&f_obj) == levelConstraints[level][m.first].end())
            {
                //std::cout  << "Adding constaints for function "<<&f_obj<<std::endl;
                levelConstraints[level][m.first][&f_obj] = EqualZeroConstraintPtr<LinearFunction>(new LinearFunction(var,f_obj.getA(),f_obj.getb()));
            
                getSolver(level)->addConstraint(levelConstraints[level][m.first][&f_obj].getConstraint());
                
            }else{
                //std::cout  << "Function "<<&f_obj<<" already exists"<<std::endl;
            }

            auto& f = levelConstraints[level][m.first][&f_obj].getFunction();
            f.changeb( f_obj.getA() * minus_solution );
            f.changeA( f_obj.getA() );
        }
        
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

void CascadeQPSolver::doSolve(void)
{
    //std::cout  << "CascadeQPSolver::doSolve" << std::endl;
    OptimizationResult tmp_res;
    for(auto m : solvermap)
    {
        
        updateHierarchicalContraints(m.first);
        
        //std::cout  << "Solving" << std::endl;
        
        tmp_res = m.second->solve();
        if(tmp_res.info == ocra::RETURN_SUCCESS)
            _result = tmp_res;
        //std::cout  << "RESULT "<<m.first<<" : "<< _result.solution.transpose() << std::endl;
        //std::cout  << "Solved" << std::endl;
    }
    
    //std::cout  << "FINAL RESULT : "<< _result.solution.transpose() << std::endl;
}


void CascadeQPSolver::doPrepare(void)
{
    //std::cout  << "CascadeQPSolver::doPrepare" << std::endl;
}


void CascadeQPSolver::doConclude()
{
    //std::cout  << "CascadeQPSolver::doConclude" << std::endl;
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
            //std::cout  << "[Warning] You are tying to add al already existing solver, I'm cloning for you a new " << solver->getName() << std::endl;
            this->solvermap[level] = solver->clone();
        } else {
            //std::cout  << "Adding solver " << solver->getName() << " at level "<<level<< std::endl;
            this->solvermap[level] = solver;
        }
    } else {
        //std::cout  << "A Solver is already present at level " << level<< " : "<<solvermap[level]->getName() << std::endl;
    }
//     int current_highest_hierarchy_level = solvermap.begin()->first;
//     if(solvermap.size() > 1 &&
//         current_highest_hierarchy_level != highest_hierarchy_level)
//     {
//         // Remove all the regularisation objectives
//         // As the lower levels will have them as constraints
//         solver->removeObjective(std_obj[level]->minDdqObjective);
//         solver->removeObjective(std_obj[level]->minTauObjective);        
//         
//     }
}

OneLevelSolver::Ptr CascadeQPSolver::getSolver(int level)
{
    return solvermap.at(level);
}
void CascadeQPSolver::excludeObjective(int at_level, const ocra::GenericObjective & obj)
{
    
        if(_exclusion_constraints.find(at_level) == _exclusion_constraints.end() ||
            has_element(_exclusion_constraints[at_level],&obj.getFunction()) == false)
        {
            //std::cout  << "Function "<<&obj.getFunction()<<" at level "<<at_level<<" is EXCLUDED"<<std::endl;
            _exclusion_constraints[at_level].push_back(&obj.getFunction());
        }else
        {
            //std::cout  << "Function "<<&obj.getFunction()<<" is already in the exclusion vector"<<std::endl;
        }
}

void CascadeQPSolver::addTask(Task::Ptr task)
{
    const int level = task->getHierarchyLevel();
    if(task->isActiveAsObjective())
        this->taskmap[level].push_back(task);
    else if(task->isActiveAsConstraint())
        for(auto& tm : taskmap)
            this->taskmap[level].push_back(task);
    
    if(std::find(solverInitialized.begin(),solverInitialized.end(),level) == solverInitialized.end())
    {
        //std::cout  << "Initializing constraints and regulation terms at level "<<level << std::endl;
        solverInitialized.push_back(level);
        std_obj[level] = std::make_shared<StandardObjectivesAndConstraints>(innerModel);
        auto new_solver = levelSolver->clone();
        
        new_solver->addConstraint(std_obj[level]->dynamicEquation.getConstraint());
        new_solver->addObjective(std_obj[level]->minDdqObjective);
        new_solver->addObjective(std_obj[level]->minTauObjective);

        excludeObjective(level,std_obj[level]->minDdqObjective);
        excludeObjective(level,std_obj[level]->minTauObjective);
//         new_solver->addObjective(std_obj[level]->minFcObjective);  
        
        this->addSolver(new_solver,level);

    }
    try {
        auto ctask = std::dynamic_pointer_cast<ocra::OneLevelTask>(task);
        ctask->connectToController(this->getSolver(level), 
                                   std_obj[level]->dynamicEquation, 
                                   useReducedProblem);        
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


