#include "ocra/optim/CascadeQPSolver.h"

#include <algorithm>
#include "ocra/utilities.h"

namespace ocra
{

CascadeQPSolver::CascadeQPSolver():
    ocra::Solver(),
    ocra::NamedInstance("CascadeQPSolver")
{
    std::cout << "CascadeQPSolver constructor" << std::endl;
}



void CascadeQPSolver::doSolve(void)
{
    std::cout << "CascadeQPSolver::doSolve" << std::endl;
}


void CascadeQPSolver::doPrepare(void)
{
    std::cout << "CascadeQPSolver::doPrepare" << std::endl;
}




void CascadeQPSolver::doConclude()
{
    std::cout << "CascadeQPSolver::doConclude" << std::endl;
}
std::string CascadeQPSolver::toString() const
{

}
void CascadeQPSolver::addSolver(OneLevelSolver::Ptr solver, int level)
{
   std::cout << "Creating solver " << solver->getName() << std::endl;
    this->solvermap[level] = solver->clone();
}
void CascadeQPSolver::addTask(Task::Ptr task)
{
    this->taskmap[task->getHierarchyLevel()].push_back(std::move(task));
}
void CascadeQPSolver::printValuesAtSolution()
{

}


} // Namespace ocra


