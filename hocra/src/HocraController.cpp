#include <hocra/HocraController.h>


namespace hocra {
HocraController::HocraController(const std::string& ctrlName, 
                                 std::shared_ptr< Model > innerModel, 
                                 std::shared_ptr< OneLevelSolver > innerSolver, 
                                 bool useReducedProblem): 
WocraController(ctrlName, innerModel, innerSolver, useReducedProblem)
{
    std::cout << "Constructing a HOCRA Controller ! " << std::endl;
}

    
    
}