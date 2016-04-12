#ifndef __HOCRACTRL_H__

#define __HOCRACTRL_H__


#include <string>
#include <vector>
#include <iostream>
#include <memory>

#include <ocra/control/Controller.h>
#include <ocra/optim/CascadeQPSolver.h>
#include <ocra/control/ControlConstraint.h>
#include <ocra/optim/QuadraticFunction.h>
#include <ocra/control/Tasks/Task.h>
#include <ocra/control/Tasks/OneLevelTask.h>
#include <ocra/control/FullDynamicEquationFunction.h>
#include <wocra/WocraController.h>

namespace hocra

{
using namespace ocra;
using namespace wocra;

/** \addtogroup core

 * \{

 */



/** \brief Hocra Controller based on LQP solver for the ocra framework.

 *

 */

class HocraController: public WocraController

{

public:



    HocraController(const std::string& ctrlName, 
                    std::shared_ptr<Model> innerModel, 
                    std::shared_ptr<OneLevelSolver> innerSolver, 
                    bool useReducedProblem);

private:

    struct Pimpl;

    boost::shared_ptr<Pimpl> pimpl;


};



/** \} */ // end group core



} //end namespace hocra







#endif
