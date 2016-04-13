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

namespace hocra

{
using namespace ocra;

/** \addtogroup core

 * \{

 */



/** \brief Hocra Controller based on LQP solver for the ocra framework.

 *

 */

class HocraController: public Controller

{

public:

    HocraController(const std::string& ctrlName, 
                    std::shared_ptr<Model> innerModel, 
                    std::shared_ptr<OneLevelSolver> innerSolver, 
                    bool useReducedProblem);
    virtual void doAddContactSet(const ContactSet& contacts);
    virtual void doAddTask(std::shared_ptr< Task > task);
    virtual void doComputeOutput(VectorXd& tau);
    virtual std::shared_ptr< Task > doCreateContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const;
    virtual std::shared_ptr< Task > doCreateTask(const std::string& name, const Feature& feature) const;
    virtual std::shared_ptr< Task > doCreateTask(const std::string& name, const Feature& feature, const Feature& featureDes) const;
    virtual ~HocraController(){};
    
private:
    struct Pimpl;

    std::shared_ptr<Pimpl> pimpl;


};



/** \} */ // end group core



} //end namespace hocra







#endif
