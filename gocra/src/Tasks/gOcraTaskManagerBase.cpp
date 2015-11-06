#include "gocra/Tasks/gOcraTaskManagerBase.h"

namespace gocra
{

/** base constructor
 *
 * \param ctrl                  GHCJTController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 */
gOcraTaskManagerBase::gOcraTaskManagerBase(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName)
    : ctrl(_ctrl), model(_model), name(_taskName)
{
}

/** Returns the error vector of the task 
 * 
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
Eigen::VectorXd gOcraTaskManagerBase::getTaskError()
{
    throw std::runtime_error(std::string("[gOcraTaskManagerBase::getTaskError()]: getTaskError has not been implemented or is not supported"));
}

/** Returns the norm of the error vector 
 * 
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
double gOcraTaskManagerBase::getTaskErrorNorm()
{
    return getTaskError().norm();
}

}
