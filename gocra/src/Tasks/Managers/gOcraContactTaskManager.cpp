#include "gocra/Tasks/Managers/gOcraContactTaskManager.h"

namespace gocra
{

/** Base constructor
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the tasks (prefix for the set of tasks)
 * \param _segmentName          Name of segment that the contacts are attached to
 * \param _H_segment_frame      Contact point local to the segment
 * \param _mu                   Coefficient of friction
 * \param _margin               Margin inside the friction cone
 */
gOcraContactTaskManager::gOcraContactTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, Eigen::Displacementd _H_segment_frame, double _mu, double _margin)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), _H_segment_frame);
    feat = new ocra::PointContactFeature(name + ".PointContactFeature", *featFrame);

    task = &(ctrl.createGHCJTContactTask(name, *feat, _mu, _margin));

    ctrl.addTask(*task);

    task->activateAsConstraint();
}

/** Activate function
 *
 *  Activates the constraint 
 */
void gOcraContactTaskManager::activate()
{
    task->activateAsConstraint();
}

/** Deactivate function
 *
 *  Deactivates the constraint
 */
void gOcraContactTaskManager::deactivate()
{
    task->deactivate();
}

// Masks base class function
VectorXd gOcraContactTaskManager::getTaskError()
{
    throw std::runtime_error("[gOcraContactTaskManager::getTaskError()] Error is meaningless in this context or has not been computed");
}

// Masks base class function
double gOcraContactTaskManager::getTaskErrorNorm()
{
    throw std::runtime_error("[gOcraContactTaskManager::getTaskErrorNorm()] Error is meaningless in this context or has not been computed");
}


}
