#include "wocra/Tasks/Managers/wOcraContactTaskManager.h"

namespace wocra
{

/** Base constructor
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the tasks (prefix for the set of tasks)
 * \param _segmentName          Name of segment that the contacts are attached to
 * \param _H_segment_frame      Contact point local to the segment
 * \param _mu                   Coefficient of friction
 * \param _margin               Margin inside the friction cone
 */
wOcraContactTaskManager::wOcraContactTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, Eigen::Displacementd _H_segment_frame, double _mu, double _margin, bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), _H_segment_frame);
    feat = new ocra::PointContactFeature(name + ".PointContactFeature", *featFrame);

    task = &(ctrl.createwOcraContactTask(name, *feat, _mu, _margin));
    // Control the acceleration of the contact point
    task->initAsAccelerationTask();
    ctrl.addTask(*task);

    task->activateAsConstraint();
}

wOcraContactTaskManager::~wOcraContactTaskManager()
{
    
}

/** Activate function
 *
 *  Activates the constraint
 */
void wOcraContactTaskManager::activate()
{
    task->activateAsConstraint();
}

/** Deactivate function
 *
 *  Deactivates the constraint
 */
void wOcraContactTaskManager::deactivate()
{
    task->deactivate();
}

// Masks base class function
VectorXd wOcraContactTaskManager::getTaskError()
{
    throw std::runtime_error("[wOcraContactTaskManager::getTaskError()] Error is meaningless in this context or has not been computed");
}

// Masks base class function
double wOcraContactTaskManager::getTaskErrorNorm()
{
    throw std::runtime_error("[wOcraContactTaskManager::getTaskErrorNorm()] Error is meaningless in this context or has not been computed");
}

std::string wOcraContactTaskManager::getTaskManagerType()
{
    return "wOcraContactTaskManager";
}


}
