#include "ocra/control/TaskManagers/ContactTaskManager.h"

namespace ocra
{

/** Base constructor
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the tasks (prefix for the set of tasks)
 * \param _segmentName          Name of segment that the contacts are attached to
 * \param _H_segment_frame      Contact point local to the segment
 * \param _mu                   Coefficient of friction
 * \param _margin               Margin inside the friction cone
 */
ContactTaskManager::ContactTaskManager(ocra::Controller& _ctrl, const ocra::Model& _model, const std::string& _taskName, const std::string& _segmentName, Eigen::Displacementd _H_segment_frame, double _mu, double _margin, bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName)
{
    featFrame = new ocra::SegmentFrame(name + ".SegmentFrame", model, model.SegmentName(segmentName), _H_segment_frame);
    feat = new ocra::PointContactFeature(name + ".PointContactFeature", *featFrame);

    task = ctrl.createContactTask(name, *feat, _mu, _margin);
    // Control the acceleration of the contact point
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);

    task->activateAsConstraint();
}

ContactTaskManager::~ContactTaskManager()
{

}

// Masks base class function
VectorXd ContactTaskManager::getTaskError()
{
    throw std::runtime_error("[ContactTaskManager::getTaskError()] Error is meaningless in this context or has not been computed");
}

// Masks base class function
double ContactTaskManager::getTaskErrorNorm()
{
    throw std::runtime_error("[ContactTaskManager::getTaskErrorNorm()] Error is meaningless in this context or has not been computed");
}

std::string ContactTaskManager::getTaskManagerType()
{
    return "ContactTaskManager";
}


}
