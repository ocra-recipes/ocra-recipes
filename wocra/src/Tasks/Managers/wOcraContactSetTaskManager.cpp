#include "wocra/Tasks/Managers/wOcraContactSetTaskManager.h"
#include <stdio.h>

namespace wocra
{

/** Base constructor
 *
 * \param _ctrl                 wOcraController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the tasks (prefix for the set of tasks)
 * \param _segmentName          Name of segment that the contacts are attached to
 * \param _H_segment_frames      Array of contact points local to the segment
 * \param _numContacts          Number of contact points in this set
 * \param _mu                   Coefficient of friction
 * \param _margin               Margin inside the friction cone
 */
wOcraContactSetTaskManager::wOcraContactSetTaskManager(wOcraController& _ctrl, const wOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, std::vector<Eigen::Displacementd> _H_segment_frames, double _mu, double _margin, bool _usesYarpPorts)
    : wOcraTaskManagerBase(_ctrl, _model, _taskName, _usesYarpPorts), segmentName(_segmentName), numContacts(_H_segment_frames.size())
{
    tasks = new wocra::wOcraTask*[numContacts];
    feats = new ocra::PointContactFeature*[numContacts];
    featFrames = new ocra::SegmentFrame*[numContacts];
    names = new std::string[numContacts];

    for (int i = 0; i < numContacts; i++)
    {
        std::ostringstream name_stream;
        name_stream << name << i;
        names[i] = name_stream.str();

        featFrames[i] = new ocra::SegmentFrame(names[i] + ".SegmentFrame", model, model.SegmentName(segmentName), _H_segment_frames[i]);
        feats[i] = new ocra::PointContactFeature(names[i] + ".PointContactFeature", *featFrames[i]);
        tasks[i] = &(ctrl.createwOcraContactTask(names[i], *feats[i], _mu, _margin));
        // Control the acceleration of the contact point
        tasks[i]->initAsAccelerationTask();
        ctrl.addTask(*tasks[i]);

        tasks[i]->activateAsConstraint();
    }

}

wOcraContactSetTaskManager::~wOcraContactSetTaskManager()
{
    for (int i = 0; i < numContacts; i++)
    {
        tasks[i]->deactivate();
        tasks[i]->disconnectFromController();
    }
}

/** Activate function
 *
 *  Activates all the constraints (as constraints)
 */
void wOcraContactSetTaskManager::activate()
{
    for (int i = 0; i < numContacts; i++)
    {
        tasks[i]->activateAsConstraint();
    }
}

/** Deactivate function
 *
 *  Deactivates all the constraints
 */
void wOcraContactSetTaskManager::deactivate()
{
    for (int i = 0; i < numContacts; i++)
    {
        tasks[i]->deactivate();
    }
}

// Masks base class function
VectorXd wOcraContactSetTaskManager::getTaskError()
{
    throw std::runtime_error("[wOcraContactSetTaskManager::getTaskError()] Error is meaningless in this context or has not been computed");
}

// Masks base class function
double wOcraContactSetTaskManager::getTaskErrorNorm()
{
    throw std::runtime_error("[wOcraContactSetTaskManager::getTaskErrorNorm()] Error is meaningless in this context or has not been computed");
}

std::string wOcraContactSetTaskManager::getTaskManagerType()
{
    return "wOcraContactSetTaskManager";
}

}
