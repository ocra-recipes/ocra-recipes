#include "gocra/Tasks/Managers/gOcraContactSetTaskManager.h"
#include <stdio.h>

namespace gocra
{

/** Base constructor
 *
 * \param _ctrl                 GHCJTController to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the tasks (prefix for the set of tasks)
 * \param _segmentName          Name of segment that the contacts are attached to
 * \param _H_segment_frame      Array of contact points local to the segment
 * \param _numContacts          Number of contact points in this set
 * \param _mu                   Coefficient of friction
 * \param _margin               Margin inside the friction cone
 */
gOcraContactSetTaskManager::gOcraContactSetTaskManager(GHCJTController& _ctrl, const gOcraModel& _model, const std::string& _taskName, const std::string& _segmentName, Eigen::Displacementd _H_segment_frame[], int _numContacts, double _mu, double _margin)
    : gOcraTaskManagerBase(_ctrl, _model, _taskName), segmentName(_segmentName), numContacts(_numContacts)
{
    tasks = new gocra::GHCJTTask*[numContacts];
    feats = new ocra::PointContactFeature*[numContacts];
    featFrames = new ocra::SegmentFrame*[numContacts];
    names = new std::string[numContacts];

    for (int i = 0; i < numContacts; i++)
    {
        std::ostringstream name_stream;
        name_stream << name << i;
        names[i] = name_stream.str();

        featFrames[i] = new ocra::SegmentFrame(names[i] + ".SegmentFrame", model, model.SegmentName(segmentName), _H_segment_frame[i]);
        feats[i] = new ocra::PointContactFeature(names[i] + ".PointContactFeature", *featFrames[i]);
        tasks[i] = &(ctrl.createGHCJTContactTask(names[i], *feats[i], _mu, _margin));

        ctrl.addTask(*tasks[i]);

        tasks[i]->activateAsConstraint();
    }

}

/** Activate function
 *
 *  Activates all the constraints (as constraints) 
 */
void gOcraContactSetTaskManager::activate()
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
void gOcraContactSetTaskManager::deactivate()
{
    for (int i = 0; i < numContacts; i++)
    {
        tasks[i]->deactivate();
    }
}

// Masks base class function
VectorXd gOcraContactSetTaskManager::getTaskError()
{
    throw std::runtime_error("[gOcraContactSetTaskManager::getTaskError()] Error is meaningless in this context or has not been computed");
}

// Masks base class function
double gOcraContactSetTaskManager::getTaskErrorNorm()
{
    throw std::runtime_error("[gOcraContactSetTaskManager::getTaskErrorNorm()] Error is meaningless in this context or has not been computed");
}

}
