#include "ocra/control/TaskManagers/ContactSetTaskManager.h"
#include <stdio.h>

namespace ocra
{

/** Base constructor
 *
 * \param _ctrl                 ocra::Controller to connect to
 * \param _model                ocra model to setup the task
 * \param _taskName             Name of the taskVector (prefix for the set of taskVector)
 * \param _segmentName          Name of segment that the contacts are attached to
 * \param _H_segment_frames      Array of contact points local to the segment
 * \param _numContacts          Number of contact points in this set
 * \param _mu                   Coefficient of friction
 * \param _margin               Margin inside the friction cone
 */
ContactSetTaskManager::ContactSetTaskManager(   ocra::Controller& _ctrl,
                                                const ocra::Model& _model,
                                                const std::string& _taskName,
                                                const std::string& _segmentName,
                                                std::vector<Eigen::Displacementd> _H_segment_frames,
                                                double _mu,
                                                double _margin,
                                                int _hierarchyLevel,
                                                bool _usesYarpPorts)
    : TaskManager(_ctrl, _model, _taskName, _usesYarpPorts)
    , segmentName(_segmentName)
    , numContacts(_H_segment_frames.size())
{
    taskVector.resize(numContacts);
    feats.resize(numContacts);
    featFrames.resize(numContacts);
    names.resize(numContacts);
    // feats = std::make_shared<PointContactFeature*[numContacts];
    // featFrames = std::make_shared<SegmentFrame*[numContacts];
    // names = new std::string[numContacts];

    for (int i = 0; i < numContacts; i++)
    {
        std::ostringstream name_stream;
        name_stream << name << i;
        names[i] = name_stream.str();

        featFrames[i] = std::make_shared<ocra::SegmentFrame>(names[i] + ".SegmentFrame", model, model.SegmentName(segmentName), _H_segment_frames[i]);
        feats[i] = std::make_shared<ocra::PointContactFeature>(names[i] + ".PointContactFeature", featFrames[i]);

        taskVector[i] = ctrl.createContactTask(names[i], feats[i], _mu, _margin);
        // Control the acceleration of the contact point
        taskVector[i]->setTaskType(ocra::Task::ACCELERATIONTASK);
        taskVector[i]->setHierarchyLevel(_hierarchyLevel);
        ctrl.addTask(taskVector[i]);

        taskVector[i]->activateAsConstraint();
    }

}

ContactSetTaskManager::~ContactSetTaskManager()
{
    // for (int i = 0; i < numContacts; i++)
    // {
    //     taskVector[i]->deactivate();
    //     // taskVector[i]->disconnectFromController();
    // }
}

// Masks base class function
VectorXd ContactSetTaskManager::getTaskError()
{
    throw std::runtime_error("[ContactSetTaskManager::getTaskError()] Error is meaningless in this context or has not been computed");
}

// Masks base class function
double ContactSetTaskManager::getTaskErrorNorm()
{
    throw std::runtime_error("[ContactSetTaskManager::getTaskErrorNorm()] Error is meaningless in this context or has not been computed");
}

std::string ContactSetTaskManager::getTaskManagerType()
{
    return "ContactSetTaskManager";
}

}
