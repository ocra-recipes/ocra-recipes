#ifndef CONTACTSETTASKMANAGER_H
#define CONTACTSETTASKMANAGER_H

#include "ocra/control/Model.h"
#include "ocra/control/TaskManagers/TaskManager.h"


// #include "wocra/Features/Feature.h"
#include "ocra/control/Feature.h"

#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for a set of contact tasks
 *
 *  This class eases the activation and deactivation for a set of contact tasks (no need to write a loop remembering the task indices to activate and deactivate)
 */
class ContactSetTaskManager: public TaskManager
{
    public:
        ContactSetTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, std::vector<Eigen::Displacementd> H_segment_frames, double mu, double margin, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        ~ContactSetTaskManager();


        virtual std::string getTaskManagerType();


        VectorXd getTaskError(); // Overrides base class function in this context
        double getTaskErrorNorm(); // Overrides base class function in this context

    private:

        int                                                             numContacts;
        std::string                                                     segmentName;
        std::vector< std::string >                                      names;
        std::vector< std::shared_ptr<ocra::PointContactFeature> >       feats;
};

}

#endif // CONTACTSETTASKMANAGER_H
