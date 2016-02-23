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
        ContactSetTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, std::vector<Eigen::Displacementd> H_segment_frames, double mu, double margin, bool usesYarpPorts = true);

        ~ContactSetTaskManager();

        void activate();
        void deactivate();
        virtual std::string getTaskManagerType();


        VectorXd getTaskError(); // Overrides base class function in this context
        double getTaskErrorNorm(); // Overrides base class function in this context

    private:
        const std::string&              segmentName;

        std::string*                    names;
        ocra::Task**             tasks;
        int                             numContacts;

        ocra::PointContactFeature**      feats;
        ocra::SegmentFrame**             featFrames;
};

}

#endif // CONTACTSETTASKMANAGER_H
