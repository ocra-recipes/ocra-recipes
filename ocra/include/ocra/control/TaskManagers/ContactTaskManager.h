#ifndef CONTACTTASKMANAGER_H
#define CONTACTTASKMANAGER_H

#include "ocra/control/Model.h"

#include "ocra/control/TaskManagers/TaskManager.h"

// #include "wocra/Features/Feature.h"
#include "ocra/control/Feature.h"

#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for a contact task
 *
 */
class ContactTaskManager : public TaskManager
{
    public:
        ContactTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, Eigen::Displacementd H_segment_frame, double mu, double margin, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        ~ContactTaskManager();


        virtual std::string getTaskManagerType();

        VectorXd getTaskError(); // Overrides base class function in this context
        double getTaskErrorNorm(); // Overrides base class function in this context
    private:

        const std::string&          segmentName;

        ocra::PointContactFeature::Ptr   feat;
};

}

#endif // CONTACTTASKMANAGER_H
