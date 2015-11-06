#ifndef gOcraCONTACTTASKMANAGER_H
#define gOcraCONTACTTASKMANAGER_H

#include "ocra/control/Model.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "gocra/GHCJTController.h"
#include "gocra/Features/gOcraFeature.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for a contact task
 *
 */
class gOcraContactTaskManager : public gOcraTaskManagerBase
{
    public:
        gOcraContactTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, Eigen::Displacementd H_segment_frame, double mu, double margin);

        ~gOcraContactTaskManager();

        void activate();
        void deactivate();

        VectorXd getTaskError(); // Overrides base class function in this context
        double getTaskErrorNorm(); // Overrides base class function in this context
    private:
        gocra::GHCJTTask*          task;
        const std::string&          segmentName;

        ocra::PointContactFeature*   feat;
        ocra::SegmentFrame*          featFrame;
};

}

#endif // gOcraCONTACTTASKMANAGER_H
