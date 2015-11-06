#ifndef gOcraCONTACTSETTASKMANAGER_H
#define gOcraCONTACTSETTASKMANAGER_H

#include "ocra/control/Model.h"
#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/GHCJTController.h"
#include "gocra/Features/gOcraFeature.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for a set of contact tasks
 *
 *  This class eases the activation and deactivation for a set of contact tasks (no need to write a loop remembering the task indices to activate and deactivate)
 */
class gOcraContactSetTaskManager: public gOcraTaskManagerBase
{
    public:
        gOcraContactSetTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, Eigen::Displacementd H_segment_frame[], int numContacts, double mu, double margin);

        ~gOcraContactSetTaskManager();

        void activate();
        void deactivate();
        
        VectorXd getTaskError(); // Overrides base class function in this context
        double getTaskErrorNorm(); // Overrides base class function in this context
 
    private:
        const std::string&              segmentName;

        std::string*                    names;
        gocra::GHCJTTask**             tasks;
        int                             numContacts;

        ocra::PointContactFeature**      feats;
        ocra::SegmentFrame**             featFrames;
};

}

#endif // gOcraCONTACTSETTASKMANAGER_H
