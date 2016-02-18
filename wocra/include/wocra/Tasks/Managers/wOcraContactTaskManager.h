#ifndef wOcraCONTACTTASKMANAGER_H
#define wOcraCONTACTTASKMANAGER_H

#include "ocra/control/Model.h"

#include "wocra/Tasks/wOcraTaskManagerBase.h"

// #include "wocra/Features/wOcraFeature.h"
#include "ocra/control/Feature.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for a contact task
 *
 */
class wOcraContactTaskManager : public wOcraTaskManagerBase
{
    public:
        wOcraContactTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, Eigen::Displacementd H_segment_frame, double mu, double margin, bool usesYarpPorts = true);

        ~wOcraContactTaskManager();

        void activate();
        void deactivate();
        virtual std::string getTaskManagerType();

        VectorXd getTaskError(); // Overrides base class function in this context
        double getTaskErrorNorm(); // Overrides base class function in this context
    private:
        
        const std::string&          segmentName;

        ocra::PointContactFeature*   feat;
        ocra::SegmentFrame*          featFrame;
};

}

#endif // wOcraCONTACTTASKMANAGER_H
