#ifndef gOcraSEGPOSETASKMANAGER_H
#define gOcraSEGPOSETASKMANAGER_H

#include "ocra/control/Model.h"
#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/GHCJTController.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for a segment's pose
 *
 */
class gOcraSegPoseTaskManager : public gOcraTaskManagerBase
{
    public:
        gOcraSegPoseTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping);

        gOcraSegPoseTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::Displacementd& targetPose);

        ~gOcraSegPoseTaskManager();

        void setState(const Eigen::Displacementd& pose);
        void setState(const Eigen::Displacementd& pose, const Eigen::Twistd& velocity, const Eigen::Twistd& acceleration);

        // All Managers have this
        void activate();
        void deactivate();

        // For objective tasks
        void setStiffness(double stiffness);
        double getStiffness();
        void setDamping(double damping);
        double getDamping();

        // Task error
        Eigen::VectorXd getTaskError();

    private:
        gocra::GHCJTTask*              task;

        const std::string&              segmentName;
        ocra::ECartesianDof              axes;

        ocra::DisplacementFeature*       feat;
        ocra::DisplacementFeature*       featDes;
        ocra::SegmentFrame*              featFrame;
        ocra::TargetFrame*               featDesFrame;

        void _init(const Eigen::Displacementd& ref_LocalFrame, double stiffness, double damping);
};

}

#endif // gOcraSEGPOSETASKMANAGER_H
