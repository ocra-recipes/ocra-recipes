#ifndef wOcraSEGPOSETASKMANAGER_H
#define wOcraSEGPOSETASKMANAGER_H

#include "ocra/control/Model.h"
#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/wOcraController.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for a segment's pose
 *
 */
class wOcraSegPoseTaskManager : public wOcraTaskManagerBase
{
    public:
        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = true);

        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = true);

        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = false);

        wOcraSegPoseTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = false);

        ~wOcraSegPoseTaskManager();

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
        void setWeight(double weight);
        void setWeight(const Eigen::VectorXd& weight);
        Eigen::VectorXd getWeight();
        void setDesiredState();


        // Task error
        Eigen::VectorXd getTaskError();

        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();
        virtual bool checkIfActivated();



    private:
        // wocra::wOcraTask*              task;

        const std::string&              segmentName;
        ocra::ECartesianDof              axes;

        ocra::DisplacementFeature*       feat;
        ocra::DisplacementFeature*       featDes;
        ocra::SegmentFrame*              featFrame;
        ocra::TargetFrame*               featDesFrame;

        void _init(const Eigen::Displacementd& ref_LocalFrame, double stiffness, double damping, double weight);
        void _init(const Eigen::Displacementd& ref_LocalFrame, double stiffness, double damping, const Eigen::VectorXd& weight);
};

}

#endif // wOcraSEGPOSETASKMANAGER_H
