#ifndef SEGPOSETASKMANAGER_H
#define SEGPOSETASKMANAGER_H

#include "ocra/control/Model.h"
#include "ocra/control/TaskManagers/TaskManager.h"



#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for a segment's pose
 *
 */
class SegPoseTaskManager : public TaskManager
{
    public:
        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = true);

        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = true);

        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = false);

        SegPoseTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Displacementd& segFrame_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Displacementd& targetPose, bool usesYarpPorts = false);

        ~SegPoseTaskManager();

        void setState(const Eigen::Displacementd& pose);
        void setState(const Eigen::Displacementd& pose, const Eigen::Twistd& velocity, const Eigen::Twistd& acceleration);

        
        void setDesiredState();


        // Task error


        Eigen::Displacementd getTaskFrameDisplacement();
        Eigen::Twistd getTaskFrameVelocity();
        Eigen::Twistd getTaskFrameAcceleration();
        Eigen::Vector3d getTaskFramePosition();
        Eigen::Rotation3d getTaskFrameOrientation();
        Eigen::Vector3d getTaskFrameLinearVelocity();
        Eigen::Vector3d getTaskFrameAngularVelocity();
        Eigen::Vector3d getTaskFrameLinearAcceleration();
        Eigen::Vector3d getTaskFrameAngularAcceleration();


        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();



    private:

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

#endif // SEGPOSETASKMANAGER_H
