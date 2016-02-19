#ifndef SEGCARTESIANTASKMANAGER_H
#define SEGCARTESIANTASKMANAGER_H

#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/Model.h"



#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for the cartesian position of a specified segment
 *
 */
class SegCartesianTaskManager : public TaskManager
{
    public:
        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = true);

        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = true);

        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = false);

        SegCartesianTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = false);

        ~SegCartesianTaskManager();

        // Sets the position
        void setState(const Eigen::Vector3d& position);
        // Sets the position velocity and acceleration
        void setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);

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
        virtual bool checkIfActivated();





    private:

        const std::string&              segmentName;
        ocra::ECartesianDof              axes;
        ocra::PositionFeature*           feat;
        ocra::PositionFeature*           featDes;
        ocra::SegmentFrame*              featFrame;
        ocra::TargetFrame*               featDesFrame;

        void _init(const Eigen::Vector3d& refPoint_LocalFrame, double stiffness, double damping, double weight);
        void _init(const Eigen::Vector3d& refPoint_LocalFrame, double stiffness, double damping, const Eigen::VectorXd& weight);
};

}

#endif // SEGCARTESIANTASKMANAGER_H
