#ifndef wOcraSEGCARTESIANTASKMANAGER_H
#define wOcraSEGCARTESIANTASKMANAGER_H

#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/wOcraController.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for the cartesian position of a specified segment
 *
 */
class wOcraSegCartesianTaskManager : public wOcraTaskManagerBase
{
    public:
        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = true);

        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, double weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = true);

        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = false);

        wOcraSegCartesianTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Vector3d& targetPose, bool usesYarpPorts = false);

        ~wOcraSegCartesianTaskManager();

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


        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();
        virtual bool checkIfActivated();


        Eigen::Vector3d getTaskFramePosition();



    private:
        // wocra::wOcraTask*              task;

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

#endif // wOcraSEGCARTESIANTASKMANAGER_H
