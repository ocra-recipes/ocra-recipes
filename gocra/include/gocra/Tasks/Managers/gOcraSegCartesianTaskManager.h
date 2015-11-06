#ifndef gOcraSEGCARTESIANTASKMANAGER_H
#define gOcraSEGCARTESIANTASKMANAGER_H

#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/GHCJTController.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for the cartesian position of a specified segment
 *
 */
class gOcraSegCartesianTaskManager : public gOcraTaskManagerBase
{
    public:
        gOcraSegCartesianTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping);

        gOcraSegCartesianTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping);

        gOcraSegCartesianTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::Vector3d& targetPose);

        gOcraSegCartesianTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, const Eigen::Vector3d& segPoint_Local, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::Vector3d& targetPose);

        ~gOcraSegCartesianTaskManager();

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

        // Task error
        Eigen::VectorXd getTaskError();


    private:
        gocra::GHCJTTask*              task;

        const std::string&              segmentName;
        ocra::ECartesianDof              axes;
        ocra::PositionFeature*           feat;
        ocra::PositionFeature*           featDes;
        ocra::SegmentFrame*              featFrame;
        ocra::TargetFrame*               featDesFrame;

        void _init(const Eigen::Vector3d& refPoint_LocalFrame, double stiffness, double damping);
};

}

#endif // gOcraSEGCARTESIANTASKMANAGER_H
