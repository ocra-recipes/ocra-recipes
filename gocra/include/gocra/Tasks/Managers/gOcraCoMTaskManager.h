#ifndef gOcraCOMTASKMANAGER_H
#define gOcraCOMTASKMANAGER_H

#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/GHCJTController.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the gOcra Controller
 *
 */
class gOcraCoMTaskManager: public gOcraTaskManagerBase
{
    public:
        gOcraCoMTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping);

        gOcraCoMTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, Eigen::Vector3d posDes);

        gOcraCoMTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, Eigen::Vector3d posDes, Eigen::Vector3d velDes, Eigen::Vector3d accDes);

        ~gOcraCoMTaskManager();


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
        ocra::ECartesianDof              axes;
        ocra::PositionFeature*           feat;
        ocra::CoMFrame*                  featFrame;
        ocra::PositionFeature*           featDes;
        ocra::TargetFrame*               featDesFrame;

        void _init(double stiffness, double damping);
};

}

#endif // gOcraCOMTASKMANAGER_H
