#ifndef wOcraCOMTASKMANAGER_H
#define wOcraCOMTASKMANAGER_H

#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/wOcraController.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the wOcra Controller
 *
 */
class wOcraCoMTaskManager: public wOcraTaskManagerBase
{
    public:
        wOcraCoMTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        wOcraCoMTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        wOcraCoMTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight,
            Eigen::Vector3d posDes, bool usesYarpPorts = true);

        wOcraCoMTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight,
            Eigen::Vector3d posDes, bool usesYarpPorts = false);

        wOcraCoMTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight,
            Eigen::Vector3d posDes, Eigen::Vector3d velDes, Eigen::Vector3d accDes, bool usesYarpPorts = true);

        wOcraCoMTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight,
            Eigen::Vector3d posDes, Eigen::Vector3d velDes, Eigen::Vector3d accDes, bool usesYarpPorts = false);

        ~wOcraCoMTaskManager();


        // Sets the position
        void setState(const Eigen::Vector3d& position);
        // Sets the position velocity and acceleration
        void setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);


        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();
        virtual bool checkIfActivated();

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

    private:
        // wocra::wOcraTask*          task;
        ocra::ECartesianDof              axes;
        ocra::PositionFeature*           feat;
        ocra::CoMFrame*                  featFrame;
        ocra::PositionFeature*           featDes;
        ocra::TargetFrame*               featDesFrame;

        void _init(double stiffness, double damping, double weight);
        void _init(double stiffness, double damping, const Eigen::VectorXd& weight);
};

}

#endif // wOcraCOMTASKMANAGER_H
