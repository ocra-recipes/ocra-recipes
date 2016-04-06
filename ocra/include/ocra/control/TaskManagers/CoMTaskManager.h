#ifndef COMTASKMANAGER_H
#define COMTASKMANAGER_H

#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/Model.h"



#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the  Controller
 *
 */
class CoMTaskManager: public TaskManager
{
    public:
        CoMTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        CoMTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        CoMTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight,
            Eigen::Vector3d posDes, bool usesYarpPorts = true);

        CoMTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight,
            Eigen::Vector3d posDes, bool usesYarpPorts = false);

        CoMTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, double weight,
            Eigen::Vector3d posDes, Eigen::Vector3d velDes, Eigen::Vector3d accDes, bool usesYarpPorts = true);

        CoMTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, ocra::ECartesianDof axes, double stiffness, double damping, const Eigen::VectorXd& weight,
            Eigen::Vector3d posDes, Eigen::Vector3d velDes, Eigen::Vector3d accDes, bool usesYarpPorts = false);

        ~CoMTaskManager();


        // Sets the position
        void setState(const Eigen::Vector3d& position);
        // Sets the position velocity and acceleration
        void setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);


        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();


        void setDesiredState();

        // Task error


    private:

        ocra::ECartesianDof              axes;
        ocra::PositionFeature*           feat;
        ocra::CoMFrame*                  comFeatFrame;
        ocra::PositionFeature*           featDes;
        ocra::TargetFrame*               featDesFrame;

        void _init(double stiffness, double damping, double weight);
        void _init(double stiffness, double damping, const Eigen::VectorXd& weight);
};

}

#endif // COMTASKMANAGER_H
