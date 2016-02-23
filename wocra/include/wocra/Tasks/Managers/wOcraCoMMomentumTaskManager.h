#ifndef wOcraCOMMOMENTUMTASKMANAGER_H
#define wOcraCOMMOMENTUMTASKMANAGER_H

#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/wOcraController.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for the Center of Mass (CoM) momentum task with the wOcra Controller
 *
 */
class wOcraCoMMomentumTaskManager: public wOcraTaskManagerBase
{
    public:
        wOcraCoMMomentumTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double damping, double weight, bool usesYarpPorts = true);

        wOcraCoMMomentumTaskManager(wOcraController& ctrl, const wOcraModel& model, const std::string& taskName, ocra::ECartesianDof axes, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);


        ~wOcraCoMMomentumTaskManager();


//        // Sets the position
//        void setState(const Eigen::Vector3d& position);
//        // Sets the position velocity and acceleration
//        void setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);


        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();
        virtual bool checkIfActivated();

        // All Managers have this
        void activate();
        void deactivate();

        // For objective tasks
//        void setStiffness(double stiffness);
//        double getStiffness();
        void setDamping(double damping);
        double getDamping();
        void setWeight(double weight);
        void setWeight(const Eigen::VectorXd& weight);
        Eigen::VectorXd getWeight();
//        void setDesiredState();

        // Task error
//        Eigen::VectorXd getTaskError();

    private:
        // wocra::wOcraTask*          task;
        ocra::ECartesianDof              axes;
        ocra::PositionFeature*           feat;
        ocra::CoMFrame*                  featFrame;
        ocra::PositionFeature*           featDes;
        ocra::TargetFrame*               featDesFrame;

        void _init(double damping, double weight);
        void _init(double damping, const Eigen::VectorXd& weight);
};

}

#endif // wOcraCOMTASKMANAGER_H
