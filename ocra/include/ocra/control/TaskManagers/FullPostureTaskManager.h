#ifndef FULLPOSTURETASKMANAGER_H
#define FULLPOSTURETASKMANAGER_H

#include "ocra/control/Model.h"
#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/FullState.h"
//


#include <Eigen/Dense>

namespace ocra
{

/** \brief  Task Manager for the joint space posture
 *
 */
class FullPostureTaskManager : public TaskManager
{
    public:
        FullPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, double stiffness, double damping, double weight, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        FullPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, double stiffness, double damping, const Eigen::VectorXd& weight, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        FullPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, double stiffness, double damping, double weight, const Eigen::VectorXd& init_q, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        FullPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::VectorXd& init_q, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        ~FullPostureTaskManager();


        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();

        // Set the task reference
        void setPosture(const Eigen::VectorXd& q);
        void setPosture(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot);

        
        void setDesiredState();


        // Task error



    private:



        ocra::FullStateFeature::Ptr          feat;
        ocra::FullModelState::Ptr            featState;

        ocra::FullStateFeature::Ptr          featDes;
        ocra::FullTargetState::Ptr           featDesState;

        void _init(int fullStateType, double stiffness, double damping, double weight, int hierarchyLevel);
        void _init(int fullStateType, double stiffness, double damping, const Eigen::VectorXd& weight, int hierarchyLevel);
};

}

#endif // FULLPOSTURETASKMANAGER_H
