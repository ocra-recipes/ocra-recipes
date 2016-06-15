#ifndef PARTIALPOSTURETASKMANAGER_H
#define PARTIALPOSTURETASKMANAGER_H

#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/Model.h"


// #include "wocra/Features/Feature.h"
#include "ocra/control/Feature.h"

#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for partal joint posture task
 *
 */
class PartialPostureTaskManager : public TaskManager
{
    public:
        PartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        PartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, const Eigen::VectorXd& weight, int hierarchyLevel = -1 , bool usesYarpPorts = false);

        PartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight,
            Eigen::VectorXd& init_q, int hierarchyLevel = -1 , bool usesYarpPorts = true);

        PartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, const Eigen::VectorXd& weight,
            Eigen::VectorXd& init_q, int hierarchyLevel = -1 , bool usesYarpPorts = false);

        ~PartialPostureTaskManager();

        void setPosture(Eigen::VectorXd& q);
        void setPosture(Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot);

        void setDesiredState();



        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();

    private:
        ocra::PartialStateFeature::Ptr           feat;
        ocra::PartialModelState::Ptr             featState;

        ocra::PartialStateFeature::Ptr           featDes;
        ocra::PartialTargetState::Ptr            featDesState;

/*
        Eigen::Vector3d                 _posDes;
        Eigen::Vector3d                 _velDes;
        Eigen::Vector3d                 _accDes;
*/

        void _init(int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight, int hierarchyLevel);
        void _init(int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, const Eigen::VectorXd& weight, int hierarchyLevel);
};

}

#endif // FULLPOSTURETASKMANAGER_H
