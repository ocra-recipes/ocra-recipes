#ifndef gOcraPARTIALPOSTURETASKMANAGER_H
#define gOcraPARTIALPOSTURETASKMANAGER_H

#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/GHCJTController.h"
#include "gocra/Features/gOcraFeature.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for partal joint posture task
 *
 */
class gOcraPartialPostureTaskManager : public gOcraTaskManagerBase
{
    public:
        gOcraPartialPostureTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping);

        gOcraPartialPostureTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, Eigen::VectorXd& init_q);

        ~gOcraPartialPostureTaskManager();

        void setPosture(Eigen::VectorXd& q);
        void setPosture(Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot);
/*
        void setStiffnessDamping(double stiffness, double damping);
*/

        void activate();
        void deactivate();
 
    private:
        gocra::GHCJTTask*                      task;
        gocra::PartialStateFeature*           feat;
        gocra::PartialModelState*             featState;

        gocra::PartialStateFeature*           featDes;
        gocra::PartialTargetState*            featDesState;

/*
        Eigen::Vector3d                 _posDes;
        Eigen::Vector3d                 _velDes;
        Eigen::Vector3d                 _accDes;
*/

        void _init(int fullStateType, VectorXi& dofIndices, double stiffness, double damping);
};

}

#endif // gOcraFULLPOSTURETASKMANAGER_H
