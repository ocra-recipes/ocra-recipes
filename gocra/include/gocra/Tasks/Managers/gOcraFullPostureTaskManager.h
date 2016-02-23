#ifndef gOcraFULLPOSTURETASKMANAGER_H
#define gOcraFULLPOSTURETASKMANAGER_H

#include "ocra/control/Model.h"
#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "ocra/control/FullState.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/GHCJTController.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief gOcra Task Manager for the joint space posture
 *
 */
class gOcraFullPostureTaskManager : public gOcraTaskManagerBase
{
    public:
        gOcraFullPostureTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, int fullStateType, double stiffness, double damping);

        gOcraFullPostureTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, int fullStateType, double stiffness, double damping, const Eigen::VectorXd& init_q);

        ~gOcraFullPostureTaskManager();

        // All Managers have this
        void activate();
        void deactivate();

        // Set the task reference
        void setPosture(const Eigen::VectorXd& q);
        void setPosture(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot);

        // For objective tasks
        void setStiffness(double stiffness);
        double getStiffness();
        void setDamping(double damping);
        double getDamping();


        // Task error
        Eigen::VectorXd getTaskError();


    private:
        gocra::GHCJTTask*              task;

        ocra::FullStateFeature*          feat;
        ocra::FullModelState*            featState;

        ocra::FullStateFeature*          featDes;
        ocra::FullTargetState*           featDesState;

        void _init(int fullStateType, double stiffness, double damping);
};

}

#endif // gOcraFULLPOSTURETASKMANAGER_H
