#ifndef wOcraPARTIALPOSTURETASKMANAGER_H
#define wOcraPARTIALPOSTURETASKMANAGER_H

#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "ocra/control/Model.h"


// #include "wocra/Features/wOcraFeature.h"
#include "ocra/control/Feature.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for partal joint posture task
 *
 */
class wOcraPartialPostureTaskManager : public wOcraTaskManagerBase
{
    public:
        wOcraPartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        wOcraPartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = false);

        wOcraPartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight,
            Eigen::VectorXd& init_q, bool usesYarpPorts = true);

        wOcraPartialPostureTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, const Eigen::VectorXd& weight,
            Eigen::VectorXd& init_q, bool usesYarpPorts = false);

        ~wOcraPartialPostureTaskManager();

        void setPosture(Eigen::VectorXd& q);
        void setPosture(Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot);
/*
        void setStiffnessDamping(double stiffness, double damping);
*/

        void setStiffness(double stiffness);
        double getStiffness();
        void setDamping(double damping);
        double getDamping();
        void setWeight(double weight);
        void setWeight(const Eigen::VectorXd& weight);
        Eigen::VectorXd getWeight();
        void setDesiredState();


        void activate();
        void deactivate();

        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();
        virtual bool checkIfActivated();

    private:
        ocra::PartialStateFeature*           feat;
        ocra::PartialModelState*             featState;

        ocra::PartialStateFeature*           featDes;
        ocra::PartialTargetState*            featDesState;

/*
        Eigen::Vector3d                 _posDes;
        Eigen::Vector3d                 _velDes;
        Eigen::Vector3d                 _accDes;
*/

        void _init(int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, double weight);
        void _init(int fullStateType, Eigen::VectorXi& dofIndices, double stiffness, double damping, const Eigen::VectorXd& weight);
};

}

#endif // wOcraFULLPOSTURETASKMANAGER_H
