#ifndef VARIABLEWEIGHTSTASKMANAGER_H
#define VARIABLEWEIGHTSTASKMANAGER_H

#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/Model.h"



#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for the cartesian position of a specified segment
 *
 */
class VariableWeightsTaskManager : public TaskManager
{
    public:
        VariableWeightsTaskManager(ocra::Controller& ctrl,
                                        const ocra::Model& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        bool usesYarpPorts = false);

        VariableWeightsTaskManager(ocra::Controller& ctrl,
                                        const ocra::Model& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        const Eigen::Vector3d& segPoint_Local,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        bool usesYarpPorts = false);

        VariableWeightsTaskManager(ocra::Controller& ctrl,
                                        const ocra::Model& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        const Eigen::Vector3d& targetPose,
                                        bool usesYarpPorts = false);

        VariableWeightsTaskManager(ocra::Controller& ctrl,
                                        const ocra::Model& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        const Eigen::Vector3d& segPoint_Local,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        const Eigen::Vector3d& targetPose,
                                        bool usesYarpPorts = false);

        ~VariableWeightsTaskManager();

        // Sets the position
        void setState(const Eigen::Vector3d& position);
        // Sets the position velocity and acceleration
        void setState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);



        // For objective tasks
        void setStiffness(double stiffness);
        double getStiffness();
        void setDamping(double damping);
        double getDamping();
        void setWeights(Eigen::Vector3d weight);
        Eigen::VectorXd getWeights();

        // Task error
        Eigen::VectorXd getTaskError();


        Eigen::Displacementd getTaskFrameDisplacement();
        Eigen::Twistd getTaskFrameVelocity();
        Eigen::Twistd getTaskFrameAcceleration();
        Eigen::Vector3d getTaskFramePosition();
        Eigen::Rotation3d getTaskFrameOrientation();
        Eigen::Vector3d getTaskFrameLinearVelocity();
        Eigen::Vector3d getTaskFrameAngularVelocity();
        Eigen::Vector3d getTaskFrameLinearAcceleration();
        Eigen::Vector3d getTaskFrameAngularAcceleration();



    private:
        std::vector<ocra::ECartesianDof>     axes;
        std::vector<std::string>             axesLabels;
        int                                  nDoF;

        const std::string&              segmentName;


        ocra::SegmentFrame*              featFrame;
        ocra::TargetFrame*               featDesFrame;

        void _init(const Eigen::Vector3d& refPoint_LocalFrame, double stiffness, double damping, Eigen::Vector3d weight);
};

}

#endif // VARIABLEWEIGHTSTASKMANAGER_H
