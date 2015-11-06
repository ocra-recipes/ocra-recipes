#ifndef wOcraVARIABLEWEIGHTSTASKMANAGER_H
#define wOcraVARIABLEWEIGHTSTASKMANAGER_H

#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "ocra/control/Model.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/wOcraController.h"

#include <Eigen/Dense>

namespace wocra
{

/** \brief Task Manager for the cartesian position of a specified segment
 *
 */
class wOcraVariableWeightsTaskManager : public wOcraTaskManagerBase
{
    public:
        wOcraVariableWeightsTaskManager(wOcraController& ctrl,
                                        const wOcraModel& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        bool usesYarpPorts = false);

        wOcraVariableWeightsTaskManager(wOcraController& ctrl,
                                        const wOcraModel& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        const Eigen::Vector3d& segPoint_Local,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        bool usesYarpPorts = false);

        wOcraVariableWeightsTaskManager(wOcraController& ctrl,
                                        const wOcraModel& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        const Eigen::Vector3d& targetPose,
                                        bool usesYarpPorts = false);

        wOcraVariableWeightsTaskManager(wOcraController& ctrl,
                                        const wOcraModel& model,
                                        const std::string& taskName,
                                        const std::string& segmentName,
                                        const Eigen::Vector3d& segPoint_Local,
                                        double stiffness,
                                        double damping,
                                        Eigen::Vector3d weight,
                                        const Eigen::Vector3d& targetPose,
                                        bool usesYarpPorts = false);

        ~wOcraVariableWeightsTaskManager();

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
        void setWeights(Eigen::Vector3d weight);
        Eigen::VectorXd getWeights();

        // Task error
        Eigen::VectorXd getTaskError();

        Eigen::Vector3d getTaskFramePosition();
        


    private:
        std::vector<ocra::ECartesianDof>     axes;
        std::vector<std::string>             axesLabels;
        int                                  nDoF;
        std::vector<wocra::wOcraTask*>       tasks;

        const std::string&              segmentName;


        ocra::SegmentFrame*              featFrame;
        ocra::TargetFrame*               featDesFrame;

        void _init(const Eigen::Vector3d& refPoint_LocalFrame, double stiffness, double damping, Eigen::Vector3d weight);
};

}

#endif // wOcraVARIABLEWEIGHTSTASKMANAGER_H
