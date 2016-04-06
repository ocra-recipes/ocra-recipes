#ifndef SEGORIENTATIONTASKMANAGER_H
#define SEGORIENTATIONTASKMANAGER_H

#include "ocra/control/Model.h"
#include "ocra/control/TaskManagers/TaskManager.h"



#include <Eigen/Dense>

namespace ocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the  Controller
 *
 */
class SegOrientationTaskManager: public TaskManager
{
    public:
        SegOrientationTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping, double weight, bool usesYarpPorts = true);

        SegOrientationTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping, const Eigen::VectorXd& weight, bool usesYarpPorts = true);

        SegOrientationTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping, double weight, const Eigen::Rotation3d& targetPose, bool usesYarpPorts = true);

        SegOrientationTaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping, const Eigen::VectorXd& weight, const Eigen::Rotation3d& targetPose, bool usesYarpPorts = true);

        ~SegOrientationTaskManager();

        void setOrientation(const Eigen::Rotation3d& pos);

        void setDesiredState();

        // Task error


        // Yarp related:
        virtual const double * getCurrentState();
        virtual std::string getTaskManagerType();


    private:
        const std::string&              segmentName;

        ocra::OrientationFeature*        feat;
        ocra::SegmentFrame*              featFrame;
        ocra::OrientationFeature*        featDes;
        ocra::TargetFrame*               featDesFrame;

//        Eigen::Displacementd            _poseDes;

        void _init(Eigen::Rotation3d refOrientation_LocalFrame, double stiffness, double damping, double weight);
        void _init(Eigen::Rotation3d refOrientation_LocalFrame, double stiffness, double damping, const Eigen::VectorXd& weight);
};

}

#endif // SEGORIENTATIONTASKMANAGER_H
