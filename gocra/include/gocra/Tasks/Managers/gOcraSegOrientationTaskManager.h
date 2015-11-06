#ifndef gOcraSEGORIENTATIONTASKMANAGER_H
#define gOcraSEGORIENTATIONTASKMANAGER_H

#include "ocra/control/Model.h"
#include "gocra/Tasks/gOcraTaskManagerBase.h"
#include "gocra/Tasks/GHCJTTask.h"
#include "gocra/GHCJTController.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the gOcra Controller
 *
 */
class gOcraSegOrientationTaskManager: public gOcraTaskManagerBase
{
    public:
        gOcraSegOrientationTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping);

        gOcraSegOrientationTaskManager(GHCJTController& ctrl, const gOcraModel& model, const std::string& taskName, const std::string& segmentName, double stiffness, double damping, const Eigen::Rotation3d& targetPose);

        ~gOcraSegOrientationTaskManager();

        void setOrientation(const Eigen::Rotation3d& pos);

        // All Managers have this
        void activate();
        void deactivate();

        // For objective tasks
        void setStiffness(double stiffness);
        double getStiffness();
        void setDamping(double damping);
        double getDamping();


        // Task error
        Eigen::VectorXd getTaskError();
 
    private:
        gocra::GHCJTTask*              task;
        const std::string&              segmentName;

        ocra::OrientationFeature*        feat;
        ocra::SegmentFrame*              featFrame;
        ocra::OrientationFeature*        featDes;
        ocra::TargetFrame*               featDesFrame;

//        Eigen::Displacementd            _poseDes;

        void _init(Eigen::Rotation3d refOrientation_LocalFrame, double stiffness, double damping);
};

}

#endif // gOcraSEGORIENTATIONTASKMANAGER_H
