#ifndef TASK_STATE_H
#define TASK_STATE_H

#include <Eigen/Core>
#include <Eigen/Lgsm>
#include "ocra/utilities.h"


namespace ocra {


class TaskState {
DEFINE_CLASS_POINTER_TYPEDEFS(TaskState)

public:
    Eigen::Displacementd position;
    Eigen::Twistd velocity;
    Eigen::Twistd acceleration;

    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
    Eigen::VectorXd torque;

    Eigen::Wrenchd wrench;

public:
    TaskState()
    {
        //
    }
    virtual ~TaskState()
    {
        
    }

};

} // namespace ocra
#endif // TASK_STATE_H
