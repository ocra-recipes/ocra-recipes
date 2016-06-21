#ifndef TASK_STATE_H
#define TASK_STATE_H

#include <Eigen/Core>
#include <Eigen/Lgsm>
#include <ocra/util/Macros.h>
#include <ocra/util/YarpUtilities.h>
#include <ocra/control/TaskYarpInterfaceVocab.h>
#include <yarp/os/Bottle.h>

namespace ocra {


class TaskState {
DEFINE_CLASS_POINTER_TYPEDEFS(TaskState)

private:
    Eigen::Displacementd position;
    Eigen::Twistd velocity;
    Eigen::Twistd acceleration;

    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
    Eigen::VectorXd torque;

    Eigen::Wrenchd wrench;


    bool containsPosition;
    bool containsVelocity;
    bool containsAcceleration;
    bool containsQ;
    bool containsQd;
    bool containsQdd;
    bool containsTorque;
    bool containsWrench;

    static const int TASK_STATE_BOTTLE = 12345;

public:
    TaskState();
    virtual ~TaskState();

    Eigen::Displacementd getPosition() const;
    Eigen::Twistd getVelocity() const;
    Eigen::Twistd getAcceleration() const;
    Eigen::VectorXd getQ() const;
    Eigen::VectorXd getQd() const;
    Eigen::VectorXd getQdd() const;
    Eigen::VectorXd getTorque() const;
    Eigen::Wrenchd getWrench() const;

    void setPosition(const Eigen::Displacementd& newPosition);
    void setVelocity(const Eigen::Twistd& newVelocity);
    void setAcceleration(const Eigen::Twistd& newAcceleration);
    void setQ(const Eigen::VectorXd& newQ);
    void setQd(const Eigen::VectorXd& newQd);
    void setQdd(const Eigen::VectorXd& newQdd);
    void setTorque(const Eigen::VectorXd& newTorque);
    void setWrench(const Eigen::Wrenchd& newWrench);

    bool hasPosition() const ;
    bool hasVelocity() const ;
    bool hasAcceleration() const ;
    bool hasQ() const ;
    bool hasQd() const ;
    bool hasQdd() const ;
    bool hasTorque() const ;
    bool hasWrench() const ;


    bool extractFromBottle(const yarp::os::Bottle& bottle, int& sizeOfOptions);
    void putIntoBottle(yarp::os::Bottle& bottle);



};

} // namespace ocra
#endif // TASK_STATE_H
