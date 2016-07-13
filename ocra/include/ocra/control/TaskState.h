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
    void putIntoBottle(yarp::os::Bottle& bottle) const;



    friend std::ostream& operator<<(std::ostream &out, const TaskState& state)
    {
        if(state.hasPosition())
            out << "getPosition():\n" << state.getPosition() << std::endl << std::endl;

        if(state.hasVelocity())
            out << "getVelocity():\n" << state.getVelocity() << std::endl << std::endl;

        if(state.hasAcceleration())
            out << "getAcceleration():\n" << state.getAcceleration() << std::endl << std::endl;

        if(state.hasQ())
            out << "getQ():\n" << state.getQ() << std::endl << std::endl;

        if(state.hasQd())
            out << "getQd():\n" << state.getQd() << std::endl << std::endl;

        if(state.hasQdd())
            out << "getQdd():\n" << state.getQdd() << std::endl << std::endl;

        if(state.hasTorque())
            out << "getTorque():\n" << state.getTorque() << std::endl << std::endl;

        if(state.hasWrench())
            out << "getWrench():\n" << state.getWrench() << std::endl << std::endl;


        return out;
    }

};

} // namespace ocra
#endif // TASK_STATE_H
