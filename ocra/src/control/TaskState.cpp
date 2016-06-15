#include <ocra/control/TaskState.h>

using namespace ocra;

TaskState::TaskState()
: containsPosition(false)
, containsVelocity(false)
, containsAcceleration(false)
, containsQ(false)
, containsQd(false)
, containsQdd(false)
, containsTorque(false)
, containsWrench(false)
{
    // Do nothing
}

TaskState::~TaskState()
{
    // Do nothing
}

Eigen::Displacementd TaskState::getPosition() const
{
    return this->position;
}


Eigen::Twistd TaskState::getVelocity() const
{
    return this->velocity;
}


Eigen::Twistd TaskState::getAcceleration() const
{
    return this->acceleration;
}


Eigen::VectorXd TaskState::getQ() const
{
    return this->q;
}


Eigen::VectorXd TaskState::getQd() const
{
    return this->qd;
}


Eigen::VectorXd TaskState::getQdd() const
{
    return this->qdd;
}


Eigen::VectorXd TaskState::getTorque() const
{
    return this->torque;
}


Eigen::Wrenchd TaskState::getWrench() const
{
    return this->wrench;
}


void TaskState::setPosition(const Eigen::Displacementd& newPosition)
{
    this->position = newPosition;
    this->containsPosition = true;
}


void TaskState::setVelocity(const Eigen::Twistd& newVelocity)
{
    this->velocity = newVelocity;
    this->containsVelocity = true;
}


void TaskState::setAcceleration(const Eigen::Twistd& newAcceleration)
{
    this->acceleration = newAcceleration;
    this->containsAcceleration = true;
}


void TaskState::setQ(const Eigen::VectorXd& newQ)
{
    this->q = newQ;
    this->containsQ = true;
}


void TaskState::setQd(const Eigen::VectorXd& newQd)
{
    this->qd = newQd;
    this->containsQd = true;
}


void TaskState::setQdd(const Eigen::VectorXd& newQdd)
{
    this->qdd = newQdd;
    this->containsQdd = true;
}


void TaskState::setTorque(const Eigen::VectorXd& newTorque)
{
    this->torque = newTorque;
    this->containsTorque = true;
}


void TaskState::setWrench(const Eigen::Wrenchd& newWrench)
{
    this->wrench = newWrench;
    this->containsWrench = true;
}

bool TaskState::hasPosition() const
{
    return this->containsPosition;
}

bool TaskState::hasVelocity() const
{
    return this->containsVelocity;
}

bool TaskState::hasAcceleration() const
{
    return this->containsAcceleration;
}

bool TaskState::hasQ() const
{
    return this->containsQ;
}

bool TaskState::hasQd() const
{
    return this->containsQd;
}

bool TaskState::hasQdd() const
{
    return this->containsQdd;
}

bool TaskState::hasTorque() const
{
    return this->containsTorque;
}

bool TaskState::hasWrench() const
{
    return this->containsWrench;
}
