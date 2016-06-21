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

bool TaskState::extractFromBottle(const yarp::os::Bottle& bottle, int& sizeOfState)
{
    int i = 0;
    if (bottle.get(i).asInt() == TASK_STATE_BOTTLE)
    {
        ++i; containsPosition = bottle.get(i).asBool();
        ++i; containsVelocity = bottle.get(i).asBool();
        ++i; containsAcceleration = bottle.get(i).asBool();
        ++i; containsQ = bottle.get(i).asBool();
        ++i; containsQd = bottle.get(i).asBool();
        ++i; containsQdd = bottle.get(i).asBool();
        ++i; containsTorque = bottle.get(i).asBool();
        ++i; containsWrench = bottle.get(i).asBool();
        int indexesToSkip;


        if (this->hasPosition()) {
            this->setPosition( util::pourBottleIntoDisplacementd(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }
        if (this->hasVelocity()) {
            this->setVelocity( util::pourBottleIntoTwistd(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }
        if (this->hasAcceleration()) {
            this->setAcceleration( util::pourBottleIntoTwistd(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }
        if (this->hasQ()) {
            this->setQ( util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }
        if (this->hasQd()) {
            this->setQd( util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }
        if (this->hasQdd()) {
            this->setQdd( util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }
        if (this->hasTorque()) {
            this->setTorque( util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }
        if (this->hasWrench()) {
            this->setWrench( util::pourBottleIntoWrenchd(util::trimBottle(bottle, i+1), indexesToSkip) );
            i += indexesToSkip;
        }

        sizeOfState = i;
        return true;
    }
    return false;

}

void TaskState::putIntoBottle(yarp::os::Bottle& bottle)
{
    bottle.addInt(TASK_STATE_BOTTLE);

    bottle.addInt(this->hasPosition());
    bottle.addInt(this->hasVelocity());
    bottle.addInt(this->hasAcceleration());
    bottle.addInt(this->hasQ());
    bottle.addInt(this->hasQd());
    bottle.addInt(this->hasQdd());
    bottle.addInt(this->hasTorque());
    bottle.addInt(this->hasWrench());

    if (this->hasPosition()) {
        util::pourDisplacementdIntoBottle(this->position, bottle);
    }
    if (this->hasVelocity()) {
        util::pourTwistdIntoBottle(this->velocity, bottle);
    }
    if (this->hasAcceleration()) {
        util::pourTwistdIntoBottle(this->acceleration, bottle);
    }
    if (this->hasQ()) {
        util::pourEigenVectorIntoBottle(this->q, bottle);
    }
    if (this->hasQd()) {
        util::pourEigenVectorIntoBottle(this->qd, bottle);
    }
    if (this->hasQdd()) {
        util::pourEigenVectorIntoBottle(this->qdd, bottle);
    }
    if (this->hasTorque()) {
        util::pourEigenVectorIntoBottle(this->torque, bottle);
    }
    if (this->hasWrench()) {
        util::pourWrenchdIntoBottle(this->wrench, bottle);
    }

}
