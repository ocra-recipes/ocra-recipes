#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <yarp/os/Portable.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/PortReader.h>

#include <Eigen/Dense>
#include <Eigen/Lgsm>

#include <ocra/control/Model.h>
#include <ocra/util/Macros.h>


namespace ocra_recipes
{

/*! \class RobotState
 *  \brief A portable class for sending robot state information over yarp.
 *
 *  blah.
 */
class RobotState : public yarp::os::Portable
{
DEFINE_CLASS_POINTER_TYPEDEFS(RobotState)
public:
    RobotState ();
    RobotState (const int numberOfDoF);
    virtual ~RobotState ();

    virtual bool write(yarp::os::ConnectionWriter& connection);
    virtual bool read(yarp::os::ConnectionReader& connection);

    // friend std::ostream& operator<<(std::ostream &out, const RobotState& state);


public:
    Eigen::VectorXd              q;
    Eigen::VectorXd             qd;
    Eigen::Displacementd    H_root;
    Eigen::Twistd           T_root;

private:
    int nDoF;
};



/*! \class StateListener
 *  \brief A callback for a port listing to robot states.
 *
 *  Blah.
 */
class StateListener : public yarp::os::PortReader {
private:
    std::shared_ptr<ocra::Model> model;

public:
    StateListener();
    StateListener (std::shared_ptr<ocra::Model> modelPtr);
    virtual ~StateListener ();

    bool read(yarp::os::ConnectionReader& connection);
};

} /* ocra_recipes */
#endif
