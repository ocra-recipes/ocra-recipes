#include <ocra-recipes/RobotState.h>

using namespace ocra_recipes;


RobotState::RobotState()
{
}

RobotState::RobotState(const int numberOfDoF)
: nDoF(numberOfDoF)
{
    q.resize(nDoF);
    qd.resize(nDoF);
}

RobotState::~RobotState()
{
}

std::ostream& operator<<(std::ostream &out, const RobotState& state)
{
    std::cout << "q \t | \t qd" << std::endl;
    for(auto i=0; i<state.q.size(); ++i)
    {
        out << state.q(i) << "\t | \t" << state.qd(i) << std::endl;
    }

    out << "x" << " " << "y" << " " << "z" << " " << "qx" << " " << "qy" << " " << "qz" << " " << "qw" << std::endl;
    out << state.H_root.x() << " ";
    out << state.H_root.y() << " ";
    out << state.H_root.z() << " ";
    out << state.H_root.qx() << " ";
    out << state.H_root.qy() << " ";
    out << state.H_root.qz() << " ";
    out << state.H_root.qw() << std::endl;

    out << "rx" << " " << "ry" << " " << "rz" << " " << "rx" << " " << "ry" << " " << "rz" << std::endl;
    // out << state.T_root.rx() << " ";
    // out << state.T_root.ry() << " ";
    // out << state.T_root.rz() << " ";
    // out << state.T_root.vx() << " ";
    // out << state.T_root.vy() << " ";
    // out << state.T_root.vz() << std::endl;
    out << state.T_root << std::endl;

    return out;
}

bool RobotState::write(yarp::os::ConnectionWriter& connection)
{
    for(auto i=0; i<q.size(); ++i)
    {
        connection.appendDouble(q(i));
        connection.appendDouble(qd(i));
    }
    connection.appendDouble(H_root.x());
    connection.appendDouble(H_root.y());
    connection.appendDouble(H_root.z());
    connection.appendDouble(H_root.qx());
    connection.appendDouble(H_root.qy());
    connection.appendDouble(H_root.qz());
    connection.appendDouble(H_root.qw());

    connection.appendDouble(T_root.rx());
    connection.appendDouble(T_root.ry());
    connection.appendDouble(T_root.rz());
    connection.appendDouble(T_root.vx());
    connection.appendDouble(T_root.vy());
    connection.appendDouble(T_root.vz());
    return true;
}

bool RobotState::read(yarp::os::ConnectionReader& connection)
{

    for(auto i=0; i<q.size(); ++i)
    {
        q(i) = connection.expectDouble();
        qd(i) = connection.expectDouble();
    }
    H_root.x() = connection.expectDouble();
    H_root.y() = connection.expectDouble();
    H_root.z() = connection.expectDouble();
    H_root.qx() = connection.expectDouble();
    H_root.qy() = connection.expectDouble();
    H_root.qz() = connection.expectDouble();
    H_root.qw() = connection.expectDouble();

    T_root.rx() = connection.expectDouble();
    T_root.ry() = connection.expectDouble();
    T_root.rz() = connection.expectDouble();
    T_root.vx() = connection.expectDouble();
    T_root.vy() = connection.expectDouble();
    T_root.vz() = connection.expectDouble();

    return !connection.isError();
}


/**************************************************************************************************
                                    StateListener Class
**************************************************************************************************/
StateListener::StateListener()
{
}

StateListener::~StateListener()
{
}

StateListener::StateListener(std::shared_ptr<ocra::Model> modelPtr)
: model(modelPtr)
{
    //do nothing
}

bool StateListener::read(yarp::os::ConnectionReader& connection)
{
    RobotState state;

    if (!state.read(connection)){
        return false;
    }
    else{
        std::cout << state << std::endl;
        model->setState(state.H_root, state.q, state.T_root, state.qd);
        return true;
    }
}
