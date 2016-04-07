#include <ocra-recipes/TaskConnection.h>

using namespace ocra_recipes;

TaskConnection::TaskConnection()
{

}

TaskConnection::TaskConnection(const std::string& destinationTaskName)
: taskName(destinationTaskName)
{
    std::shared_ptr<ClientCommunications> ccComs = std::make_shared<ClientCommunications>();
    ccComs->open();
    taskRpcServerName = ccComs->getTaskPortName(taskName);
    ccComs->close();

    taskRpcClientName = "/TaskConnection/"+taskName+"/rpc:o";
    taskRpcClient.open(taskRpcClientName.c_str());

    yarp::os::Network yarp;
    yarp.connect(taskRpcClientName.c_str(), taskRpcServerName.c_str());
}

TaskConnection::~TaskConnection()
{
    taskRpcClient.close();
}

bool TaskConnection::activate()
{
    yarp::os::Bottle message, reply;
    message.addInt(ocra::TASK_MESSAGE::ACTIVATE);
    taskRpcClient.write(message, reply);
    if(reply.get(0).asInt() == ocra::TASK_MESSAGE::OCRA_FAILURE) {
        yLog.error() << "Could not activate " << taskName;
    }
}

bool TaskConnection::deactivate()
{
    yarp::os::Bottle message, reply;
    message.addInt(ocra::TASK_MESSAGE::DEACTIVATE);
    taskRpcClient.write(message, reply);
    if(reply.get(0).asInt() == ocra::TASK_MESSAGE::OCRA_FAILURE) {
        yLog.error() << "Could not deactivate " << taskName;
    }
}

std::string TaskConnection::getPortName()
{
    yarp::os::Bottle message, reply;
    message.addInt(ocra::TASK_MESSAGE::GET_TASK_PORT_NAME);
    taskRpcClient.write(message, reply);
    return reply.get(0).asString();
}

bool TaskConnection::isActivated()
{
    yarp::os::Bottle message, reply;
    message.addInt(ocra::TASK_MESSAGE::GET_ACTIVITY_STATUS);
    taskRpcClient.write(message, reply);
    if(reply.get(0).asInt() == ocra::TASK_MESSAGE::TASK_IS_ACTIVATED) {
        return true;
    } else {
        return false;
    }
}

Eigen::VectorXd TaskConnection::getTaskError()
{
    yarp::os::Bottle message, reply;
    message.addInt(ocra::TASK_MESSAGE::GET_ACTIVITY_STATUS);
    taskRpcClient.write(message, reply);
    if(reply.get(0).asInt() == ocra::TASK_MESSAGE::OCRA_SUCCESS) {
        int dummy;
        return ocra::pourBottleIntoEigenVector(reply.tail(), dummy);
    } else {
        return Eigen::VectorXd::Zero(0);
    }
}

double TaskConnection::getTaskErrorNorm()
{

}

void TaskConnection::setStiffness(double K)
{

}

void TaskConnection::setStiffness(const VectorXd& K)
{

}

void TaskConnection::setStiffness(const MatrixXd& K)
{

}

double TaskConnection::getStiffness()
{

}

Eigen::MatrixXd TaskConnection::getStiffnessMatrix()
{

}

void TaskConnection::setDamping(double B)
{

}

void TaskConnection::setDamping(const VectorXd& B)
{

}

void TaskConnection::setDamping(const MatrixXd& B)
{

}

double TaskConnection::getDamping()
{

}

Eigen::MatrixXd TaskConnection::getDampingMatrix()
{

}

void TaskConnection::setWeight(double weight)
{

}

void TaskConnection::setWeight(Eigen::VectorXd& weights)
{

}

Eigen::VectorXd TaskConnection::getWeight()
{

}


Eigen::Displacementd TaskConnection::getTaskFrameDisplacement()
{

}

Eigen::Twistd TaskConnection::getTaskFrameVelocity()
{

}

Eigen::Twistd TaskConnection::getTaskFrameAcceleration()
{

}

Eigen::Vector3d TaskConnection::getTaskFramePosition()
{

}

Eigen::Rotation3d TaskConnection::getTaskFrameOrientation()
{

}

Eigen::Vector3d TaskConnection::getTaskFrameLinearVelocity()
{

}

Eigen::Vector3d TaskConnection::getTaskFrameAngularVelocity()
{

}

Eigen::Vector3d TaskConnection::getTaskFrameLinearAcceleration()
{

}

Eigen::Vector3d TaskConnection::getTaskFrameAngularAcceleration()
{

}
