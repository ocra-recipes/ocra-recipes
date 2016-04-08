#ifndef TASK_CONNECTION_H
#define TASK_CONNECTION_H

#include <iostream>

#include <ocra/control/TaskManagers/TaskManager.h>
#include <ocra/control/TaskManagers/TaskManagerMessageVocab.h>

#include <yarp/os/RpcClient.h>
#include <yarp/os/Network.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <ocra-recipes/ClientCommunications.h>

namespace ocra_recipes {

class TaskConnection {

private:
    yarp::os::Network yarp;             /*!< Yarp network instance. */
    yarp::os::Log yLog;                 /*!< Yarp logging tool. */
    yarp::os::RpcClient taskRpcClient;  /*!< Task rpc client. */
    std::string taskName;               /*!< Name of the task we are connecting to. */
    std::string taskRpcServerName;      /*!< Name of the rpc server port. */
    std::string taskRpcClientName;      /*!< Name of the rpc client port. */

    std::string taskOutputPortName;
    std::string taskInputPortName;
    std::string inputPortName;
    std::string outputPortName;

    yarp::os::Port inputPort;
    yarp::os::Port outputPort;

    Eigen::VectorXd currentStateVector;
    bool controlPortsAreOpen;


private:
    void parseInput(yarp::os::Bottle& input);


public:
    TaskConnection ();
    TaskConnection (const std::string& destinationTaskName);
    virtual ~TaskConnection ();

    /*! Activates the underlying task(s)
     *  \note \ref `deactivate()` must be called before this function to work properly.
     *
     *  \return A boolean indicating the success of the operation.
     */
    bool activate();

    /*! Deactivates the underlying task(s). This is where we call \ref`getTaskMode()` to see if the task in an objective or a constraint.
     *
     *  \return A boolean indicating the success of the operation.
     */
    bool deactivate();

    /*! Gets the name of the task manager's RPC server port.
     *
     *  \return The port's full name as a string.
     */
    std::string getPortName();

    /*! Check if the task is active as either an objective or a constraint.
     *
     *  \return A boolean indicating the task(s) activity.
     */
    bool isActivated();

    /*! Gets the current error vector of the task.
     *  \warn Only valid for task managers with 1 task, not sets of tasks.
     *
     *  \return A vector of errors in each DoF of the task.
     */
    Eigen::VectorXd getTaskError();

    /*! Gets the current error vector norm of the task.
     *  \warn Only valid for task managers with 1 task, not sets of tasks.
     *
     *  \return The norm of the task error vector.
     */
    double getTaskErrorNorm();

    /*! Sets the stiffness, or proportional (Kp) gain, of the task.
     *  \param K The proportional gain to set. Fills a diagonal matrix.
     */
    void setStiffness(double K);

    /*! Sets the stiffness, or proportional (Kp) gains, of the task.
     *  \param K The proportional gains to set for each DoF. Creates a diagonal matrix from K.
     */
    void setStiffness(const VectorXd& K);


    /*! Sets the stiffness, or proportional (Kp) gains, of the task.
     *  \param K The proportional gains to set for the task. Off diagonal components will create stiffness correlations between the task DoF.
     */
    void setStiffness(const MatrixXd& K);

    /*! Gets the current stiffness, or proportional (Kp) gain, of the task.
     *
     *  \return The first diagonal entry of the stiffness matrix.
     */
    double getStiffness();

    /*! Gets the current stiffness, or proportional (Kp) gain, of the task.
     *
     *  \return The stiffness matrix.
     */
    Eigen::MatrixXd getStiffnessMatrix();

    /*! Sets the damping, or damping (Kd) gain, of the task.
     *  \param K The damping gain to set. Fills a diagonal matrix.
     */
    void setDamping(double B);

    /*! Sets the damping, or damping (Kd) gains, of the task.
     *  \param K The damping gains to set for each DoF. Creates a diagonal matrix from K.
     */
    void setDamping(const VectorXd& B);


    /*! Sets the damping, or damping (Kd) gains, of the task.
     *  \param K The damping gains to set for the task. Off diagonal components will create damping correlations between the task DoF.
     */
    void setDamping(const MatrixXd& B);

    /*! Gets the current damping, or damping (Kd) gain, of the task.
     *
     *  \return The first diagonal entry of the damping matrix.
     */
    double getDamping();

    /*! Gets the current damping, or damping (Kd) gain, of the task.
     *
     *  \return The damping matrix.
     */
    Eigen::MatrixXd getDampingMatrix();


    /*! Sets the weight for all DoF of a task.
     *  \param weight The weight to apply to all task DoF.
     */
    void setWeight(double weight);

    /*! Sets the weights for each DoF of a task.
     *  \param weights A vector of weights to apply to each task DoF individually.
     */
    void setWeight(Eigen::VectorXd& weights);

    /*! Gets the weights associated with the task. Often a task will have a single weight but it is still represented as a vector to be applied to each DoF of the task.
     *
     *  \return The vector of task weights.
     */
    Eigen::VectorXd getWeight();

    /*! Opens the high speed task control ports.
     *
     *  \return True if the ports open successfully and are connected.
     */
    bool openControlPorts();

    /*! If the control ports are open then this will return the current task state vector.
     *
     *  \return The task's current state.
     */
    Eigen::VectorXd getCurrentState();

    /*! Gets the current task state vector via a message over the Rpc port. This should not be used for high speed querying.
     *
     *  \return The task's current state.
     */
    Eigen::VectorXd getCurrentStateRpc();

    /*! The name of the task we are connected to.
     *
     *  \return The name of the task we are connected to.
     */
    std::string getTaskName(){return taskName;}

    int getTaskDimension();

    int getTaskStateDimension();

    void sendDesiredStateAsBottle(yarp::os::Bottle& bottle);

    bool closeControlPorts();



    Eigen::Displacementd getTaskFrameDisplacement();
    Eigen::Twistd getTaskFrameVelocity();
    Eigen::Twistd getTaskFrameAcceleration();
    Eigen::Vector3d getTaskFramePosition();
    Eigen::Rotation3d getTaskFrameOrientation();
    Eigen::Vector3d getTaskFrameLinearVelocity();
    Eigen::Vector3d getTaskFrameAngularVelocity();
    Eigen::Vector3d getTaskFrameLinearAcceleration();
    Eigen::Vector3d getTaskFrameAngularAcceleration();

    /************** controlInputCallback *************/
    class inputCallback : public yarp::os::PortReader {
        private:
            TaskConnection& tcRef;

        public:
            inputCallback(TaskConnection& _tcRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
    };
    /************** controlInputCallback *************/

private:
    std::shared_ptr<inputCallback> inpCallback;

};
} /* ocra_recipes */

#endif // TASK_CONNECTION_H
