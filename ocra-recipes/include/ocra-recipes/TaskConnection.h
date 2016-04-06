#ifndef TASK_CONNECTION_H
#define TASK_CONNECTION_H

#include <iostream>

#include <ocra/control/TaskManagers/TaskManager.h>
#include <ocra/control/TaskManagers/TaskManagerMessageVocab.h>

#include <yarp/os/RpcClient.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <ocra-recipes/ClientCommunications.h>

namespace ocra_recipes {

class TaskConnection {

private:
    yarp::os::Log yLog;                 /*!< Yarp logging tool. */
    yarp::os::RpcClient taskRpcClient;  /*!< Task rpc client. */
    std::string taskName;               /*!< Name of the task we are connecting to. */
    std::string taskRpcServerName;      /*!< Name of the rpc server port. */
    std::string taskRpcClientName;      /*!< Name of the rpc client port. */

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

    Eigen::Displacementd getTaskFrameDisplacement();
    Eigen::Twistd getTaskFrameVelocity();
    Eigen::Twistd getTaskFrameAcceleration();
    Eigen::Vector3d getTaskFramePosition();
    Eigen::Rotation3d getTaskFrameOrientation();
    Eigen::Vector3d getTaskFrameLinearVelocity();
    Eigen::Vector3d getTaskFrameAngularVelocity();
    Eigen::Vector3d getTaskFrameLinearAcceleration();
    Eigen::Vector3d getTaskFrameAngularAcceleration();

};


} /* ocra_recipes */

#endif // TASK_CONNECTION_H
