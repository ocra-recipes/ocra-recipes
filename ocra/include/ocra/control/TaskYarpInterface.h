#ifndef TASK_YARP_INTERFACE_H
#define TASK_YARP_INTERFACE_H


#include "ocra/util/Macros.h"
#include "ocra/util/ErrorsHelper.h"
#include "ocra/control/Task.h"
#include "ocra/control/TaskState.h"
#include "ocra/control/TaskYarpInterfaceVocab.h"
#include "ocra/control/Trajectory/Trajectories.h"
#include "ocra/util/ErrorsHelper.h"
#include <Eigen/Dense>

#include <yarp/os/Network.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace ocra
{

/*! \enum TASK_MODE
 *  \brief A basic enumeration for the different types of tasks we can have.
 */
enum TASK_MODE
{
    TASK_AS_OBJECTIVE,
    TASK_AS_CONSTRAINT,
    TASK_NOT_DEFINED
};


/*! \class TaskYarpInterface
 *
 *  a long description
 */
class TaskYarpInterface
{
    DEFINE_CLASS_POINTER_TYPEDEFS(TaskYarpInterface)
public:

    /*! Constructor.
     *  \param ctrl A reference to an ocra controller.
     *  \param model A reference to and ocra model.
     *  \param name The name of the task.
     *  \param name The hierarchy level for Hocra
     *  \param usesYarpPorts Whether or not to use yarp for interprocess communication with the task. True by defualt.
     */
    TaskYarpInterface(Task::Ptr taskPtr);

    /*! Destructor. Mostly just closes yarp ports and threads.
     */
    virtual ~TaskYarpInterface();

    /*! Activates the underlying task(s)
     *  \note \ref `deactivate()` must be called before this function to work properly.
     *
     *  \return A boolean indicating the success of the operation.
     */
    bool activate();

    /*! Activates a single task as either an objective or a constraint.
     *  \note \ref `deactivate()` must be called before this function to work properly.
     *
     *  \return A boolean indicating the success of the operation.
     */
    bool activate(const TASK_MODE tmode);

    /*! Deactivates the underlying task(s). This is where we call \ref`getTaskMode()` to see if the task in an objective or a constraint.
     *
     *  \return A boolean indicating the success of the operation.
     */
    bool deactivate();

    /*! Gets the current task mode. Either objective, constraint or undefined.
     *  \param tsk A pointer to a single task.
     *
     *  \return The mode of that task.
     */
    TASK_MODE getTaskMode();

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
        /*! Gets the defined Level for a task
     *  \warn Cannot be changed at runtime .
     *
     *  \return The level.
     */
    int getTaskHierarchyLevel();
    void setTaskHierarchyLevel(int level);
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
    void setWeight(const Eigen::VectorXd& weights);

    /*! Gets the weights associated with the task. Often a task will have a single weight but it is still represented as a vector to be applied to each DoF of the task.
     *
     *  \return The vector of task weights.
     */
    Eigen::VectorXd getWeight();



    TaskState getTaskState();
    TaskState getDesiredTaskState();
    void setDesiredTaskState(const TaskState& newDesiredTaskState);
    void setDesiredTaskStateDirect(const TaskState& newDesiredTaskState);
    void publishTaskState();
private:
//     yarp::os::Mutex getTaskStateMutex;


public: /* Segment Frame Based Tasks */

    // TODO: Document...
    Eigen::Displacementd getTaskFrameDisplacement();
    Eigen::Twistd getTaskFrameVelocity();
    Eigen::Twistd getTaskFrameAcceleration();
    Eigen::Vector3d getTaskFramePosition();
    Eigen::Rotation3d getTaskFrameOrientation();
    Eigen::Vector3d getTaskFrameLinearVelocity();
    Eigen::Vector3d getTaskFrameAngularVelocity();
    Eigen::Vector3d getTaskFrameLinearAcceleration();
    Eigen::Vector3d getTaskFrameAngularAcceleration();




public: /*Nested callback classes */

    /*! \class RpcMessageCallback
     *  \brief a short description
     *
     *  a long description
     */
    class RpcMessageCallback : public yarp::os::PortReader {
        DEFINE_CLASS_POINTER_TYPEDEFS(RpcMessageCallback)

    private:
        TaskYarpInterface& tmBase;

    public:
        RpcMessageCallback(TaskYarpInterface& tmBaseRef);

        virtual bool read(yarp::os::ConnectionReader& connection);
    };

    /*! \class ControlInputCallback
     *  \brief a short description
     *
     *  a long description
     */
    class ControlInputCallback : public yarp::os::PortReader {
        DEFINE_CLASS_POINTER_TYPEDEFS(ControlInputCallback)

    private:
        TaskYarpInterface& tmBase;
        yarp::os::Bottle input;

    public:
        ControlInputCallback(TaskYarpInterface& tmBaseRef);

        virtual bool read(yarp::os::ConnectionReader& connection);
    };

    /*! \class StateUpdateThread
     *  \brief a short description
     *
     *  a long description
     */
    class StateUpdateThread : public yarp::os::RateThread
    {
        DEFINE_CLASS_POINTER_TYPEDEFS(StateUpdateThread)

    private:
        TaskYarpInterface& tmBase;


    public:
        StateUpdateThread(int period, TaskYarpInterface& tmBaseRef);
        bool threadInit();
        void run();
        void threadRelease();
    };


private: /* Private methods */
    // For parsing and compiling yarp messages.
    void parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply);
    std::string printValidMessageTags();
    bool openControlPorts();
    bool closeControlPorts();
    bool parseControlInput(yarp::os::Bottle& input);


private:
    Task::Ptr                   task;
    TASK_MODE                   taskMode;
    yarp::os::Log               yLog;

    yarp::os::Network           yarp;
    yarp::os::RpcServer         rpcPort;
    std::string                 portName;
    RpcMessageCallback::Ptr     rpcCallback;

    bool                        controlPortsOpen;

    yarp::os::Bottle            stateInBottle;
    std::string                 inputControlPortName;
    yarp::os::Port              inputControlPort;
    ControlInputCallback::Ptr   controlCallback;

    yarp::os::Bottle            stateOutBottle;
    std::string                 outputControlPortName;
    yarp::os::Port              outputControlPort;

    yarp::os::Bottle            desiredStateOutBottle;
    std::string                 desiredStateOutputPortName;
    yarp::os::Port              desiredStateOutputPort;




    StateUpdateThread::Ptr      stateThread;
    int numberOfOpenRequests;
    

    bool logMessages;

};

}

#endif // TASK_YARP_INTERFACE_H
