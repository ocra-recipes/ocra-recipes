#ifndef TASKMANAGERBASE_H
#define TASKMANAGERBASE_H

#include <memory>

#include "ocra/control/Model.h"
#include "ocra/control/Controller.h"
#include "ocra/control/Tasks/OneLevelTask.h"
#include "ocra/control/Trajectory/Trajectories.h"

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


/*! \class TaskManager
 *  \brief A factory base class which facilitates the construction of common task types.
 *
 *  a long description
 */
class TaskManager
{
public:

    /*! Constructor.
     *  \param ctrl A reference to an ocra controller.
     *  \param model A reference to and ocra model.
     *  \param name The name of the task.
     *  \param usesYarpPorts Whether or not to use yarp for interprocess communication with the task. True by defualt.
     */
    TaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& name, bool usesYarpPorts=true);

    /*! Destructor. Mostly just closes yarp ports and threads.
     */
    virtual ~TaskManager();

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
    bool activate(std::shared_ptr<Task> tsk, const TASK_MODE tmode);

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
    TASK_MODE getTaskMode(std::shared_ptr<Task> tsk);

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

    double getTaskErrorNorm();

public: /*Nested callback classes */

    /*! \class RpcMessageCallback
     *  \brief a short description
     *
     *  a long description
     */
    class RpcMessageCallback : public yarp::os::PortReader {
    private:
        TaskManager& tmBase;

    public:
        RpcMessageCallback(TaskManager& tmBaseRef);

        virtual bool read(yarp::os::ConnectionReader& connection);
    };

    /*! \class ControlInputCallback
     *  \brief a short description
     *
     *  a long description
     */
    class ControlInputCallback : public yarp::os::PortReader {
    private:
        TaskManager& tmBase;

    public:
        ControlInputCallback(TaskManager& tmBaseRef);

        virtual bool read(yarp::os::ConnectionReader& connection);
    };

    /*! \class StateUpdateThread
     *  \brief a short description
     *
     *  a long description
     */
    class StateUpdateThread : public yarp::os::RateThread
    {
    private:
        TaskManager& tmBase;

    public:
        StateUpdateThread(int period, TaskManager& tmBaseRef);
        bool threadInit();
        void run();
        void threadRelease();
    };

public: /* Public virtual methods */

    virtual std::string getTaskManagerType();
    virtual VectorXd getTaskError();

    virtual void setStiffness(double stiffness){ std::cout << "setStiffness() Not implemented" << std::endl; }
    virtual double getStiffness(){return 0.0;}
    virtual void setDamping(double damping){ std::cout << "setDamping() Not implemented" << std::endl; }
    virtual double getDamping(){return 0.0;}
    virtual void setWeight(double weight){task->setWeight(weight);}
    virtual void setWeight(Eigen::VectorXd& weight){task->setWeight(weight);}
    virtual Eigen::VectorXd getWeight(){}
    virtual void setDesiredState(){ std::cout << "setDesiredState() Not implemented" << std::endl; }
    virtual void setWeights(Eigen::Vector3d weight){};
    // virtual void activate() = 0;
    // virtual void deactivate() = 0;


protected: /* Protected virtual methods */
    virtual const double* getCurrentState();


protected: /* Protected methods */
    void updateDesiredStateVector(const double* ptrToFirstIndex);
    void updateCurrentStateVector(const double* ptrToFirstIndex);
    void setStateDimension(int taskDimension);
    // For parsing and compiling yarp messages.
    virtual void parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply);
    std::string printValidMessageTags();
    bool openControlPorts();
    bool closeControlPorts();
    bool parseControlInput(yarp::os::Bottle& input);


protected:
    std::shared_ptr<ocra::Task>                 task;
    std::vector< std::shared_ptr<ocra::Task> >  taskVector;

    ocra::Controller&               ctrl;
    const ocra::Model&              model;
    const std::string&              name;
    std::string                     stableName; //hack to avoid using name in compileOutgoingMessage()

    bool taskManagerActive;

    //Generic double vector to store states:
    std::vector<double> currentStateVector, desiredStateVector, newDesiredStateVector;
    Eigen::VectorXd eigenCurrentStateVector, eigenDesiredStateVector;
    int stateDimension;


    yarp::os::Network yarp;
    yarp::os::RpcServer rpcPort;
    std::string portName;



    bool controlPortsOpen;
    yarp::os::Bottle stateInBottle, stateOutBottle;
    std::string inputControlPortName, outputControlPortName;
    yarp::os::Port inputControlPort, outputControlPort;




private:
    std::shared_ptr<RpcMessageCallback> rpcCallback;
    std::shared_ptr<ControlInputCallback> controlCallback;
    std::shared_ptr<StateUpdateThread> stateThread;

    TASK_MODE taskMode;
    yarp::os::Log yLog;
};

}

#endif // TASKMANAGERBASE_H
