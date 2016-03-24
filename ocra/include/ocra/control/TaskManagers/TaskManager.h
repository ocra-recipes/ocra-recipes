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

namespace ocra
{


/** \brief Task Manager for the Center of Mass (CoM) task with the  Controller
 *
 */
class TaskManager
{
    public:
        TaskManager(ocra::Controller& ctrl, const ocra::Model& model, const std::string& name, bool usesYarpPorts=false);
        virtual ~TaskManager();


        virtual void activate() = 0;
        virtual void deactivate() = 0;

        std::string getPortName();

        void updateTrajectory(double time);
        bool isFollowingTrajectory(){return followingTrajectory;}

        // For getting the task type
        virtual std::string getTaskManagerType();

        virtual VectorXd getTaskError();
        double getTaskErrorNorm();

        /*****************************************************************************************/
        /** \brief RpcMessageCallback
        *
        */
        class RpcMessageCallback : public yarp::os::PortReader {
        private:
            TaskManager& tmBase;

        public:
            RpcMessageCallback(TaskManager& tmBaseRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
        };

        /** \brief ControlInputCallback
        *
        */
        class ControlInputCallback : public yarp::os::PortReader {
        private:
            TaskManager& tmBase;

        public:
            ControlInputCallback(TaskManager& tmBaseRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
        };

        /** \brief StateUpdateThread
        *
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
        /*****************************************************************************************/


        virtual void setStiffness(double stiffness){ std::cout << "setStiffness() Not implemented" << std::endl; }
        virtual double getStiffness(){return 0.0;}
        virtual void setDamping(double damping){ std::cout << "setDamping() Not implemented" << std::endl; }
        virtual double getDamping(){return 0.0;}
        virtual void setWeight(double weight){task->setWeight(weight);}
        virtual void setWeight(Eigen::VectorXd& weight){task->setWeight(weight);}
        virtual Eigen::VectorXd getWeight(){}
        virtual void setDesiredState(){ std::cout << "setDesiredState() Not implemented" << std::endl; }

        virtual void setWeights(Eigen::Vector3d weight){};

    protected:
        std::shared_ptr<ocra::Task>                 task;
        std::vector< std::shared_ptr<ocra::Task> >  taskVector;
        // ocra::Task*                     task;


        ocra::Controller&               ctrl;
        const ocra::Model&              model;
        const std::string&              name;
        std::string                     stableName; //hack to avoid using name in compileOutgoingMessage()

        bool taskManagerActive;

        bool usesTrajectory;
        bool followingTrajectory;

        std::unique_ptr<Trajectory>     taskTrajectory;

        //Generic double vector to store states:

        std::vector<double> currentStateVector, desiredStateVector, newDesiredStateVector;

        Eigen::VectorXd eigenCurrentStateVector, eigenDesiredStateVector;

        int waypointSelector;




        virtual const double* getCurrentState();
        virtual bool checkIfActivated();

        void updateDesiredStateVector(const double* ptrToFirstIndex);
        void updateCurrentStateVector(const double* ptrToFirstIndex);

        void setStateDimension(int taskDimension, int waypointDimension=0);


        // For parsing and compiling yarp messages.
        virtual void parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply);

        std::string printValidMessageTags();


        yarp::os::Network yarp;
        yarp::os::RpcServer rpcPort;
        std::string portName;

        int stateDimension;

        void setTrajectoryType(std::string trajType="MinJerk");
        bool openControlPorts();
        bool closeControlPorts();


        bool controlPortsOpen;
        yarp::os::Bottle stateInBottle, stateOutBottle;
        std::string inputControlPortName, outputControlPortName;
        yarp::os::Port inputControlPort, outputControlPort;

        std::unique_ptr<RpcMessageCallback> rpcCallback;
        std::shared_ptr<ControlInputCallback> controlCallback;
        std::shared_ptr<StateUpdateThread> stateThread;

        bool parseControlInput(yarp::os::Bottle& input);


};

}

#endif // TASKMANAGERBASE_H
