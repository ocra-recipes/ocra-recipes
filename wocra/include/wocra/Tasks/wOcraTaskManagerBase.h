#ifndef wOcraTASKMANAGERBASE_H
#define wOcraTASKMANAGERBASE_H

#include "wocra/Models/wOcraModel.h"
#include "wocra/wOcraController.h"
#include "wocra/Tasks/wOcraTask.h"
#include "wocra/Trajectory/wOcraTrajectory.h"

#include <Eigen/Dense>

#include <yarp/os/Network.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>

namespace wocra
{


/** \brief Task Manager for the Center of Mass (CoM) task with the wOcra Controller
 *
 */
class wOcraTaskManagerBase
{
    public:
        wOcraTaskManagerBase(wocra::wOcraController& ctrl, const wocra::wOcraModel& model, const std::string& name, bool usesYarpPorts=false);
        virtual ~wOcraTaskManagerBase();


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
        /** \brief rpcMessageCallback
        *
        */
        class rpcMessageCallback : public yarp::os::PortReader {
        private:
            wOcraTaskManagerBase& tmBase;

        public:
            rpcMessageCallback(wOcraTaskManagerBase& tmBaseRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
        };

        /** \brief controlInputCallback
        *
        */
        class controlInputCallback : public yarp::os::PortReader {
        private:
            wOcraTaskManagerBase& tmBase;

        public:
            controlInputCallback(wOcraTaskManagerBase& tmBaseRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
        };

        /** \brief stateUpdateThread
        *
        */
        class stateUpdateThread : public yarp::os::RateThread
        {
        private:
            wOcraTaskManagerBase& tmBase;

        public:
            stateUpdateThread(int period, wOcraTaskManagerBase& tmBaseRef);
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
        wocra::wOcraTask*              task;


        wocra::wOcraController&        ctrl;
        const wocra::wOcraModel&       model;
        const std::string&              name;
        std::string                     stableName; //hack to avoid using name in compileOutgoingMessage()

        bool taskManagerActive;

        bool usesTrajectory;
        bool followingTrajectory;

        wocra::wOcraTrajectory*     taskTrajectory;

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
        virtual void parseIncomingMessage(yarp::os::Bottle *input, yarp::os::Bottle *reply);

        std::string printValidMessageTags();


        bool usesYARP;
        yarp::os::Network yarp;
        yarp::os::RpcServer rpcPort;
        std::string portName;
        rpcMessageCallback* processor;

        int stateDimension;

        void setTrajectoryType(std::string trajType="MinJerk");
        bool openControlPorts();
        bool closeControlPorts();


        bool controlPortsOpen;
        yarp::os::Bottle stateInBottle, stateOutBottle;
        std::string inputControlPortName, outputControlPortName;
        yarp::os::Port inputControlPort, outputControlPort;

        controlInputCallback* controlCallback;
        // controlOutputCallback* stateCallback;

        bool parseControlInput(yarp::os::Bottle *input);
        stateUpdateThread* stateThread;


};

}

#endif // wOcraTASKMANAGERBASE_H
