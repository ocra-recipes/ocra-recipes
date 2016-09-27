/*! \file       ControlThread.h
 *  \brief      A class for launching generic control threads.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-recipes.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>

#include <Eigen/Dense>
#include <Eigen/Lgsm>

#include <ocra/util/Macros.h>



namespace ocra_recipes
{
class TaskParameters
{
DEFINE_CLASS_POINTER_TYPEDEFS(TaskParameters)

public:
    double kp;
    double kd;
    int dimension;
    Eigen::VectorXd weight;
    Eigen::VectorXd desired;
    Eigen::VectorXd currentState;
    std::string type;
    std::string name;
    bool isActive;

    friend std::ostream& operator<<(std::ostream &out, const TaskParameters& params)
        {
            out << "kp = " << params.kp << std::endl;
            out << "kd = " << params.kd << std::endl;
            out << "dimension = " << params.dimension << std::endl;
            out << "weight = " << params.weight.transpose() << std::endl;
            out << "desired = " << params.desired.transpose() << std::endl;
            out << "currentState = " << params.currentState.transpose() << std::endl;
            out << "type = " << params.type << std::endl;
            out << "name = " << params.name << std::endl;
            out << "isActive = " << params.isActive << std::endl;
            return out;
        }
};



class ControlThread: public yarp::os::RateThread
{
DEFINE_CLASS_POINTER_TYPEDEFS(ControlThread)

public:
    // Constructor
    ControlThread(int period, const std::string& taskRpcPortName);
    ~ControlThread();

    int threadId;
    static int CONTROL_THREAD_COUNT;

    // RateThread virtual functions
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    // ControlThread pure virtual functions
    virtual bool ct_threadInit()=0;
    virtual void ct_threadRelease()=0;
    virtual void ct_run()=0;

    // ControlThread functions
    std::string getThreadType(){return controlThreadType;}
    bool deactivateTask();
    bool activateTask();

    std::string getOutputPortName(){return outputPortName;}
    std::string getInputPortName(){return inputPortName;}


    /************** controlInputCallback *************/
    class inputCallback : public yarp::os::PortReader {
        private:
            ControlThread& ctBase;

        public:
            inputCallback(ControlThread& ctBaseRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
    };
    /************** controlInputCallback *************/

protected:

    void setThreadType(const std::string& _threadType = "ControlThread"){controlThreadType = _threadType;}

    std::string controlThreadType;

    // Yarp control ports
    std::string inputPortName, outputPortName;
    yarp::os::Port inputPort, outputPort;

    //Yarp RPC client
    std::string taskRpcServerName, threadRpcClientName;
    yarp::os::RpcClient threadRpcClient;

    //Yarp network
    yarp::os::Network yarp;

    bool openControlPorts();
    bool connectControlPorts();

    bool isFirstInputBottle;
    inputCallback* inpCallback;
    Eigen::VectorXd currentStateVector;
    bool parseInput(yarp::os::Bottle* input);

    Eigen::VectorXd getCurrentState();
    bool waitingForFirstStateMessage;
    void sendGetStateMessage();

    double controlThreadPeriod;

    TaskParameters originalTaskParams;
    TaskParameters currentTaskParams;

    int weightDimension;
    int stateDimension;

    bool getTaskDimensions();
    bool getTaskParameters(TaskParameters& TP);


    double closePortTimeout;
};
} // namespace ocra_recipes
#endif //CONTROL_THREAD_H
