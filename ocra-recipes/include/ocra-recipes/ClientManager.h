/*! \file       ClientManager.h
 *  \brief      Module class for the controller Client.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-icub.
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

#ifndef CLIENT_MANAGER_H
#define CLIENT_MANAGER_H

#include <iostream>
#include <memory>

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>

#include "ocra-recipes/ControllerClient.h"


namespace ocra_recipes
{
/*! \class ClientManager
 *  \brief The controller module which launches the controller thread.
 *
 *  Basically all this does is parse the command line arguments and look for the various config and task set files. It then instantiates a WBI instance (yarpWBI specifically) and a \ref IcubControllerClient instance. It launches these threads and then basically just waits till it gets a kill (ctrl+c) command to close them down. Does a little keeping track of time as well.
 */
class ClientManager : public yarp::os::RFModule
{
public:
    /*! Constructor which essentially does nothing.
     */
    ClientManager(std::shared_ptr<ControllerClient> customClient);

    /*! Destructor which essentially does nothing.
     */
    ~ClientManager();

    /*! Configures the module by parsing the RF contents. Passes to client when done.
     *  \param rf A resource finder instance which is initialized from the command line args.
     *
     *  \return True or false if the configuration was successful.
     */
    bool configure(yarp::os::ResourceFinder &rf);

    /*! Simply calls client->start() then runModule()
     */
    int launchClient();

    /*! Interrupts the module execution and stops the control and wbi threads.
     */
    bool interruptModule();

    /*! Closes the module. First shuts down the threads.
     */
    bool close();

    /*! Updates the ClientManager. Basically just clocks the thread run() method.
     *  \return Whether or not the clocking functions worked.
     */
    bool updateModule();

    /*! Prints all the command line args one could use.
     */
    void printHelp();

    /*! Gets the name of the module. Override this function to set a different name.
     *  \return A string with the module name.
     */
    virtual std::string getManagerName();


public:
    /*! \class moduleCallback
     *  \brief A callback function which binds the rpc server port opened in the contoller server module to the controller thread's parsing function.
     */
    class moduleCallback : public yarp::os::PortReader
    {

    public:

        /*! Constructor
         *  \param newModuleRef A shared pointer to the control thread.
         */
        moduleCallback(ClientManager& newModuleRef);

        /*! read
         *  \param connection Reads a port connection.
         *
         *  \return A boolean which tells whether or not a message was read.
         */
        virtual bool read(yarp::os::ConnectionReader& connection);

    private:
        ClientManager& moduleRef; /*!< A ref to the client module. */
    };


private:
    void callbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply);

protected:
    virtual void customCallbackParser(yarp::os::Bottle& message, yarp::os::Bottle& reply);
    virtual bool customUpdateModule();


private:
    std::shared_ptr<ControllerClient> client;               /*!< The controller client thread pointer. */
    std::shared_ptr<moduleCallback> rpcCallback;                 /*!< Rpc server port callback function. */
    yarp::os::RpcServer rpcPort;                            /*!< Rpc server port. */
    yarp::os::Log yLog;                                     /*!< For logging in yarp. */

    int expectedClientPeriod;                         /*!< The expected period of the client thread. In ms.*/
    double avgTime;                                         /*!< Average time between successive calls of the `run()` method.*/
    double stdDev;                                          /*!< Standard deviation of the average time between successive calls of the `run()` method. */
    double avgTimeUsed;                                     /*!< Average time for the `run()` method to execute. Should be close to avgTime. */
    double stdDevUsed;                                      /*!< Standard deviation of the average time for the `run()` method to execute. */

    std::string rpcPortName;                                /*!< The name of the rpc port for the thread. */

    static int CONTROLLER_CLIENT_MANAGER_COUNT;              /*!< A count that is incremented each time a client thread is constructed. */
    int moduleNumber;                                       /*!< The unique thread number. */


};

} /* ocra_recipes */

#endif
