#ifndef CLIENT_COMMUNICATIONS_H
#define CLIENT_COMMUNICATIONS_H

#include <iostream>
#include <memory>
#include <map>

#include <Eigen/Dense>
#include <Eigen/Lgsm>


// TODO: Should put in defines for yarp independent builds
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Time.h>

#include <ocra/control/TaskYarpInterfaceVocab.h>
#include <ocra-recipes/MessageVocabulary.h>
#include <ocra/util/Macros.h>


namespace ocra_recipes
{
class ClientCommunications : public yarp::os::PortReader
{
    DEFINE_CLASS_POINTER_TYPEDEFS(ClientCommunications)
using TaskPortMap = std::map<std::string, std::shared_ptr<yarp::os::RpcClient> >;

public:
    ClientCommunications();
    virtual ~ClientCommunications();

    /**
     *
     *  Opens an RPC port for the client with the format /ControllerClient/<client-number>/rpc:o and an input one with the format /ControllerClient/<client-number>/:i
     *  @param[in]   connectToTasks True to automatically connect to task RPC ports. True by default.
     *  @return True when the conection to tasks is successful.
     *
     */
     bool open(double timeout = 20.0, bool connectToTasks = true);

    /**
     *  Closes the RPC and input ports opened by this client.
     *
     *  @return True after all ports are closed.
     */
    bool close();

    /**
     *  Closes the task-specific RPC ports and deletes this task from the list of current clients.
     *
     *  @param taskName Name of the task.
     */
    void close(const std::string& taskName);

    /**
     *  Reads and parses (parseMessage()) a message sent by the server.
     *
     *  @param connection Source of the message.
     *
     *  @return True if the message is parsed succesfully, false otherwise.
     */
    virtual bool read(yarp::os::ConnectionReader& connection);

    /**
     *  The method that does the real parsing. The expected messages from the server are: REMOVE_TASK_PORT or HELP.
     *
     *  @param input A bottle containing the message sent by the server.
     */
    void parseMessage(yarp::os::Bottle& input);

    /**
     *  Queries the server for the task types as strings.
     *
     *  @return A vector of strings containint the task types sent by the server.
     */
    std::vector<std::string> getTaskTypes();

    /**
     *  Takes a bottled query from the client and sends it to the server via RPC.
     *
     *  @param requestBottle Bottled query to be sent to the server.
     *
     *  @return A bottled reply from the server.
     */
    yarp::os::Bottle queryController(yarp::os::Bottle& requestBottle);

    /**
     *  Sends a request to the server among one of the predefined messages in ocra_recipes::SERVER_COMMUNICATIONS_MESSAGE.
     *
     *  @param request These could be General indicators, controller requests, controller status indicators or task requests. See ocra_recipes::SERVER_COMMUNICATIONS_MESSAGE for more options.
     *
     *  @return The reply of the server.
     */
    yarp::os::Bottle queryController(const SERVER_COMMUNICATIONS_MESSAGE request);

    /**
     *  Allows to send a series of messages from ocra_recipes::SERVER_COMMUNICATIONS_MESSAGE.
     *
     *  @param requestVector Vector of communication messages to be sent to the server.
     *
     *  @return Bottled server reply.
     */
    yarp::os::Bottle queryController(const std::vector<SERVER_COMMUNICATIONS_MESSAGE> requestVector);

    // void queryController(const SERVER_COMMUNICATIONS_MESSAGE request, yarp::os::Bottle& reply);

    // void queryTask(const std::string& taskName, const SERVER_COMMUNICATIONS_MESSAGE request, yarp::os::Bottle& reply);
    // void queryTask(const int taskIndex, const SERVER_COMMUNICATIONS_MESSAGE request, yarp::os::Bottle& reply);
    // void queryTasks(const SERVER_COMMUNICATIONS_MESSAGE request, std::vector<yarp::os::Bottle&>& replies);
    // void queryTasks(const std::vector<SERVER_COMMUNICATIONS_MESSAGE>& requests, std::vector<yarp::os::Bottle&>& replies);
    // void queryTasks(const std::vector<SERVER_COMMUNICATIONS_MESSAGE>& requests, std::vector<yarp::os::Bottle&>& replies);

    // TODO: Finish the documentation for the following methods
    std::vector<std::string> getTaskPortNames();
    std::string getTaskPortName(const std::string& taskName);

    std::vector<std::string> getTaskNames();
    std::shared_ptr<yarp::os::RpcClient> getTaskClient(const std::string& taskName);


    bool parseInput(yarp::os::Bottle& input);


    /*! \class ControlInputCallback
     *  \brief a short description
     *
     *  a long description
     */
    class InputCallback : public yarp::os::PortReader {
    private:
        ClientCommunications& coms;

    public:
        InputCallback(ClientCommunications& comsRef);

        virtual bool read(yarp::os::ConnectionReader& connection);
    };


private:
    bool openServerConnections(double timeout=20.0);
    bool openTaskConnections();

private:
    yarp::os::RpcClient rpcClientPort;
    yarp::os::Port         inputPort;

    std::string     rpcClientPort_Name;
    std::string        inputPort_Name;

    int clientNumber;
    static int CONTROLLER_CLIENT_COUNT;

    yarp::os::Network yarp;
    yarp::os::Log yLog;

    TaskPortMap taskRpcClients;

    InputCallback inputCallback;


};
} // namespace ocra_recipes
#endif // CLIENT_COMMUNICATIONS_H
