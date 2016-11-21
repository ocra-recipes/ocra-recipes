#ifndef TASK_YARP_INTERFACE_VOCAB_H
#define TASK_YARP_INTERFACE_VOCAB_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <yarp/os/Bottle.h>

#include <algorithm>
#include <string>
#include <regex>
#include <iterator>

namespace ocra
{

enum TASK_MESSAGE
{
    // Indicators
    OCRA_FAILURE = 0,
    OCRA_SUCCESS,
    OCRA_WARNING,
    // Getters
    GET_CURRENT_STATE,
    GET_STIFFNESS,
    GET_DAMPING,
    GET_WEIGHTS,
    GET_ACTIVITY_STATUS,
    GET_DIMENSION,
    GET_TYPE,
    GET_TYPE_AS_STRING,
    GET_NAME,
    GET_CONTROL_PORT_NAMES,
    GET_TASK_PORT_NAME,
    GET_TASK_ERROR,
    GET_TASK_STATE,
    GET_DESIRED_TASK_STATE,
    GET_TASK_POSITION,
    GET_DESIRED_TASK_POSITION,
    // Setters
    SET_STIFFNESS,
    SET_STIFFNESS_VECTOR,
    SET_STIFFNESS_MATRIX,
    SET_DAMPING,
    SET_DAMPING_VECTOR,
    SET_DAMPING_MATRIX,
    SET_WEIGHT,
    SET_WEIGHT_VECTOR,
    SET_DESIRED_TASK_STATE,
    SET_DESIRED_TASK_POSITION,
    // Other
    ACTIVATE,
    DEACTIVATE,
    TASK_IS_ACTIVATED,
    TASK_IS_DEACTIVATED,
    OPEN_CONTROL_PORTS,
    CLOSE_CONTROL_PORTS,
    HELP

};

class TaskMessageHandler
{
public:
    static const std::vector<std::string> TASK_MESSAGES_AS_STRINGS;

    static std::string taskManagerMessageTagToString(TASK_MESSAGE msg);
    static std::string removeUnderscores(std::string s);
    static TASK_MESSAGE stringToTaskManagerMessageTag(std::string testString);
};



} // namespace ocra
#endif // TASK_YARP_INTERFACE_VOCAB_H
