#ifndef TASK_YARP_INTERFACE_VOCAB_H
#define TASK_YARP_INTERFACE_VOCAB_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <yarp/os/Bottle.h>

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
    GET_DESIRED,
    GET_ACTIVITY_STATUS,
    GET_DIMENSION,
    GET_STATE_DIMENSION, // TODO: Remove
    GET_TYPE,
    GET_NAME,
    GET_CONTROL_PORT_NAMES,
    GET_TASK_PORT_NAME,
    GET_TASK_ERROR,
    // Setters
    SET_STIFFNESS,
    SET_STIFFNESS_VECTOR,
    SET_STIFFNESS_MATRIX,
    SET_DAMPING,
    SET_DAMPING_VECTOR,
    SET_DAMPING_MATRIX,
    SET_WEIGHT,
    SET_WEIGHT_VECTOR,
    SET_DESIRED,
    // Other
    ACTIVATE,
    DEACTIVATE,
    TASK_IS_ACTIVATED,
    TASK_IS_DEACTIVATED,
    OPEN_CONTROL_PORTS,
    CLOSE_CONTROL_PORTS,
    HELP

};

inline TASK_MESSAGE stringToTaskManagerMessageTag(const std::string& testString)
{
    return OCRA_FAILURE;
}



} // namespace ocra
#endif // TASK_YARP_INTERFACE_VOCAB_H
