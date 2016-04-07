#ifndef TASK_MANAGER_MESSAGE_VOCAB_H
#define TASK_MANAGER_MESSAGE_VOCAB_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
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
    GET_STATE_DIMENSION,
    GET_TYPE,
    GET_NAME,
    GET_CONTROL_PORT_NAMES,
    GET_TASK_PORT_NAME,
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

TASK_MESSAGE stringToTaskManagerMessageTag(const std::string& testString);

std::vector<double> pourBottleIntoStdVector(yarp::os::Bottle bottle, int& indexesToSkip);
Eigen::VectorXd pourBottleIntoEigenVector(yarp::os::Bottle bottle, int& indexesToSkip);
Eigen::MatrixXd pourBottleIntoEigenMatrix(yarp::os::Bottle bottle, int& indexesToSkip);

void pourStdVectorIntoBottle(const std::vector<double>& vec, yarp::os::Bottle& bottle);
void pourEigenVectorIntoBottle(const Eigen::VectorXd& vec, yarp::os::Bottle& bottle);
void pourEigenMatrixIntoBottle(const Eigen::MatrixXd& mat, yarp::os::Bottle& bottle);

yarp::os::Bottle trimBottle(const yarp::os::Bottle& bottle, int startIndex, int endIndex=-1);



} // namespace ocra
#endif // TASK_MANAGER_MESSAGE_VOCAB_H
