#include <ocra/control/TaskManagers/TaskManagerMessageVocab.h>

using namespace ocra;


TASK_MESSAGE stringToTaskManagerMessageTag(const std::string& testString)
{
    return OCRA_FAILURE;
}

std::vector<double> pourBottleIntoStdVector(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    std::vector<double> returnVector(nVals);
    for (auto i = 0; i < nVals; ++i) {
        returnVector[i] = bottle.tail().get(i).asDouble();
    }
    indexesToSkip = nVals + 1;
    return returnVector;
}

Eigen::VectorXd pourBottleIntoEigenVector(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    Eigen::VectorXd returnVector(nVals);
    for (auto i = 0; i < nVals; ++i) {
        returnVector(i) = bottle.tail().get(i).asDouble();
    }
    indexesToSkip = nVals + 1;
    return returnVector;
}

Eigen::MatrixXd pourBottleIntoEigenMatrix(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nRows = bottle.get(0).asInt();
    int nCols = bottle.get(1).asInt();
    Eigen::MatrixXd returnMatrix(nRows,nCols);

    int startIndex = 2;
    for (auto i = 0; i < nRows; ++i) {
        for (auto j = 0; j < nCols; ++j) {
        returnMatrix(i,j) = bottle.get(startIndex).asDouble();
        ++startIndex;
        }
    }
    indexesToSkip = nRows + nCols + 1;
    return returnMatrix;
}

void pourStdVectorIntoBottle(const std::vector<double>& vec, yarp::os::Bottle& bottle)
{
    if(vec.size()>0) {
        bottle.addInt(OCRA_SUCCESS);
        bottle.addInt(vec.size());
        for (auto val : vec) {
            bottle.addDouble(val);
        }
    } else {
        bottle.addInt(OCRA_FAILURE);
    }
}

void pourEigenVectorIntoBottle(const Eigen::VectorXd& vec, yarp::os::Bottle& bottle)
{
    if(vec.size()>0) {
        bottle.addInt(OCRA_SUCCESS);
        bottle.addInt(vec.size());
        for (auto i=0; i<vec.size(); ++i) {
            bottle.addDouble(vec(i));
        }
    } else {
        bottle.addInt(OCRA_FAILURE);
    }
}

void pourEigenMatrixIntoBottle(const Eigen::MatrixXd& mat, yarp::os::Bottle& bottle)
{
    if(mat.size()>0) {
        bottle.addInt(OCRA_SUCCESS);
        bottle.addInt(mat.rows());
        bottle.addInt(mat.cols());
        for (auto i=0; i<mat.rows(); ++i) {
            for (auto j=0; j<mat.cols(); ++j) {
                bottle.addDouble(mat(i,j));
            }
        }
    } else {
        bottle.addInt(OCRA_FAILURE);
    }
}

yarp::os::Bottle trimBottle(const yarp::os::Bottle& bottle, int startIndex, int endIndex)
{
    int btlSize = bottle.size();
    int lastIndex = btlSize - 1;
    int trimFrom, trimTo;


    if (endIndex==-1) {
        trimTo = btlSize;
    } else {
        trimTo = (endIndex <= btlSize) ? endIndex : btlSize;
    }

    trimFrom = (startIndex >= 0) ? startIndex : 0;
    trimFrom = ( startIndex < (trimTo-2) ) ? startIndex : (trimTo-2);

    yarp::os::Bottle tmpBtl;
    while(trimFrom<trimTo)
    {
        tmpBtl.add(bottle.get(trimFrom));
        ++trimFrom;
    }
    return tmpBtl;
}
