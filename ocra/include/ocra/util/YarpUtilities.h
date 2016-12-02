#ifndef YARP_UTILITIES_H
#define YARP_UTILITIES_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <yarp/os/Bottle.h>

namespace ocra
{

namespace util
{

inline std::vector<double> pourBottleIntoStdVector(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    std::vector<double> returnVector(nVals);
    for (auto i = 0; i < nVals; ++i) {
        returnVector[i] = bottle.tail().get(i).asDouble();
    }
    indexesToSkip = nVals + 1;
    return returnVector;
}

inline Eigen::VectorXd pourBottleIntoEigenVector(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    Eigen::VectorXd returnVector(nVals);
    for (auto i = 0; i < nVals; ++i) {
        returnVector(i) = bottle.tail().get(i).asDouble();
    }
    indexesToSkip = nVals + 1;
    return returnVector;
}

inline Eigen::VectorXi pourBottleIntoEigenVectorXi(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    Eigen::VectorXi returnVector(nVals);
    for (auto i = 0; i < nVals; ++i) {
        returnVector(i) = bottle.tail().get(i).asInt();
    }
    indexesToSkip = nVals + 1;
    return returnVector;
}

inline Eigen::Displacementd pourBottleIntoDisplacementd(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    double x = bottle.get(1).asDouble();
    double y = bottle.get(2).asDouble();
    double z = bottle.get(3).asDouble();
    double qw = bottle.get(4).asDouble();
    double qx = bottle.get(5).asDouble();
    double qy = bottle.get(6).asDouble();
    double qz = bottle.get(7).asDouble();
    Eigen::Displacementd disp(x,y,z,qw,qx,qy,qz);
    indexesToSkip = nVals + 1;
    return disp;
}

inline Eigen::Twistd pourBottleIntoTwistd(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    double rx = bottle.get(1).asDouble();
    double ry = bottle.get(2).asDouble();
    double rz = bottle.get(3).asDouble();
    double vx = bottle.get(4).asDouble();
    double vy = bottle.get(5).asDouble();
    double vz = bottle.get(6).asDouble();
    Eigen::Twistd twist(rx,ry,rz,vx,vy,vz);
    indexesToSkip = nVals + 1;
    return twist;
}

inline Eigen::Wrenchd pourBottleIntoWrenchd(yarp::os::Bottle bottle, int& indexesToSkip)
{
    int nVals = bottle.get(0).asInt();
    double tx = bottle.get(1).asDouble();
    double ty = bottle.get(2).asDouble();
    double tz = bottle.get(3).asDouble();
    double fx = bottle.get(4).asDouble();
    double fy = bottle.get(5).asDouble();
    double fz = bottle.get(6).asDouble();
    Eigen::Wrenchd wrench(tx,ty,tz,fx,fy,fz);
    indexesToSkip = nVals + 1;
    return wrench;
}

inline Eigen::MatrixXd pourBottleIntoEigenMatrix(yarp::os::Bottle bottle, int& indexesToSkip)
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

inline void pourStdVectorIntoBottle(const std::vector<double>& vec, yarp::os::Bottle& bottle)
{
    if(vec.size()>0) {
        bottle.addInt(vec.size());
        for (auto val : vec) {
            bottle.addDouble(val);
        }
    } else {
        bottle.addInt(0);
    }
}

inline void pourEigenVectorIntoBottle(const Eigen::VectorXd& vec, yarp::os::Bottle& bottle)
{
    if(vec.size()>0) {
        bottle.addInt(vec.size());
        for (auto i=0; i<vec.size(); ++i) {
            bottle.addDouble(vec(i));
        }
    } else {
        bottle.addInt(0);
    }
}

inline void pourEigenVectorXiIntoBottle(const Eigen::VectorXi& vec, yarp::os::Bottle& bottle)
{
    if(vec.size()>0) {
        bottle.addInt(vec.size());
        for (auto i=0; i<vec.size(); ++i) {
            bottle.addInt(vec(i));
        }
    } else {
        bottle.addInt(0);
    }
}

inline void pourEigenMatrixIntoBottle(const Eigen::MatrixXd& mat, yarp::os::Bottle& bottle)
{
    if(mat.size()>0) {
        bottle.addInt(mat.rows());
        bottle.addInt(mat.cols());
        for (auto i=0; i<mat.rows(); ++i) {
            for (auto j=0; j<mat.cols(); ++j) {
                bottle.addDouble(mat(i,j));
            }
        }
    } else {
        bottle.addInt(0);
    }
}

inline void pourDisplacementdIntoBottle(const Eigen::Displacementd& disp, yarp::os::Bottle& bottle)
{
    const int DISPLACEMENT_VECTOR_SIZE = 7;
    bottle.addInt(DISPLACEMENT_VECTOR_SIZE);

    bottle.addDouble(disp.x());
    bottle.addDouble(disp.y());
    bottle.addDouble(disp.z());
    bottle.addDouble(disp.qw());
    bottle.addDouble(disp.qx());
    bottle.addDouble(disp.qy());
    bottle.addDouble(disp.qz());

}

inline void pourTwistdIntoBottle(const Eigen::Twistd& twist, yarp::os::Bottle& bottle)
{
    const int TWIST_VECTOR_SIZE = 6;
    bottle.addInt(TWIST_VECTOR_SIZE);

    double rx = twist(0);
    double ry = twist(1);
    double rz = twist(2);
    double vx = twist(3);
    double vy = twist(4);
    double vz = twist(5);
    
     bottle.addDouble(rx);
     bottle.addDouble(ry);
     bottle.addDouble(rz);
     bottle.addDouble(vx);
     bottle.addDouble(vy);
     bottle.addDouble(vz);
}

inline void pourWrenchdIntoBottle(const Eigen::Wrenchd& wrench, yarp::os::Bottle& bottle)
{
    const int WRENCH_VECTOR_SIZE = 6;
    bottle.addInt(WRENCH_VECTOR_SIZE);

    bottle.addDouble(wrench.tx());
    bottle.addDouble(wrench.ty());
    bottle.addDouble(wrench.tz());
    bottle.addDouble(wrench.fx());
    bottle.addDouble(wrench.fy());
    bottle.addDouble(wrench.fz());
}

inline yarp::os::Bottle trimBottle(const yarp::os::Bottle& bottle, int startIndex, int endIndex=-1)
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


} // namespace utils
} // namespace ocra
#endif // YARP_UTILITIES_H
