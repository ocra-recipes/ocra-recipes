
#include "ocra/control/Trajectory/ExperimentalTrajectory.h"
#include <math.h>

#ifndef ALPHA_L
#define ALPHA_L 1.0
#endif

namespace ocra
{

void ExperimentalTrajectory::calculateVarianceParameters()
{
    numberOfKernels = nWaypoints;

    maxCovariance = ( waypoints - waypoints.rowwise().mean().replicate(1, waypoints.cols()) ).array().square().rowwise().sum() / (waypoints.cols() - 1) ;


    kernelCenters = Eigen::VectorXd::Zero(numberOfKernels);
    for(int i=0; i<numberOfKernels-1; i++){
        kernelCenters(i+1) = kernelCenters(i) + pointToPointDurationVector(i);
    }

    designMatrix = kernelFunction(kernelCenters);
    designMatrixInv = designMatrix.inverse();



    calculateRbfnWeights();
    varianceStartTrigger = true;

    // std::cout << "rbfn weights: " << rbfnWeights << std::endl;
    // std::cout << "maxCovariance\n" << maxCovariance << std::endl;
    // std::cout << "kernelLengthParameter\n" << kernelLengthParameter << std::endl;
    // std::cout << "kernelCenters\n" << kernelCenters << std::endl;

}

Eigen::MatrixXd ExperimentalTrajectory::getWaypointData()
{
    Eigen::MatrixXd dataMat = Eigen::MatrixXd::Zero(nWaypoints, nDoF+1);

    dataMat.col(0) = kernelCenters;
    dataMat.rightCols(nDoF) = waypoints.transpose();

    return dataMat;
}


Eigen::MatrixXd ExperimentalTrajectory::kernelFunction(double m)
{
    int nC = kernelCenters.rows();
    int nS = maxCovariance.rows();

    Eigen::VectorXd kern = ((-(m - kernelCenters.array()).array().square()) / kernelLengthParameter).array().exp();

    Eigen::MatrixXd result = (maxCovariance.transpose().replicate(nC, 1)).array() * kern.replicate(1, nS).array();

    Eigen::MatrixXd blockDiagonalResult = Eigen::MatrixXd::Zero(nS, (nS*nC));

    for(int i=0; i<result.rows(); i++)
    {
        blockDiagonalResult.block(0, i*nS, nS, nS) = result.row(i).asDiagonal();
    }

    return blockDiagonalResult;

}

Eigen::MatrixXd ExperimentalTrajectory::kernelFunction(Eigen::VectorXd& evalVec)
{
    int sizeVec = evalVec.rows();
    int nC = kernelCenters.rows();
    int nS = maxCovariance.rows();

    Eigen::MatrixXd blockDiagonalResult = Eigen::MatrixXd::Zero((nS*sizeVec), (nS*nC));

    for(int i=0; i<sizeVec; i++)
    {
        blockDiagonalResult.middleRows(i*nS, nS) = kernelFunction(evalVec(i));
    }

    return blockDiagonalResult;

}


Eigen::VectorXd ExperimentalTrajectory::rbfnKernelFunction(double m)
{
        return ((-(m - kernelCenters.array()).array().square()).array() / kernelLengthParameter).array().exp();
}

Eigen::MatrixXd ExperimentalTrajectory::rbfnKernelFunction(Eigen::VectorXd& evalVec)
{
    int sizeVec = evalVec.rows();
    int nC = kernelCenters.rows();

    Eigen::MatrixXd centersMat = kernelCenters.transpose().replicate(sizeVec,1);
    Eigen::MatrixXd evalMat = evalVec.replicate(1,nC);

    return ((-(evalMat - centersMat).array().square()).array() / kernelLengthParameter).array().exp();
}


void ExperimentalTrajectory::calculateRbfnWeights()
{
    int nC = kernelCenters.rows();
    Eigen::MatrixXd rbfnDesignMatrix = rbfnKernelFunction(kernelCenters);
    // rbfnWeights = Eigen::MatrixXd::Zero(nDoF, nC);
    Eigen::MatrixXd wayTrans = waypoints.transpose();

    // std::cout << "phi = " << rbfnDesignMatrix.rows() << " x " << rbfnDesignMatrix.cols() << std::endl;
    // std::cout << "way = " << wayTrans.rows()<< " x " << wayTrans.cols() << std::endl;
    Eigen::MatrixXd A = rbfnDesignMatrix * rbfnDesignMatrix.transpose();
    // std::cout << "A = " << A.rows()<< " x " << A.cols() << std::endl;

    Eigen::MatrixXd b = rbfnDesignMatrix * wayTrans;

    // std::cout << "b = " << b.rows()<< " x " << b.cols() << std::endl;


    // rbfnWeights = (A.inverse() * b).transpose(); // the transpose makes weights = nDoF x nCenters which is better for the output function.

    // rbfnWeights = A.fullPivLu().solve(b).transpose();
    rbfnWeights = rbfnDesignMatrix.fullPivLu().solve(wayTrans).transpose();
    // std::cout << "rbfn weights:\n" << rbfnWeights << std::endl;

}

Eigen::VectorXd ExperimentalTrajectory::getRbfnOutput(double m)
{
    Eigen::VectorXd phi = rbfnKernelFunction(m);
    return rbfnWeights * phi;
}

Eigen::MatrixXd ExperimentalTrajectory::getRbfnOutput(Eigen::VectorXd& evalVec)
{
    Eigen::MatrixXd phi = rbfnKernelFunction(evalVec);
    return rbfnWeights * phi;
}



void ExperimentalTrajectory::precalculateTrajectory(double DT, Eigen::MatrixXd& _path, Eigen::VectorXd& _timeline)
{
    double currentTime = 0.0;

    int numberOfDataPoints = ceil((pointToPointDurationVector.sum() / DT)*2);

    Eigen::MatrixXd path(numberOfDataPoints, nDoF);
    Eigen::VectorXd timeline(numberOfDataPoints);


    int index = 0;
    startTrigger = true;
    currentWaypointIndex = 0;
    while (!trajectoryFinished)
    {
        path.row(index) = getDesiredValues(currentTime).col(POS_INDEX).transpose();
        timeline(index) = currentTime;
        currentTime += DT;
        index++;
    }
    //  reset values to their original state
    startTrigger = true;
    currentWaypointIndex = 0;
    trajectoryFinished = false;

    _path = path.topRows(index);
    _timeline = timeline.head(index);


}

Eigen::MatrixXd ExperimentalTrajectory::getDesiredValues(double _time)
{
    /**
    * For details on the analytical point to point min-jerk formulation see:
    * http://www.jneurosci.org/content/5/7/1688.full.pdf
    * http://shadmehrlab.org/book/minimum_jerk/minimumjerk.htm
    */
    Eigen::MatrixXd desiredValue = Eigen::MatrixXd::Zero(nDoF, TRAJ_DIM);

    if (startTrigger)
    {
        startTrigger = false;
        t0 = _time;
        desiredValue.col(POS_INDEX) = waypoints.col(0);
        desiredValue.col(VEL_INDEX) = Eigen::VectorXd::Zero(nDoF);
        desiredValue.col(ACC_INDEX) = Eigen::VectorXd::Zero(nDoF);

        posOld = desiredValue.col(POS_INDEX);
        velOld = desiredValue.col(VEL_INDEX);

        // posOld = waypoints.col(0);
        // velOld =Eigen::VectorXd::Zero(nDoF);
        t_old = t0;

        // return desiredValue;
    }

    else
    {
        double t = _time - t0;

        if (t<=kernelCenters.maxCoeff())
        {
            desiredValue.col(POS_INDEX) = getRbfnOutput(t);
        }
        else
        {
            desiredValue.col(POS_INDEX) = waypoints.rightCols(1);
        }

        double delta_t = t-t_old;
        desiredValue.col(VEL_INDEX) = (desiredValue.col(POS_INDEX) - posOld) / delta_t;
        desiredValue.col(ACC_INDEX) = (desiredValue.col(VEL_INDEX) - velOld) / delta_t;

        posOld = desiredValue.col(POS_INDEX);
        velOld = desiredValue.col(VEL_INDEX);
        t_old = t;
    }



    return desiredValue;
}

Eigen::VectorXd ExperimentalTrajectory::getVariance(double time)
{
    if (varianceStartTrigger) {
        t0_variance = time;
        varianceStartTrigger = false;
    }


    double t = time - t0_variance;


    Eigen::VectorXd variance;

    if (t<=pointToPointDurationVector.sum()) {
        Eigen::MatrixXd Ks = kernelFunction(t);
        variance = maxCovariance - (Ks * designMatrixInv * Ks.transpose()).diagonal();
    }
    else{
        variance = Eigen::VectorXd::Zero(maxCovariance.rows());
    }

    return variance;
}

void ExperimentalTrajectory::getDesiredValues(double time, Eigen::MatrixXd& desiredValues, Eigen::VectorXd& variance)
{
    desiredValues = getDesiredValues(time);
    variance = getVariance(time);

}

void ExperimentalTrajectory::getDesiredValues(double _time, Eigen::Displacementd& _position, Eigen::Twistd& _velocity, Eigen::Twistd& _acceleration)
{


}


void ExperimentalTrajectory::initializeTrajectory()
{
    originalWaypoints = waypoints;
    kernelLengthParameter = pointToPointDurationVector.mean();
    // Add a waypoint to the end.
    int extraWaypoints = 1;
    int nStartWp = extraWaypoints;
    int nEndWp = extraWaypoints;

    int nWpNew = nWaypoints + nStartWp + nEndWp;

    Eigen::MatrixXd waypointsTmp = Eigen::MatrixXd::Zero(nDoF, nWpNew);

    waypointsTmp.leftCols(nStartWp) = waypoints.leftCols(1).replicate(1,nStartWp);
    waypointsTmp.middleCols(nStartWp, nWaypoints) = waypoints;
    waypointsTmp.rightCols(nEndWp) = waypoints.rightCols(1).replicate(1,nEndWp);

    waypoints.resize(nDoF, nWaypoints);
    waypoints = waypointsTmp;
    // std::cout << pointToPointDurationVector << std::endl;


    Eigen::VectorXd durationVectorTmp = Eigen::VectorXd::Zero(nWpNew-1);

    double extraWpDt = 0.01 * kernelLengthParameter;
    // double extraWpDt = 0.01 * pointToPointDurationVector.sum();
    // std::cout << "extraWpDt: " << extraWpDt << std::endl;

    durationVectorTmp.head(nStartWp) = Eigen::VectorXd::Constant(nStartWp, extraWpDt);
    durationVectorTmp.segment(nStartWp, nWaypoints-1) = pointToPointDurationVector;
    durationVectorTmp.tail(nEndWp) = Eigen::VectorXd::Constant(nEndWp, extraWpDt);

    pointToPointDurationVector.resize(durationVectorTmp.size());
    pointToPointDurationVector = durationVectorTmp;

    nWaypoints = nWpNew;

    std::cout << pointToPointDurationVector << std::endl;
    calculateVarianceParameters();
}


double ExperimentalTrajectory::getMaxVariance()
{
    return maxCovariance.maxCoeff();
}
void ExperimentalTrajectory::reinitialize()
{
    nWaypoints  = waypoints.cols();
    startTrigger = true;

    kernelLengthParameter = pointToPointDurationVector.segment(1,pointToPointDurationVector.rows()-2).mean();
    pointToPointDurationVector(0) = 0.01 * kernelLengthParameter;
    pointToPointDurationVector.tail(1) << 0.01 * kernelLengthParameter;


    calculateVarianceParameters();
}


void ExperimentalTrajectory::addNewWaypoint(Eigen::VectorXd newWaypoint, double waypointTime)
{
    std::cout << "Adding waypoint at: " << waypointTime << " seconds..." << std::endl;

    if ( (newWaypoint.rows() == nDoF) && (waypointTime>=0.0) && (waypointTime<=kernelCenters.maxCoeff()) )
    {
        //Find out where the new T falls...
        for (int i=0; i<kernelCenters.size(); i++)
        {
            std::cout << "i = " << i << std::endl;
            if (kernelCenters(i)>waypointTime) {
                youngestWaypointIndex=i;
                std::cout << "youngestWaypointIndex" << youngestWaypointIndex << std::endl;
                i = kernelCenters.size();
            }
        }


        Eigen::MatrixXd newWaypointsMat = Eigen::MatrixXd::Zero(nDoF, nWaypoints+1);
        newWaypointsMat.leftCols(youngestWaypointIndex) = waypoints.leftCols(youngestWaypointIndex);
        newWaypointsMat.col(youngestWaypointIndex) = newWaypoint;
        newWaypointsMat.rightCols(nWaypoints - youngestWaypointIndex) = waypoints.rightCols(nWaypoints - youngestWaypointIndex);

        waypoints.resize(nDoF, nWaypoints+1);
        waypoints = newWaypointsMat;

        Eigen::VectorXd durationVectorTmp = Eigen::VectorXd::Zero(nWaypoints);

        std::cout << "\npointToPointDurationVector\n " << pointToPointDurationVector << std::endl;

        durationVectorTmp.head(youngestWaypointIndex-1) = pointToPointDurationVector.head(youngestWaypointIndex-1);
        durationVectorTmp.row(youngestWaypointIndex-1) << waypointTime - kernelCenters(youngestWaypointIndex-1);
        durationVectorTmp.tail(nWaypoints - youngestWaypointIndex) = pointToPointDurationVector.tail(nWaypoints - youngestWaypointIndex);

        pointToPointDurationVector.resize(durationVectorTmp.size());
        pointToPointDurationVector = durationVectorTmp;

        std::cout << "\npointToPointDurationVector\n " << pointToPointDurationVector << std::endl;


        reinitialize();


    }
    else{
        if (newWaypoint.rows() != nDoF){
            std::cout << "[ERROR] (ExperimentalTrajectory::addNewWaypoint): New waypoint is not the right size. Should have dim = " <<nDoF << "x1." << std::endl;
        }
        if ((waypointTime<=0.0) || (waypointTime>=kernelCenters.maxCoeff())){
            std::cout << "[ERROR] (ExperimentalTrajectory::addNewWaypoint): New waypoint time is out of time bounds. Should have fall between 0.0 and " << kernelCenters.maxCoeff() << " seconds." << std::endl;
        }
    }


}

void ExperimentalTrajectory::removeYoungestWaypoint()
{
    removeWaypoint(youngestWaypointIndex);
}

void ExperimentalTrajectory::removeWaypoint(int index)
{
    std::cout << "Removing waypoint index: " << index << "..." << std::endl;
    if ( (index>=0) && (index<nWaypoints) )
    {
        Eigen::MatrixXd newWaypointsMat = Eigen::MatrixXd::Zero(nDoF, nWaypoints-1);

        newWaypointsMat.leftCols(index) = waypoints.leftCols(index);
        newWaypointsMat.rightCols(nWaypoints-1-index) = waypoints.rightCols(nWaypoints-1-index);

        waypoints.resize(nDoF, nWaypoints-1);
        waypoints = newWaypointsMat;

        Eigen::VectorXd durationVectorTmp = Eigen::VectorXd::Zero(nWaypoints-2);

        int durVecIndex = index;
        durationVectorTmp.head(durVecIndex) = pointToPointDurationVector.head(durVecIndex);
        durationVectorTmp.tail(nWaypoints -1 -durVecIndex) = pointToPointDurationVector.tail(nWaypoints -1 -durVecIndex);

        pointToPointDurationVector.resize(durationVectorTmp.size());
        pointToPointDurationVector = durationVectorTmp;


        reinitialize();
    }
    else{
        std::cout << "[ERROR] (ExperimentalTrajectory::addNewWaypoint): New waypoint time is out of index bounds. Should have fall between 0 and " << nWaypoints-1 << "." << std::endl;
    }

}



Eigen::MatrixXd ExperimentalTrajectory::getRbfnKernelCurves()
{
    int nSteps = 1000;
    int nC = kernelCenters.rows();
    double startTime = kernelCenters(0);
    double endTime = kernelCenters.tail(1)(0);
    Eigen::VectorXd timeVec = Eigen::VectorXd::LinSpaced(nSteps, startTime, endTime);

    Eigen::MatrixXd res(timeVec.rows(), nC+1);
    res.col(0) = timeVec;
    res.rightCols(nC) = rbfnKernelFunction(timeVec);

    return res;
}




} //namespace ocra
