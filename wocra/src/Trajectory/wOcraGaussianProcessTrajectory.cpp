
#include "wocra/Trajectory/wOcraGaussianProcessTrajectory.h"

#ifndef ALPHA_L
#define ALPHA_L 1.0
#endif

#ifndef VAR_LENGTH_FACTOR
#define VAR_LENGTH_FACTOR 0.1
#endif



#ifndef MEAN_LENGTH_FACTOR
#define MEAN_LENGTH_FACTOR 20.0//10.0
#endif

namespace wocra
{

void wOcraGaussianProcessTrajectory::initializeTrajectory()
{
    meanGP = new smlt::gaussianProcess();
    varianceGP = new smlt::gaussianProcess();


    // Store the original user defined waypoints
    originalWaypoints = waypoints;

    extraWaypoints = 1;
    extraWpDt = 0.01 * pointToPointDurationVector.mean(); // meanLengthParameter;


    numberOfOptimizationWaypoints = 1; // by default only one waypoint is used in optimization.
    boptVariablesSet = false; // must set all starting bopt values to get the other optimization parameters

    timeline = Eigen::VectorXd::Zero(waypoints.cols());
    for(int i=0; i<waypoints.cols()-1; i++){
        timeline(i+1) = timeline(i) + pointToPointDurationVector(i);
    }

    boolVector tmp;
    std::vector<Eigen::VectorXi> tmpDofToOptVec;
    setMeanWaypoints(tmp);
    setVarianceWaypoints(tmp);
    setOptimizationWaypoints(tmp);
    setDofToOptimize(tmpDofToOptVec);


    calculateGaussianProcessParameters();
}


void wOcraGaussianProcessTrajectory::printWaypointData()
{
    int w1 = 12;
    int w2 = 5;
    int w3 = 8;
    int w4 = 13;
    int w5 = 12;
    int w6 = 12;
    int w7 = 9;


    std::cout << std::setw(1) << "|" << std::setw(w1) << "waypoint #";
    std::cout << std::setw(2) << "|" << std::setw(w2) << " time";
    std::cout << std::setw(2) << "|" << std::setw(w3) << " coordinates";
    std::cout << std::setw(2) << "|" << std::setw(w4) << " meanWaypoint";
    std::cout << std::setw(2) << "|" << std::setw(w5) << " varWaypoint";
    std::cout << std::setw(2) << "|" << std::setw(w6) << " optWaypoint";
    std::cout << std::setw(2) << "|" << std::setw(w7) << " optIndexes";
    std::cout << std::setw(2) << "|";
    std::cout << std::endl;

    for(int i=0; i<waypoints.cols(); i++)
    {
        std::cout << std::setw(1) << "|" << std::setw(w1) << i;
        std::cout << std::setw(2) << "|" << std::setw(w2) << timeline(i);
        std::cout << std::setw(2) << "|" << std::setw(w3) << waypoints.col(i).transpose();
        std::cout << std::setw(2) << "|" << std::setw(w4) << isMeanWaypoint[i];
        std::cout << std::setw(2) << "|" << std::setw(w5) << isVarWaypoint[i];
        std::cout << std::setw(2) << "|" << std::setw(w6) << isOptWaypoint[i];
        std::cout << std::setw(2) << "|" << std::setw(w7) << dofToOptimize[i].transpose();
        std::cout << std::setw(2) << "|";
        std::cout << std::endl;
    }
}


bool wOcraGaussianProcessTrajectory::setMeanWaypoints(boolVector& bMeanVec)
{
    bool retval;
    if (bMeanVec.empty()) {
        isMeanWaypoint.resize(waypoints.cols());
        for (int i=0; i<waypoints.cols(); i++){
            isMeanWaypoint[i] = true;
        }
    }
    else if (bMeanVec.size()==waypoints.cols()) {
        isMeanWaypoint = bMeanVec;
        retval = true;
    }
    else {
        retval = false;
        std::cout << "[ERROR] (): The number of vector entries must match the number of waypoints ("<< waypoints.cols() <<")." << std::endl;
    }


    if (gpParametersAreSet()) {
        calculateGaussianProcessParameters();
    }

    return retval;
}

bool wOcraGaussianProcessTrajectory::setVarianceWaypoints(boolVector& bVarVec)
{
    bool retval;
    if (bVarVec.empty()) {
        isVarWaypoint.resize(waypoints.cols());
        for (int i=0; i<waypoints.cols(); i++){
            isVarWaypoint[i] = true;
        }
    }

    else if (bVarVec.size()==waypoints.cols()) {
        isVarWaypoint = bVarVec;
        retval = true;
    }
    else {
        retval = false;
        std::cout << "[ERROR] (): The number of vector entries must match the number of waypoints ("<< waypoints.cols() <<")." << std::endl;
    }


    if (gpParametersAreSet()) {
        calculateGaussianProcessParameters();
    }

    return retval;
}

bool wOcraGaussianProcessTrajectory::setOptimizationWaypoints(boolVector& bOptVec)
{
    bool retval;
    if (bOptVec.empty()) {
        isOptWaypoint.resize(waypoints.cols());
        for (int i=0; i<waypoints.cols(); i++){
            isOptWaypoint[i] = false;
        }
    }
    else if (bOptVec.size()==waypoints.cols()) {
        isOptWaypoint = bOptVec;
        retval = true;
    }
    else {
        retval = false;
        std::cout << "[ERROR] (): The number of vector entries must match the number of waypoints ("<< waypoints.cols() <<")." << std::endl;
    }

    return retval;
}

bool wOcraGaussianProcessTrajectory::setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptVec)
{
    bool retval;

    if (dofToOptVec.empty()) {
        dofToOptimize.resize(waypoints.cols());
        for (int i=0; i<waypoints.cols(); i++){
            Eigen::VectorXi tmpVec = Eigen::VectorXi::LinSpaced(nDoF+1, 0, nDoF); // all DOF
            dofToOptimize[i] = tmpVec;
        }
    }
    else if (dofToOptVec.size()==waypoints.cols()) {
        dofToOptimize = dofToOptVec;
        retval = true;
    }
    else {
        retval = false;
        std::cout << "[ERROR] (): The number of vector entries must match the number of waypoints ("<< waypoints.cols() <<")." << std::endl;
    }

    return retval;
}

bool wOcraGaussianProcessTrajectory::gpParametersAreSet()
{
    bool retVal = true;
    retVal = retVal && isMeanWaypoint.size() == waypoints.cols();
    retVal = retVal && isVarWaypoint.size() == waypoints.cols();
    return retVal;
}


void wOcraGaussianProcessTrajectory::calculateGaussianProcessParameters()
{

    // Add a waypoint(s) to the ends of the movement.
    int meanCounter = 0;
    int varCounter = 0;
    for(int i=0; i<waypoints.cols(); i++){
        if (isMeanWaypoint[i]){
             meanCounter++;
        }
        if (isVarWaypoint[i]){
             varCounter++;
        }
    }

    meanKernelCenters.resize(meanCounter);
    meanKernelTrainingData.resize(nDoF, meanCounter);
    varKernelCenters.resize(varCounter);
    varKernelTrainingData.resize(nDoF, varCounter);

    int meanTDColCounter = 0;
    int varTDColCounter = 0;

    for(int i=0; i<waypoints.cols(); i++)
    {
        if (isMeanWaypoint[i])
        {
            meanKernelCenters(meanTDColCounter) = timeline(i);
            meanKernelTrainingData.col(meanTDColCounter) << waypoints.col(i);
            meanTDColCounter++;
        }


        if (isVarWaypoint[i])
        {
            varKernelCenters(varTDColCounter) = timeline(i);
            varKernelTrainingData.col(varTDColCounter) << waypoints.col(i);
            varTDColCounter++;
        }
    }


    int minIdx;
    double minMeanTime = meanKernelCenters.minCoeff(&minIdx);


    int maxIdx;
    double maxMeanTime = meanKernelCenters.maxCoeff(&maxIdx);

    minMeanTime += extraWpDt;
    maxMeanTime += extraWpDt;



    Eigen::VectorXd extraWptTimes(2); extraWptTimes << minMeanTime, maxMeanTime;

    Eigen::MatrixXd extraWpts(nDoF, 2); extraWpts << meanKernelTrainingData.col(minIdx), meanKernelTrainingData.col(maxIdx);

    meanKernelCenters = vStack(meanKernelCenters, extraWptTimes);

    meanKernelTrainingData = hStack(meanKernelTrainingData, extraWpts);


    if (isVarWaypoint[minIdx]){
        Eigen::VectorXd extraWptT(1); extraWptT << minMeanTime;
        varKernelCenters = vStack(varKernelCenters, extraWptT);
        varKernelTrainingData = hStack(varKernelTrainingData, meanKernelTrainingData.col(minIdx) );
    }


    if (isVarWaypoint[maxIdx]){
        Eigen::VectorXd extraWptT(1); extraWptT << maxMeanTime;
        varKernelCenters = vStack(varKernelCenters, extraWptT);
        varKernelTrainingData = hStack(varKernelTrainingData, meanKernelTrainingData.col(maxIdx) );
    }

    // Calculate the max covariance vector for the movement.
    maxCovariance = ( waypoints - waypoints.rowwise().mean().replicate(1, waypoints.cols()) ).array().square().rowwise().sum() / (waypoints.cols() - 1) ;

    // Get the length param to be used on the movement.
    varLengthParameter = pointToPointDurationVector.mean() * VAR_LENGTH_FACTOR;

    // Get the length param to be used for the calculation of covariance
    meanLengthParameter =  pointToPointDurationVector.mean() * MEAN_LENGTH_FACTOR;

    Eigen::MatrixXd meanCovMat(1,1);
    meanCovMat << meanLengthParameter;
    Eigen::MatrixXd varCovMat(1,1);
    varCovMat << varLengthParameter;

    meanGP->setKernelCenters(Eigen::MatrixXd(meanKernelCenters.transpose()));
    meanGP->setKernelTrainingData(meanKernelTrainingData);
    meanGP->setCovarianceMatrix(meanCovMat);
    meanGP->setMaxCovariance(maxCovariance);
    meanGP->calculateParameters();

    varianceGP->setKernelCenters(Eigen::MatrixXd(varKernelCenters.transpose()));
    varianceGP->setKernelTrainingData(varKernelTrainingData);
    varianceGP->setCovarianceMatrix(varCovMat);
    varianceGP->setMaxCovariance(maxCovariance);
    varianceGP->calculateParameters();
    startTrigger = true;
    varianceStartTrigger = true;

}


Eigen::MatrixXd wOcraGaussianProcessTrajectory::getDesiredValues(double _time)
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
        int minTimeIndex; timeline.minCoeff(&minTimeIndex);
        desiredValue.col(POS_INDEX) = waypoints.col(minTimeIndex);
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

        if (t<=meanKernelCenters.maxCoeff())
        {
            Eigen::VectorXd tVec = Eigen::VectorXd::Constant(1, t);
            Eigen::VectorXd posMean;
            meanGP->calculateMean(tVec, posMean);
            desiredValue.col(POS_INDEX) = posMean;
        }
        else
        {
            int maxTimeIndex; meanKernelCenters.maxCoeff(&maxTimeIndex);

            desiredValue.col(POS_INDEX) = meanKernelTrainingData.col(maxTimeIndex);
            trajectoryFinished = true;
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

Eigen::VectorXd wOcraGaussianProcessTrajectory::getVariance(double _time)
{
    if (varianceStartTrigger) {
        t0_variance = _time;
        varianceStartTrigger = false;
    }


    double t = _time - t0_variance;


    Eigen::VectorXd variance;

    if (t<=meanKernelCenters.maxCoeff())
    {
        Eigen::VectorXd tVec = Eigen::VectorXd::Constant(1, t);
        varianceGP->calculateVariance(tVec, variance);
    }
    else{
        variance = Eigen::VectorXd::Zero(maxCovariance.rows());
        trajectoryFinished = true;

    }

    return variance;
}

void wOcraGaussianProcessTrajectory::getDesiredValues(double _time, Eigen::MatrixXd& desiredValues, Eigen::VectorXd& variance)
{
    desiredValues = getDesiredValues(_time);
    variance = getVariance(_time);
}

void wOcraGaussianProcessTrajectory::getDesiredValues(double _time, Eigen::Displacementd& _position, Eigen::Twistd& _velocity, Eigen::Twistd& _acceleration)
{
}


Eigen::MatrixXd wOcraGaussianProcessTrajectory::getWaypointData()
{
    Eigen::MatrixXd dataMat = Eigen::MatrixXd::Zero(waypoints.cols(), nDoF+1);

    dataMat.col(0) = timeline;
    dataMat.rightCols(nDoF) = waypoints.transpose();

    return dataMat;
}

Eigen::MatrixXd wOcraGaussianProcessTrajectory::getMeanGPData()
{
    Eigen::MatrixXd dataMat = Eigen::MatrixXd::Zero(meanKernelCenters.rows(), nDoF+1);

    dataMat.col(0) = meanKernelCenters;
    dataMat.rightCols(nDoF) = meanKernelTrainingData.transpose();

    return dataMat;
}

Eigen::MatrixXd wOcraGaussianProcessTrajectory::getVarGPData()
{
    Eigen::MatrixXd dataMat = Eigen::MatrixXd::Zero(varKernelCenters.rows(), nDoF+1);

    dataMat.col(0) = varKernelCenters;
    dataMat.rightCols(nDoF) = varKernelTrainingData.transpose();

    return dataMat;
}

void wOcraGaussianProcessTrajectory::addWaypoint(const Eigen::VectorXd newWaypoint, const double waypointTime)
{
    addWaypoint(newWaypoint, waypointTime, Eigen::VectorXi::LinSpaced(nDoF+1, 0, nDoF));
}

void wOcraGaussianProcessTrajectory::addWaypoint(const Eigen::VectorXd newWaypoint, const double waypointTime, const Eigen::VectorXi& _dofToOpt, const bool useForMean, const bool useForVar, const bool useForOpt)
{
    Eigen::VectorXi dofToOpt = _dofToOpt;
    if ( (newWaypoint.rows() == nDoF) && (waypointTime>=0.0) )
    {
        waypoints = hStack(waypoints, newWaypoint);
        Eigen::VectorXd wyptime(1); wyptime<< waypointTime;
        timeline = vStack(timeline, wyptime);

        isMeanWaypoint.push_back(useForMean);
        isVarWaypoint.push_back(useForVar);
        isOptWaypoint.push_back(useForOpt);

        if (dofToOpt.rows()==0) {
            dofToOpt.resize(nDoF+1);
            dofToOpt << Eigen::VectorXi::LinSpaced(nDoF+1, 0, nDoF);
        }else if (dofToOpt.rows()>nDoF+1) {
            dofToOpt = dofToOpt.head(nDoF).eval();
        }
        dofToOptimize.push_back(dofToOpt);

        calculateGaussianProcessParameters();


    }
    else{
        if (newWaypoint.rows() != nDoF){
            std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::addWaypoint): New waypoint is not the right size. Should have dim = " <<nDoF << "x1." << std::endl;
        }
        if (waypointTime<=0.0){
            std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::addWaypoint): New waypoint time should be greater than 0.0 seconds." << std::endl;
        }
    }


}

void wOcraGaussianProcessTrajectory::removeWaypoint(int index)
{
    // std::cout << "Removing waypoint index: " << index << "..." << std::endl;
    if ( (index>=0) && (index<waypoints.cols()) )
    {

        removeCols(waypoints, index, 1);
        removeRows(meanKernelCenters, index, 1);

        isMeanWaypoint.erase(isMeanWaypoint.begin()+index);
        isVarWaypoint.erase(isVarWaypoint.begin()+index);
        isOptWaypoint.erase(isOptWaypoint.begin()+index);
        dofToOptimize.erase(dofToOptimize.begin()+index);


        calculateGaussianProcessParameters();
    }
    else
    {
        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::addWaypoint): New waypoint time is out of index bounds. Should have fall between 0 and " << nWaypoints-1 << "." << std::endl;
    }

}


Eigen::MatrixXd wOcraGaussianProcessTrajectory::getBoptCovarianceMatrix()
{

    int optVectorSize = 0;
    for (int i =0; i<waypoints.cols(); i++)
    {
        if (isOptWaypoint[i]){optVectorSize+=dofToOptimize[i].size();}
    }

    Eigen::MatrixXd covarianceMatrix = Eigen::MatrixXd::Identity(optVectorSize,optVectorSize);
    Eigen::VectorXd maxVarVec = getMaxCovarianceVector();

    if (boptVariablesSet) {
        int indexCounter = 0;
        for (int i =0; i<waypoints.cols(); i++)
        {
            if (isOptWaypoint[i])
            {
                for(int j=0; j<dofToOptimize[i].size(); j++)
                {

                    if (dofToOptimize[i](j)==0)
                    {
                        covarianceMatrix(indexCounter,indexCounter) = meanLengthParameter;//getVarianceLengthParameter()*1.0;
                    }
                    else if (dofToOptimize[i](j)>0 && dofToOptimize[i](j)<=nDoF)
                    {
                        covarianceMatrix(indexCounter,indexCounter) = maxVarVec(j-1)*1.0;
                    }
                    else
                    {
                        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptCovarianceMatrix): You are trying to optimize an index which doesn't exist! Ignoring." << std::endl;
                    }
                    indexCounter++;
                }
            }
        }

    }
    else {
        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptCovarianceMatrix): Bayesian optimization variables have not yet been set. Please call wOcraGaussianProcessTrajectory::getBoptVariables(const int extraPointsToAdd) first." << std::endl;
    }
    return  covarianceMatrix;
}

Eigen::VectorXd wOcraGaussianProcessTrajectory::getBoptSearchSpaceMinBound()
{
    int optVectorSize = 0;
    for (int i =0; i<waypoints.cols(); i++)
    {
        if (isOptWaypoint[i]){optVectorSize+=dofToOptimize[i].size();}
    }

    Eigen::VectorXd minBound(optVectorSize);

    if (boptVariablesSet)
    {
        double timeMin = timeline.minCoeff() + (extraWpDt*10.0);
        Eigen::VectorXd minDofCoord(waypoints.rows());
        minDofCoord << waypoints.rowwise().minCoeff();

        int indexCounter = 0;
        for (int i =0; i<waypoints.cols(); i++)
        {
            if (isOptWaypoint[i])
            {
                for(int j=0; j<dofToOptimize[i].size(); j++)
                {

                    if (dofToOptimize[i](j)==0)
                    {
                        minBound(indexCounter) = timeMin;
                    }
                    else if (dofToOptimize[i](j)>0 && dofToOptimize[i](j)<=nDoF)
                    {
                        minBound(indexCounter) = minDofCoord(dofToOptimize[i](j)-1);
                    }
                    else
                    {
                        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptSearchSpaceMinBound): You are trying to optimize an index which doesn't exist! Ignoring." << std::endl;
                    }
                    indexCounter++;
                }
            }
        }
    }
    else {
        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptSearchSpaceMinBound): Bayesian optimization variables have not yet been set. Please call wOcraGaussianProcessTrajectory::getBoptVariables(const int extraPointsToAdd) first." << std::endl;
    }

    return minBound;
}

Eigen::VectorXd wOcraGaussianProcessTrajectory::getBoptSearchSpaceMaxBound()
{
    int optVectorSize = 0;
    for (int i =0; i<waypoints.cols(); i++)
    {
        if (isOptWaypoint[i]){optVectorSize+=dofToOptimize[i].size();}
    }

    Eigen::VectorXd maxBound(optVectorSize);

    if (boptVariablesSet)
    {
        double timeMax = timeline.maxCoeff() - (extraWpDt*10.0);
        Eigen::VectorXd maxDofCoord(waypoints.rows());
        maxDofCoord << waypoints.rowwise().maxCoeff();

        int indexCounter = 0;
        for (int i =0; i<waypoints.cols(); i++)
        {
            if (isOptWaypoint[i])
            {
                for(int j=0; j<dofToOptimize[i].size(); j++)
                {

                    if (dofToOptimize[i](j)==0)
                    {
                        maxBound(indexCounter) = timeMax;
                    }
                    else if (dofToOptimize[i](j)>0 && dofToOptimize[i](j)<=nDoF)
                    {
                        maxBound(indexCounter) = maxDofCoord(dofToOptimize[i](j)-1);
                    }
                    else
                    {
                        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptSearchSpaceMaxBound): You are trying to optimize an index which doesn't exist! Ignoring." << std::endl;
                    }
                    indexCounter++;
                }
            }
        }
    }
    else {
        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptSearchSpaceMaxBound): Bayesian optimization variables have not yet been set. Please call wOcraGaussianProcessTrajectory::getBoptVariables(const int extraPointsToAdd) first." << std::endl;
    }

    return maxBound;

}


Eigen::VectorXd wOcraGaussianProcessTrajectory::getBoptVariables(const int extraPointsToAdd, std::vector<Eigen::VectorXi> dofToOptVec)
{
    int nP;
    int nW = originalWaypoints.cols();

    bool optimizeExistingWaypoints = false;
    for (int i=0; i<waypoints.cols(); i++){
        optimizeExistingWaypoints = optimizeExistingWaypoints || isOptWaypoint[i];
    }

    if (extraPointsToAdd<=0 && !optimizeExistingWaypoints) {
        if (nW<3) {nP = 1;}
        else{nP = 2;}
    }else{nP = extraPointsToAdd;}

    if (nP>0) {


        numberOfOptimizationWaypoints = nP;
        // int optDim = (nDoF+1)*nP; // +1 for the time dimension
        // Eigen::MatrixXd variables = Eigen::MatrixXd::Zero(optDim, 1);

        Eigen::VectorXd timeVec = Eigen::VectorXd::LinSpaced(nP+2, 0.0, timeline.maxCoeff()).segment(1,nP);

        for (int i=0; i<nP; i++)
        {
            double tmpTime = timeVec(i);
            Eigen::VectorXd tVec = Eigen::VectorXd::Constant(1, tmpTime);
            Eigen::VectorXd tmpWaypt;
            meanGP->calculateMean(tVec, tmpWaypt);
            if (nP==dofToOptVec.size()) {
                addWaypoint(tmpWaypt, tmpTime, dofToOptVec[i]);
            }
            else {
                addWaypoint(tmpWaypt, tmpTime);
            }

        }
    }

    int optVectorSize = 0;
    for (int i =0; i<waypoints.cols(); i++)
    {
        if (isOptWaypoint[i]){optVectorSize+=dofToOptimize[i].size();}
    }

    Eigen::VectorXd optVector = Eigen::VectorXd::Zero(optVectorSize);
    int indexCounter = 0;
    for (int i =0; i<waypoints.cols(); i++)
    {
        if (isOptWaypoint[i]) {
            for(int j=0;j<dofToOptimize[i].size();j++){
                if (dofToOptimize[i](j)==0) {
                    optVector(indexCounter) = timeline(i);
                }else if (dofToOptimize[i](j)>0 && dofToOptimize[i](j)<=nDoF) {
                    optVector(indexCounter) = waypoints(dofToOptimize[i](j)-1, i);
                }
                else{
                    std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptVariables): You are trying to optimize an index which doesn't exist! Ignoring and setting to zero." << std::endl;
                }
                indexCounter++;

            }
        }
    }

    boptVariablesSet = true;
    return optVector;

}

bool wOcraGaussianProcessTrajectory::setBoptVariables(const Eigen::VectorXd& newOptVariables)
{
    int optVectorSize = 0;
    for (int i =0; i<waypoints.cols(); i++)
    {
        if (isOptWaypoint[i]){optVectorSize+=dofToOptimize[i].size();}
    }
    bool retval = false;

    if (newOptVariables.rows() == optVectorSize)
    {
        int indexCounter = 0;
        for (int i =0; i<waypoints.cols(); i++)
        {
            if (isOptWaypoint[i])
            {
                for(int j=0; j<dofToOptimize[i].size(); j++)
                {

                    if (dofToOptimize[i](j)==0)
                    {
                        timeline(i) = newOptVariables(indexCounter);
                    }
                    else if (dofToOptimize[i](j)>0 && dofToOptimize[i](j)<=nDoF)
                    {
                        waypoints(dofToOptimize[i](j)-1, i) = newOptVariables(indexCounter);
                    }
                    else
                    {
                        std::cout << "[ERROR] (wOcraGaussianProcessTrajectory::getBoptVariables): You are trying to change an index which doesn't exist! Ignoring." << std::endl;
                    }
                    indexCounter++;
                }
            }
        }
        calculateGaussianProcessParameters();
        retval = true;
    }
    else
    {
        std::cout << "[ERROR] (): The new optimization variable vector size (" << newOptVariables.rows() << ") doesn't match what I was expecting ("<<  optVectorSize <<")." << std::endl;
        retval = false;
    }

    return retval;

}


// smlt::bopt_Parameters wOcraGaussianProcessTrajectory::getBoptParameters()
// {
//     smlt::bopt_Parameters bopt_params = smlt::bopt_Parameters();
//
//     bopt_params.dataLogDir = "./tmp/";
//     bopt_params.maxIter = 20;
//     bopt_params.gridSpacing = Eigen::VectorXd::Constant(2, 1.0);
//     bopt_params.costMaxCovariance = Eigen::VectorXd::Ones(1);
//
//
//     bopt_params.searchSpaceMinBound = getBoptSearchSpaceMinBound();
//
//     bopt_params.searchSpaceMaxBound = getBoptSearchSpaceMaxBound();
//
//     bopt_params.costCovariance = getBoptCovarianceMatrix();
//
//     return bopt_params;
// }


void wOcraGaussianProcessTrajectory::precalculateTrajectory(Eigen::MatrixXd& _traj, Eigen::MatrixXd& _variance, Eigen::VectorXd& _timeline, const double DT)
{
    double currentTime = 0.0;

    int numberOfDataPoints = ceil((pointToPointDurationVector.sum() / DT)*2);

    Eigen::MatrixXd traj(numberOfDataPoints, nDoF*3);
    Eigen::MatrixXd variance(numberOfDataPoints, nDoF);
    Eigen::VectorXd timeline(numberOfDataPoints);


    int index = 0;
    startTrigger = true;
    trajectoryFinished = false;

    while (!trajectoryFinished)
    {
        Eigen::MatrixXd desValsMat;
        Eigen::VectorXd desVarianceVec;

        getDesiredValues(currentTime, desValsMat, desVarianceVec);

        traj.row(index) <<  desValsMat.col(POS_INDEX).transpose(),
                            desValsMat.col(VEL_INDEX).transpose(),
                            desValsMat.col(ACC_INDEX).transpose();

        variance.row(index) << desVarianceVec.transpose();

        timeline(index) = currentTime;
        currentTime += DT;
        index++;
    }
    //  reset values to their original state
    startTrigger = true;
    trajectoryFinished = false;

    _traj = traj.topRows(index);
    _variance = variance.topRows(index);
    _timeline = timeline.head(index);


}

void wOcraGaussianProcessTrajectory::saveTrajectoryToFile(const std::string dirPath)
{
    smlt::checkAndCreateDirectory(dirPath);
    std::string filePath = dirPath + "/trajectory.txt";

    std::ofstream trajFile;
    trajFile.open(filePath.c_str());
    if (trajFile.is_open()) {
        Eigen::VectorXd timeVec;
        Eigen::MatrixXd trajMat;
        Eigen::MatrixXd varianceMat;
        precalculateTrajectory(trajMat, varianceMat, timeVec);
        for(int i=0; i<trajMat.rows(); i++)
        {
            trajFile << timeVec(i) << " " << trajMat.row(i) << " " << varianceMat.row(i) << std::endl;
        }
        trajFile.close();
    }
    else
    {
        std::cout << "[ERROR](line: "<< __LINE__ <<") -> Could not write the trajectory to file, " << filePath << std::endl;
    }

}


void wOcraGaussianProcessTrajectory::saveWaypointDataToFile(const std::string dirPath)
{
    smlt::checkAndCreateDirectory(dirPath);
    std::string filePath = dirPath + "/waypoints.txt";

    std::ofstream waypointFile;
    waypointFile.open(filePath.c_str());
    if (waypointFile.is_open()) {
        waypointFile << getWaypointData();
        waypointFile.close();
    }
    else
    {
        std::cout << "[ERROR](line: "<< __LINE__ <<") -> Could not write the trajectory to file, " << filePath << std::endl;
    }
}


} //namespace wocra
