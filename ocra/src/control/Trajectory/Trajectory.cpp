#include "ocra/control/Trajectory/Trajectory.h"
#include <math.h>

#define MAX_VEL 0.05

namespace ocra
{

/**
*
* The Trajectory class.
*
*/

/**
* Constructor function.
*/
Trajectory::Trajectory()
: maximumVelocity(MAX_VEL)
, nDoF(-1)
, usingDurationVector(false)
{
}

Trajectory::~Trajectory()
{
    // std::cout << "\nDestroying trajectory object..." << std::endl;
}

void Trajectory::setWaypoints(const std::vector<double>& startingDoubleVec, const std::vector<double>& endingDoubleVec, const int waypointSelector, bool _endsWithQuaternion)
{
    // Make sure that the vectors are the same size
    if (startingDoubleVec.size() != endingDoubleVec.size())
        throw std::runtime_error(std::string("[Trajectory::setWaypoints()]: Starting vector and ending vector are not the same size."));


    int _nRows;
    if (waypointSelector>0) {
        _nRows = waypointSelector;
    }
    else {
        _nRows = startingDoubleVec.size();
    }
    int _nCols = 2;

    Eigen::MatrixXd _waypoints(_nRows, _nCols);

    for(int i=0; i<_nRows; i++){
        _waypoints(i,0)=startingDoubleVec[i];
        _waypoints(i,1)=endingDoubleVec[i];
    }

    setWaypoints(_waypoints, _endsWithQuaternion);
}



void Trajectory::setWaypoints(const Eigen::VectorXd& _startingVector, const Eigen::VectorXd& _endingVector, bool _endsWithQuaternion)
{
    // Make sure that the vectors are the same size
    if (_startingVector.rows() != _endingVector.rows())
        throw std::runtime_error(std::string("[Trajectory::setWaypoints()]: Starting vector and ending vector are not the same size."));


    int _nRows = _startingVector.rows();
    int _nCols = 2;

    Eigen::MatrixXd _waypoints(_nRows, _nCols);

    _waypoints << _startingVector, _endingVector;


    setWaypoints(_waypoints, _endsWithQuaternion);
}

void Trajectory::setWaypoints(Eigen::Displacementd& startingDisplacement, Eigen::Displacementd& endingDisplacement, bool _endsWithQuaternion)
{

    Eigen::VectorXd _startingVector = displacementToEigenVector(startingDisplacement);
    Eigen::VectorXd _endingVector   = displacementToEigenVector(endingDisplacement);

    int _nRows = _startingVector.rows();
    int _nCols = 2;

    Eigen::MatrixXd _waypoints(_nRows, _nCols);
    _waypoints << _startingVector, _endingVector;

    setWaypoints(_waypoints, _endsWithQuaternion);


}

void Trajectory::setWaypoints(Eigen::Rotation3d& startingOrientation, Eigen::Rotation3d& endingOrientation, bool _endsWithQuaternion)
{

    Eigen::VectorXd _startingVector = quaternionToEigenVector(startingOrientation);
    Eigen::VectorXd _endingVector   = quaternionToEigenVector(endingOrientation);

    int _nRows = _startingVector.rows();
    int _nCols = 2;

    Eigen::MatrixXd _waypoints(_nRows, _nCols);
    _waypoints << _startingVector, _endingVector;

    setWaypoints(_waypoints, _endsWithQuaternion);
    // tell class that there is a quaternion in the vector


}

void Trajectory::setWaypoints(const std::list<Eigen::VectorXd>& _waypoints, bool _endsWithQuaternion)
{
    /**
    * Initialization function
    */
    endsWithQuaternion = _endsWithQuaternion;
    waypointList   = _waypoints;
    nDoF        = waypointList.begin()->rows();
    nWaypoints  = waypointList.size();
    startTrigger = true;
    trajectoryFinished = false;
    //Determine number of non-quaternion DoF
    nonRotationDof = (endsWithQuaternion) ? (nDoF - QUATERNION_DIM) : nDoF;

    initializeTrajectory();
}


void Trajectory::setWaypoints(Eigen::MatrixXd& _waypoints, bool _endsWithQuaternion)
{
    /**
    * Initialization function
    */
    endsWithQuaternion = _endsWithQuaternion;
    waypoints   = _waypoints;
    nDoF        = waypoints.rows();
    nWaypoints  = waypoints.cols();
    startTrigger = true;
    currentWaypointIndex = 0;
    trajectoryFinished = false;
    //Determine number of non-quaternion DoF
    nonRotationDof = (endsWithQuaternion) ? (nDoF - QUATERNION_DIM) : nDoF;

    setDuration();

    initializeTrajectory();

}

void Trajectory::setMaxVelocity(double newMaxVel)
{
    maximumVelocity = newMaxVel;
    if (nDoF >= 0) {
        maximumVelocityVector = Eigen::VectorXd::Constant(nDoF, newMaxVel);
    }
}

void Trajectory::setMaxVelocity(const Eigen::VectorXd& newMaxVel)
{
    if (newMaxVel.size()==nDoF) {
        maximumVelocityVector = newMaxVel;
    } else {
        OCRA_ERROR("The size of the newMaxVel vector ("<<newMaxVel.size()<<") doesn't match the nDoF (" << nDoF << ") of the trajectory. Ignoring.")
    }
}

double Trajectory::getMaxVelocity()
{
    return maximumVelocity;
}

Eigen::VectorXd Trajectory::getMaxVelocityVector()
{
    return maximumVelocityVector;
}

void Trajectory::setMaxAcceleration(double newMaxAcc)
{
    maximumAcceleration = newMaxAcc;
    if (nDoF >= 0) {
        maximumAccelerationVector = Eigen::VectorXd::Constant(nDoF, newMaxAcc);
    }
}

void Trajectory::setMaxAcceleration(const Eigen::VectorXd& newMaxAcc)
{
    if (newMaxAcc.size()==nDoF) {
        maximumAccelerationVector = newMaxAcc;
    } else {
        OCRA_ERROR("The size of the newMaxAcc vector ("<<newMaxAcc.size()<<") doesn't match the nDoF (" << nDoF << ") of the trajectory. Ignoring.")
    }
}

double Trajectory::getMaxAcceleration()
{
    return maximumAcceleration;
}

Eigen::VectorXd Trajectory::getMaxAccelerationVector()
{
    return maximumAccelerationVector;
}

void Trajectory::getDesiredValues(double _time, std::vector<double>& _desiredVector)
{
    Eigen::MatrixXd desVals = getDesiredValues(_time);

    // eigenVectorToStdVector(desVals.col(POS_INDEX), _desiredVector);
    eigenMatrixToStdVector(desVals, _desiredVector);
}


void Trajectory::getDesiredValues(double _time, Eigen::Displacementd& _desiredDisplacement)
{
    Eigen::MatrixXd desVals = getDesiredValues(_time);

    eigenVectorToDisplacement(desVals.col(POS_INDEX), _desiredDisplacement);
}

void Trajectory::getDesiredValues(double _time, Eigen::Rotation3d& _desiredOrientation)
{
    Eigen::MatrixXd desVals = getDesiredValues(_time);

    eigenVectorToQuaternion(desVals.col(POS_INDEX), _desiredOrientation);
}

void Trajectory::getDesiredValues(double _time, Eigen::Displacementd& _desiredDisplacement, Eigen::Twistd& _desiredVelocity, Eigen::Twistd& _desiredAcceleration)
{
    Eigen::MatrixXd desVals = getDesiredValues(_time);
    std::cout<< "\n\ndesVals: \n" << desVals << "\n\n";
    eigenVectorToDisplacement(desVals.col(POS_INDEX), _desiredDisplacement);
    eigenVectorToTwist(desVals.col(VEL_INDEX), _desiredVelocity);
    eigenVectorToTwist(desVals.col(ACC_INDEX), _desiredAcceleration);

    Eigen::Displacementd::AdjointMatrix H_adj = Eigen::Displacementd(Eigen::Vector3d::Zero(), _desiredDisplacement.getRotation().inverse()).adjoint();
    _desiredVelocity = H_adj * _desiredVelocity;
    _desiredAcceleration = H_adj * _desiredAcceleration;

}

Eigen::Rotation3d Trajectory::quaternionSlerp(double _tau, Eigen::Rotation3d& _qStart, Eigen::Rotation3d& _qEnd)
{
    /**
    * \param _tau Interpolation variable, 0 <= tau <= 1. Should be something like time/duration.
    * \param _qStart Starting quaternion
    * \param _qEnd Ending quaternion
    */
    Eigen::VectorXd startVec;
    startVec = quaternionToEigenVector(_qStart);
    Eigen::VectorXd endVec;
    endVec  = quaternionToEigenVector(_qEnd);
    if ((startVec - endVec).norm() == 0.0){
        return _qEnd;
    }
    else{

        Eigen::Rotation3d quaternionVector = _qEnd * _qStart.inverse();
        double theta = 2.0 * acos(quaternionVector.w() );
        Eigen::Vector3d new_XYZ;
        new_XYZ << quaternionVector.x(), quaternionVector.y(), quaternionVector.z();
        new_XYZ /= sin(theta/2.0);
        new_XYZ *= sin((_tau*theta)/2.0);
        double new_W = cos((_tau*theta)/2.0);

        Eigen::Rotation3d interpolatedQuaternion = Eigen::Rotation3d(new_W, new_XYZ);
        interpolatedQuaternion *= _qStart;

        return interpolatedQuaternion;
    }
}




/*

void Trajectory::getDesiredValues()
{
    throw std::runtime_error(std::string("[Trajectory::getDesiredValues()]: getDesiredValues has not been implemented or is not supported"));
}

*/

void Trajectory::setDuration()
{
    // Approximate some duration between waypoints based an a velMax of 50cm/s

    // pointToPointDurationVector = Eigen::VectorXd::Constant(nWaypoints-1, 1.0);
    pointToPointDurationVector.resize(nWaypoints-1);

    for (int i=0; i<nWaypoints-1; i++)
    {
        pointToPointDurationVector(i) = (waypoints.col(i+1) - waypoints.col(i)).norm() / maximumVelocity;
    }

    // std::cout << "durationVector: " << pointToPointDurationVector.transpose() << std::endl;
    setDuration(pointToPointDurationVector(0));
}

void Trajectory::setDuration(const Eigen::VectorXd& _pointToPointDurationVector)
{
    if (_pointToPointDurationVector.size() == (nWaypoints-1)) {
        pointToPointDurationVector = _pointToPointDurationVector;
        setDuration(pointToPointDurationVector(0));
        totalTrajectoryDuration = pointToPointDurationVector.sum();
        usingDurationVector = true;
    } else {
        setDuration();
        OCRA_WARNING("The point to point duration vector you passed is not the right size.")
    }
}


void Trajectory::setDuration(double _duration)
{
    pointToPointDuration = _duration;
    totalTrajectoryDuration = pointToPointDuration*(nWaypoints-1);
}



/********************************************************************************************************
*********************************************************************************************************
*********************************************************************************************************
********************************************************************************************************/

/**
*
* Useful auxiliary functions
*
*/


Eigen::VectorXd Trajectory::displacementToEigenVector(Eigen::Displacementd& _disp)
{
    /**
    * Convert from a Displacement (double) to and Eigen Vector (double)
    * Stored as [x, y, z, qw, qx, qy, qz]^T
    */
    Eigen::VectorXd outputVector(7);
    double x, y, z, qx, qy, qz, qw;
    x  = _disp.getTranslation().x();
    y  = _disp.getTranslation().y();
    z  = _disp.getTranslation().z();
    qx = _disp.getRotation().x();
    qy = _disp.getRotation().y();
    qz = _disp.getRotation().z();
    qw = _disp.getRotation().w();
    outputVector << x, y, z, qw, qx, qy, qz;

    return outputVector;
};


Eigen::VectorXd Trajectory::quaternionToEigenVector(Eigen::Rotation3d& _quat)
{
    /**
    * Convert from a Quaternion (double) to and Eigen Vector (double)
    * Stored as [qw, qx, qy, qz]^T
    */
    Eigen::VectorXd outputVector(4);
    double qx, qy, qz, qw;
    qx = _quat.x();
    qy = _quat.y();
    qz = _quat.z();
    qw = _quat.w();
    outputVector << qw, qx, qy, qz;

    return outputVector;
};

bool Trajectory::eigenVectorToStdVector(const Eigen::VectorXd& _dispVec, std::vector<double>& _doubleVec)
{
    if (_doubleVec.size() == _dispVec.rows()){
        for(int i=0; i<_dispVec.rows(); i++){
            _doubleVec[i] = _dispVec[i];
        }
        return true;
    }
    else
        return false;
}

bool Trajectory::eigenMatrixToStdVector(const Eigen::MatrixXd& _dispMat, std::vector<double>& _doubleVec)
{
    int nValues = _dispMat.size();
    if (nValues == _doubleVec.size()) {
        const double *eigPtr = _dispMat.data();
        for(int i=0; i<nValues; i++){
            _doubleVec[i] = *eigPtr;
            eigPtr++;
        }
        return true;
    }
    else {
        // std::cout << "[ERROR] (Trajectory::eigenMatrixToStdVector): Matrix size and vector size do not match." << std::endl;
        return false;
    }
}


bool Trajectory::eigenVectorToDisplacement(const Eigen::VectorXd& _dispVec, Eigen::Displacementd& _disp)
{
    Eigen::Vector3d translation;
    Eigen::Rotation3d rotation;


    if (_dispVec.rows()==3)
    {
        translation = _dispVec;
        rotation = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0); // no rotation
    }

    else if((_dispVec.rows()==4) && (endsWithQuaternion))
    {
        translation << 0.0, 0.0, 0.0; // not necessarily the best solution... maybe error checking here
        Trajectory::eigenVectorToQuaternion(_dispVec, rotation);
    }

    else if(_dispVec.rows()==7)
    {
        translation = _dispVec.head(TRANSLATION_DIM);
        Trajectory::eigenVectorToQuaternion(_dispVec.tail(QUATERNION_DIM), rotation);
    }
    else
    {
        _disp = Eigen::Displacementd(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        return false;
    }

    _disp = Eigen::Displacementd(translation, rotation);
    return true;
};

bool Trajectory::eigenVectorToQuaternion(const Eigen::VectorXd& _quatVec, Eigen::Rotation3d& _quat)
{
    if ( (_quatVec.rows()==4) && ( endsWithQuaternion ) )
    {
        _quat = Eigen::Rotation3d(_quatVec(0), _quatVec(1), _quatVec(2), _quatVec(3));
        return true;
    }
    else
    {
        _quat = Eigen::Rotation3d(1.0, 0.0, 0.0, 0.0);
        return false;
    }

};


bool Trajectory::eigenVectorToTwist(const Eigen::VectorXd& _twistVec, Eigen::Twistd& _twist)
{
    Eigen::Vector3d linearComponent, angularComponent;


    if (_twistVec.rows()==3)
    {
        linearComponent = _twistVec;
        angularComponent << 0.0, 0.0, 0.0; // no angularComponent
    }

    else if((_twistVec.rows()==4) && (endsWithQuaternion))
    {
        linearComponent << 0.0, 0.0, 0.0; // not necessarily the best solution... maybe error checking here
        angularComponent = _twistVec.head(TRANSLATION_DIM);
    }

    else if(_twistVec.rows()==7)
    {
        linearComponent = _twistVec.head(TRANSLATION_DIM);
        angularComponent = _twistVec.segment(TRANSLATION_DIM, TRANSLATION_DIM);
    }
    else
    {
        _twist = Eigen::Twistd::Zero();
        return false;
    }

    _twist = Eigen::Twistd(angularComponent, linearComponent);
    return true;
};


Eigen::MatrixXd Trajectory::getFullTrajectory(double dt)
{
    double t = 0.0;
    int nCols = nDoF*3;
    int rowCounter = 0;
    int nRows = std::ceil(totalTrajectoryDuration / dt)*2;

    Eigen::MatrixXd posVelAcc(nRows, nCols);
    while (t <= totalTrajectoryDuration) {
        Eigen::MatrixXd vals = getDesiredValues(t);
        posVelAcc.row(rowCounter) << vals.col(POS_INDEX).transpose(), vals.col(VEL_INDEX).transpose(), vals.col(ACC_INDEX).transpose();
        t += dt;
        ++rowCounter;
    }

    return posVelAcc.topRows(rowCounter-1);
}

// bool Trajectory::dumpToFile(const Eigen::MatrixXd& _desiredVals)
// {
//     std::ofstream dataFile;
//     dataFile.open("~/Desktop/trajectoryDataDump.txt", std::ios::app);
//     if (dataFile.is_open())
//     {
//         dataFile << _desiredVals;
//         dataFile << "\n";
//         dataFile.close();
//         return true;
//     }
//     else
//     {
//         std::cout << "Unable to open file"<< std::endl;
//         return false;
//     }
// };


} //namespace ocra
