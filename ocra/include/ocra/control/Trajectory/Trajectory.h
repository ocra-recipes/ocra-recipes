#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Lgsm>

#include <string>
#include <fstream>
#include <list>

#include <ocra/util/Macros.h>


#define TRANSLATION_DIM 3
#define QUATERNION_DIM 4
#define POS_INDEX 0
#define VEL_INDEX 1
#define ACC_INDEX 2

#define TRAJ_DIM 3

#define TAU_MAX 1.0l

namespace ocra
{

class Trajectory {
    public:
        // Constructor function
        Trajectory();

        void setMaxVelocity(double newMaxVel){maximumVelocity = newMaxVel;}
        double getMaxVelocity(){return maximumVelocity;}

        void setWaypoints(const std::vector<double>& startingDoubleVec, const std::vector<double>& endingDoubleVec, const int waypointSelector=0, bool endsWithQuaternion=false);
        void setWaypoints(const Eigen::VectorXd& startingVector, const Eigen::VectorXd& endingVector, bool endsWithQuaternion=false);
        void setWaypoints(Eigen::Displacementd& startingDisplacement, Eigen::Displacementd& endingDisplacement, bool endsWithQuaternion=true);
        void setWaypoints(Eigen::Rotation3d& startingOrientation, Eigen::Rotation3d& endingOrientation, bool endsWithQuaternion=true);

        // set waypoints
        void setWaypoints(Eigen::MatrixXd& waypoints, bool endsWithQuaternion=false);

        //Destructor
        ~Trajectory();

        // Primary user interface functions
        void setDuration();
        void setDuration(double time);

        bool isFinished(){return trajectoryFinished;}

        // virtual void getDesiredValues(){};
        //virtual Eigen::VectorXd getDesiredValues(double time){return Eigen::VectorXd::Zero(nDim)};
        virtual Eigen::MatrixXd getDesiredValues(double time){return Eigen::MatrixXd::Zero(nDoF, TRAJ_DIM);};

        // Note: only valid for "stochastic" trajectories, which for the moment means just GaussianProcess Trajectories.
        virtual void getDesiredValues(double time, Eigen::MatrixXd& desiredValues, Eigen::VectorXd& variance)
        {
            desiredValues = Eigen::MatrixXd::Zero(nDoF, TRAJ_DIM);
            variance = Eigen::VectorXd::Zero(nDoF);
        };


        void getDesiredValues(double time, std::vector<double>& doubleVec);
        void getDesiredValues(double time, Eigen::Displacementd& disp);
        void getDesiredValues(double time, Eigen::Rotation3d& orient);
        void getDesiredValues(double time, Eigen::Displacementd& pos, Eigen::Twistd& vel, Eigen::Twistd& acc);



        Eigen::Rotation3d quaternionSlerp(double tau, Eigen::Rotation3d& qStart, Eigen::Rotation3d& qEnd);

        // Useful auxiliary functions
        Eigen::VectorXd displacementToEigenVector(Eigen::Displacementd& disp);
        Eigen::VectorXd quaternionToEigenVector(Eigen::Rotation3d& quat);

        bool eigenVectorToStdVector(const Eigen::VectorXd& dispVec, std::vector<double>& doubleVec);
        bool eigenMatrixToStdVector(const Eigen::MatrixXd& dispMat, std::vector<double>& doubleVec);
        bool eigenVectorToDisplacement(const Eigen::VectorXd& dispVec, Eigen::Displacementd& disp);
        bool eigenVectorToQuaternion(const Eigen::VectorXd& quatVec, Eigen::Rotation3d& quat);
        bool eigenVectorToTwist(const Eigen::VectorXd& twistVec, Eigen::Twistd& twist);

        // bool dumpToFile(const Eigen::MatrixXd& desiredVals);

    protected:

        virtual void initializeTrajectory(){/*do nothing unless overloaded in derived classes.*/};

        double maximumVelocity;

        //variables
        Eigen::MatrixXd waypoints;          /**< the trajectory waypoints */
        int nDoF;                           /**< the number of Degrees of Freedom (DoF) of the trajectory */
        int nWaypoints;                     /**< the total number of waypoints */
        bool endsWithQuaternion;            /**< weather or not there is a quaternion component of the trajectory - this is needed for interpolation considerations */
        bool startTrigger;                  /**< used for zeroing the trajectory time */
        int currentWaypointIndex;           /**< used for keeping track of waypoints during execution */
        int nonRotationDof;                 /**< the number of DoF which are not part of the quaternion */


        Eigen::VectorXd pointToPointDurationVector; /**< the estimated durations between points */
        double pointToPointDuration;        /**< the total duration between the current two waypoints */

        bool trajectoryFinished;
        // double t0;

};





} // end of namespace ocra
#endif // TRAJECTORY_H
