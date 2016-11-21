#ifndef GAUSSIANPROCESS_H
#define GAUSSIANPROCESS_H

#include "ocra/control/Trajectory/Trajectory.h"
#include <math.h>
#include "smlt/gaussianProcess.hpp"
#include "smlt/bayesianOptimization.hpp"
#include "smlt/smltUtilities.hpp"
#include <iomanip>


namespace ocra
{

    typedef std::vector<bool> boolVector;


class GaussianProcessTrajectory : public Trajectory
{
DEFINE_CLASS_POINTER_TYPEDEFS(GaussianProcessTrajectory)

    public:

        Eigen::MatrixXd getDesiredValues(double time);
        Eigen::VectorXd getVariance(double time);
        void getDesiredValues(double time, Eigen::MatrixXd& desiredValues, Eigen::VectorXd& variance);

        void addWaypoint(const Eigen::VectorXd newWaypoint, const double waypointTime);

        void addWaypoint(const Eigen::VectorXd newWaypoint, const double waypointTime, const Eigen::VectorXi& dofToOpt, const bool useForMean=true, const bool useForVar=true, const bool useForOpt=true);
        void removeWaypoint(int index);

        bool setMeanWaypoints(boolVector& bMeanVec);
        bool setVarianceWaypoints(boolVector& bVarVec);
        bool setOptimizationWaypoints(boolVector& bOptVec);
        bool setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptVec);

        void printWaypointData();

        Eigen::MatrixXd getWaypointData();
        Eigen::MatrixXd getMeanGPData();
        Eigen::MatrixXd getVarGPData();


        Eigen::VectorXd getBoptVariables(const int extraPointsToAdd=0, std::vector<Eigen::VectorXi> dofToOptVec = std::vector<Eigen::VectorXi>() );
        Eigen::MatrixXd getBoptCovarianceMatrix();
        Eigen::VectorXd getBoptSearchSpaceMinBound();
        Eigen::VectorXd getBoptSearchSpaceMaxBound();
        // smlt::bopt_Parameters getBoptParameters();

        bool setBoptVariables(const Eigen::VectorXd& newOptVariables);


        void saveTrajectoryToFile(const std::string dirPath = "./");
        void saveWaypointDataToFile(const std::string dirPath = "./");

        // Simple getters/setters
        double getMaxVariance(){return maxCovariance.maxCoeff();}
        Eigen::VectorXd getMaxCovarianceVector(){return maxCovariance;}
        double getVarianceLengthParameter(){return varLengthParameter;}
        double getMeanTime(){return meanKernelCenters.mean();}
        double getDuration(){return meanKernelCenters.maxCoeff();}

    protected:
        virtual void initializeTrajectory();


    private:
        smlt::gaussianProcess* meanGP;
        smlt::gaussianProcess* varianceGP;

        // Eigen::MatrixXd meanKernelCenters;
        // Eigen::MatrixXd varKernelCenters;

        Eigen::VectorXd meanKernelCenters;
        Eigen::VectorXd varKernelCenters;

        Eigen::MatrixXd meanKernelTrainingData;
        Eigen::MatrixXd varKernelTrainingData;

        Eigen::MatrixXd originalWaypoints;
        Eigen::VectorXd timeline;


        bool gpParametersAreSet();
        void calculateGaussianProcessParameters();

        void precalculateTrajectory(Eigen::MatrixXd& traj, Eigen::MatrixXd& variance, Eigen::VectorXd& timeline, const double DT=0.01);


        Eigen::VectorXd maxCovariance;
        double meanLengthParameter;
        double varLengthParameter;

        Eigen::VectorXd posOld;
        Eigen::VectorXd velOld;
        double t_old;
        double t0, t0_variance;
        bool varianceStartTrigger;


        double extraWpDt;

        int numberOfOptimizationWaypoints;
        bool boptVariablesSet;


        int extraWaypoints;


        boolVector isMeanWaypoint, isVarWaypoint, isOptWaypoint;
        std::vector<Eigen::VectorXi> dofToOptimize;
};






} // end of namespace ocra
#endif // GAUSSIANPROCESS_H
