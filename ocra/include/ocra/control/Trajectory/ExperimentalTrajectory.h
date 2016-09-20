#ifndef EXPERIMENTALTRAJECTORY_H
#define EXPERIMENTALTRAJECTORY_H

#include "ocra/control/Trajectory/Trajectory.h"



namespace ocra
{

class ExperimentalTrajectory : public Trajectory
{
DEFINE_CLASS_POINTER_TYPEDEFS(ExperimentalTrajectory)

    public:

        void calculateVarianceParameters();

        Eigen::MatrixXd getDesiredValues(double time);
        Eigen::VectorXd getVariance(double time);


        void getDesiredValues(double time, Eigen::MatrixXd& desiredValues, Eigen::VectorXd& variance);
        void getDesiredValues(double time, Eigen::Displacementd& position, Eigen::Twistd& velocity, Eigen::Twistd& acceleration);

        double getMaxVariance();

        Eigen::MatrixXd getWaypointData();

        void addNewWaypoint(Eigen::VectorXd newWaypoint, double waypointTime);
        void removeYoungestWaypoint();
        void removeWaypoint(int index);

        Eigen::MatrixXd getRbfnKernelCurves();

    protected:
        virtual void initializeTrajectory();
        double t0, t0_variance;
        bool varianceStartTrigger;

    private:
        Eigen::MatrixXd originalWaypoints;
        int youngestWaypointIndex;


        int numberOfKernels;
        Eigen::VectorXd kernelCenters;  // where the centers are placed
        Eigen::VectorXd maxCovariance;  // maximum allowable covariance (or variance for each dof)
        double kernelLengthParameter;   // influence of the kernel centers on one another

        Eigen::MatrixXd designMatrix, designMatrixInv;   // big K matrix


        void precalculateTrajectory(double DT, Eigen::MatrixXd& path, Eigen::VectorXd& timeline);
        Eigen::MatrixXd kernelFunction(double m);
        Eigen::MatrixXd kernelFunction(Eigen::VectorXd& evalVec);



        // rbfn
        Eigen::VectorXd rbfnKernelFunction(double m);
        Eigen::MatrixXd rbfnKernelFunction(Eigen::VectorXd& evalVec);
        void calculateRbfnWeights();
        Eigen::MatrixXd rbfnWeights;

        Eigen::VectorXd getRbfnOutput(double m);
        Eigen::MatrixXd getRbfnOutput(Eigen::VectorXd& evalVec);

        Eigen::VectorXd posOld;
        Eigen::VectorXd velOld;
        double t_old;


        void reinitialize();

};

} // end of namespace ocra
#endif // EXPERIMENTALTRAJECTORY_H
