/*! \file       TrajectoryThread.h
 *  \brief      A thread for launching trajectory generators.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-recipes.
 *  Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TRAJECTORY_THREAD_H
#define TRAJECTORY_THREAD_H

#include <ocra-recipes/ControlThread.h>
#include "ocra/control/Trajectory/Trajectories.h"
#include <yarp/os/Time.h>
#include <iostream>

namespace ocra_recipes
{
enum TRAJECTORY_TYPE
{
    MIN_JERK,
    LIN_INTERP,
    GAUSSIAN_PROCESS
};

enum TERMINATION_STRATEGY
{
    BACK_AND_FORTH,
    STOP_THREAD,
    WAIT,
    STOP_THREAD_DEACTIVATE,
    WAIT_DEACTIVATE
};

class TrajectoryThread : public ControlThread
{

public:
    // TrajectoryThread();
    TrajectoryThread(int period, const std::string& taskPortName, const TRAJECTORY_TYPE = MIN_JERK, const TERMINATION_STRATEGY _terminationStrategy = STOP_THREAD);
    TrajectoryThread(int period, const std::string& taskPortName, const Eigen::MatrixXd& waypoints, const TRAJECTORY_TYPE = MIN_JERK, const TERMINATION_STRATEGY _terminationStrategy = STOP_THREAD);
    ~TrajectoryThread();

    virtual bool ct_threadInit();
    virtual void ct_threadRelease();
    virtual void ct_run();
    virtual std::string getThreadType(){return "TrajectoryThread";}


    // Setters
    void setMaxVelocity(double maxVel){trajectory->setMaxVelocity(maxVel);}

    bool setDisplacement(double dispDouble);
    bool setDisplacement(const Eigen::VectorXd& displacementVector);
    bool setTrajectoryWaypoints(const Eigen::MatrixXd& userWaypoints, bool containsStartingWaypoint=false);
    void setTerminationStrategy(const TERMINATION_STRATEGY newTermStrat){terminationStrategy = newTermStrat;}
    void setGoalErrorThreshold(const double newErrorThresh){errorThreshold = newErrorThresh;}
    void setUseVarianceModulation(bool newVarMod){useVarianceModulation = newVarMod;}
    // Getters

    // General assesment functions
    bool goalAttained();

    #if USING_SMLT
    // Setters
    void setMeanWaypoints(std::vector<bool>& isMeanWaypoint);
    void setVarianceWaypoints(std::vector<bool>& isVarWaypoint);
    void setOptimizationWaypoints(std::vector<bool>& isOptWaypoint);
    void setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptimize);

    // Getters
    Eigen::VectorXd getBayesianOptimizationVariables();
    #endif


protected:
    void init();

    bool waypointsHaveBeenSet;
    Eigen::VectorXd varianceToWeights(Eigen::VectorXd& desiredVariance, const double beta = 1.0);
    // void getTaskWeightDimension();
    void flipWaypoints();

    Eigen::MatrixXd userWaypoints;
    TRAJECTORY_TYPE trajType;
    TERMINATION_STRATEGY terminationStrategy;

    ocra::Trajectory* trajectory;


    double maximumVariance;
    bool useVarianceModulation;
    Eigen::VectorXd desiredVariance;
    Eigen::ArrayXd varianceThresh;

    Eigen::VectorXd startStateVector;
    Eigen::VectorXd goalStateVector;
    Eigen::MatrixXd allWaypoints;
    double errorThreshold;

    Eigen::VectorXd desiredState;
    yarp::os::Bottle desStateBottle;

    bool printWaitingNoticeOnce;

    double deactivationDelay;
    double deactivationTimeout;
    bool deactivationLatch;

};
} // namespace ocra_recipes
#endif // TRAJECTORY_THREAD_H
