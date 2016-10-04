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

#include <iostream>

#include "ocra/control/Trajectory/Trajectories.h"

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <Eigen/Dense>
#include <Eigen/Lgsm>

#include <ocra-recipes/TaskConnection.h>
#include <ocra/util/Macros.h>
#include <ocra/util/EigenUtilities.h>
#include <ocra/control/TaskState.h>


namespace ocra_recipes
{
enum TRAJECTORY_TYPE
{
    MIN_JERK,
    LIN_INTERP,
    GAUSSIAN_PROCESS,
    TIME_OPTIMAL
};

enum TERMINATION_STRATEGY
{
    BACK_AND_FORTH,
    STOP_THREAD,
    WAIT,
    STOP_THREAD_DEACTIVATE,
    WAIT_DEACTIVATE,
    CYCLE
};

class TrajectoryThread : public yarp::os::RateThread
{
DEFINE_CLASS_POINTER_TYPEDEFS(TrajectoryThread)

public:
    // TrajectoryThread();
    TrajectoryThread(int period, const std::string& taskPortName, const TRAJECTORY_TYPE = MIN_JERK, const TERMINATION_STRATEGY _terminationStrategy = STOP_THREAD);
    TrajectoryThread(int period, const std::string& taskPortName, const Eigen::MatrixXd& waypoints, const TRAJECTORY_TYPE = MIN_JERK, const TERMINATION_STRATEGY _terminationStrategy = STOP_THREAD);
    TrajectoryThread(int period, const std::string& taskPortName, const std::list<Eigen::VectorXd>& waypoints, const TRAJECTORY_TYPE = MIN_JERK, const TERMINATION_STRATEGY _terminationStrategy = STOP_THREAD);
    ~TrajectoryThread();

    /**
     *  If the TrajectoryThread object has been instantiated with defined waypoints (i.e. if waypointsHaveBeenSet is true) the waypoints will be set. Note that if an std::list of waypoints is passed to the constructor of this class, the trajectory type must be TIME_OPTIMAL.
     *
     *  @return True when initial waypoints have been set.
     */
    virtual bool threadInit();
    
    /**
     *  Closes the control ports opened by the task associated to this trajectory.
     */
    virtual void threadRelease();
    
    /**
     *  If the control ports are open (for the specific task associated to the trajectory) this method will mainly write to a port the new desired task state.
     *  It also handles the trajectory termination strategy. When the goal is reached, there will be 6 possible termination strategies: BACK_AND_FORTH, CYCLE, STOP_THREAD, STOP_THREAD_DEACTIVATE, WAIT, WAIT_DEACTIVATE.
     *   BACK_AND_FORTH: will invert the trajectory waypoints once all of them have been met.
     *   CYCLE: The trajectory will cycle over.
     *   STOP_THREAD: The thread will stop once the trajectory is finished.
     *   STOP_THREAD_DEACTIVATE: The task is first deactivated and then thread is stopped. If the task cannot be deactivated, the thread will wait 1 second before trying once more.
     *   WAIT: The thread will wait for new commands to arrive.
     *   WAIT_DEACTIVATE: It will first deactivate the task and then wait for new commands. If the task cannot be deactivated, the thread will wait 1 second before trying once more.
     */
    virtual void run();
    
    /**
     *  Sets isPaused to true of this class and registers the pausing time.
     */
    void pause();
    
    /**
     *  Sets isPaused to false and stores the elapsed time during the pause.
     */
    void unpause();


    // Setters
    void setMaxVelocity(double maxVel){trajectory->setMaxVelocity(maxVel);}

    /**
     *  Sets a displacement vector with the same constant value for each element and consistent with the weight dimension.
     *
     *  @param dispDouble Constant displacement value.
     *
     *  @return True when the new trajectory waypoints have been set, false otherwise.
     */
    bool setDisplacement(double dispDouble);
    
    /**
     *  Adds a displacement offset to the waypoints.
     *
     *  @param displacementVector Vector of displacement with the same number of rows as the weight dimension.
     *
     *  @return true when the new trajectory waypoints have been set, false otherwise.
     */
    bool setDisplacement(const Eigen::VectorXd& displacementVector);
    
    /**
     *  Sets the trajectory waypoints and if the initial waypoint is not included, it adds it by checking the current state of the task at hand. E.g., if it's a COM trajectory it will check the current 3D position of this point and add it at the beginning of the waypoints matrix.
     *
     *  @param[in]  userWaypoints            Column-wise trajectory waypoints.
     *  @param[out] containsStartingWaypoint True when the starting waypoint is included in the matrix. When false, this method will add it to the waypoints by checking the current state of the task.
     *
     *  @return True when the waypoints matrix is consistent with the task dimension and the full waypoints matrix is set. False otherwise.
     */
    bool setTrajectoryWaypoints(const Eigen::MatrixXd& userWaypoints, bool containsStartingWaypoint=false);
    
    /**
     *  Sets the trajectory waypoints and if the initial waypoint is not included, it adds it by checking the current state of the task at hand. E.g., if it's a COM trajectory it will check the current 3D position of this point and add it at the beginning of the waypoints matrix.
     *
     *  @param[in]  waypointList             List of trajectory waypoints.
     *  @param[out] containsStartingWaypoint True when the starting waypoint is included in the matrix. When false, this method will add it to the waypoints by checking the current state of the task.
     *
     *  @return True when the waypoints matrix is consistent with the task dimension and the full waypoints matrix is set. False otherwise.
     */
    bool setTrajectoryWaypoints(const std::list<Eigen::VectorXd>& waypointList, bool containsStartingWaypoint=false);
    
    /**
     *  Tells the thread what to do when the trajectory has been fulfilled. In particular sets the variable termimationStrategy of this class.
     *
     *  @param newTermStrat Options are: BACK_AND_FORTH, STOP_THREAD, WAIT, STOP_THREAD_DEACTIVE, WAIT_DEACTIVE, CYCLE
     */
    void setTerminationStrategy(const TERMINATION_STRATEGY newTermStrat){terminationStrategy = newTermStrat;}
    
    /**
     *  Stablishes the admissible error for the trajectory at hand. In particular sets the variable errorThreshold of this class.
     *
     *  @param newErrorThresh Error threshold.
     */
    void setGoalErrorThreshold(const double newErrorThresh){errorThreshold = newErrorThresh;}
    
    /**
     *  Assigns a user-given variance to the weights of the task when created the trajectory. Used only when trajectoryType is GAUSSIAN_PROCESS
     *
     *  @param newVarMod New weight variance modulation.
     */
    void setUseVarianceModulation(bool newVarMod){useVarianceModulation = newVarMod;}
    
    /**
     *  Computes the norm of the difference between the goal state vector and its current state.
     *
     *  @return True if the difference is less than the error threshold.
     */
    bool goalAttained();

    std::list<Eigen::VectorXd> getWaypointList(){return allWaypointList;}
    Eigen::MatrixXd getWaypoints(){return allWaypoints;}

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

    std::shared_ptr<TaskConnection> task;
    int weightDimension;

    void init();

    bool waypointsHaveBeenSet;
    Eigen::VectorXd varianceToWeights(Eigen::VectorXd& desiredVariance, const double beta = 1.0);
    // void getTaskWeightDimension();
    void flipWaypoints();
    void cycleWaypoints();


    Eigen::MatrixXd userWaypoints;
    std::list<Eigen::VectorXd> userWaypointList;

    TRAJECTORY_TYPE trajType;
    TERMINATION_STRATEGY terminationStrategy;

    std::shared_ptr<ocra::Trajectory> trajectory;


    double maximumVariance;
    bool useVarianceModulation;
    Eigen::VectorXd desiredVariance;
    Eigen::ArrayXd varianceThresh;

    Eigen::VectorXd startStateVector;
    Eigen::VectorXd goalStateVector;
    Eigen::MatrixXd allWaypoints;
    std::list<Eigen::VectorXd> allWaypointList;
    double errorThreshold;

    Eigen::VectorXd desiredState;
    yarp::os::Bottle desStateBottle;

    bool printWaitingNoticeOnce;

    double deactivationDelay;
    double deactivationTimeout;
    bool deactivationLatch;

    bool isTaskCurrentlyActive;
    bool isPaused;

    double timeElapsedDuringPause;
    double pauseTime;

    ocra::Task::META_TASK_TYPE taskType;


private:
    ocra::TaskState matrixToTaskState(const Eigen::MatrixXd& desMat);
    const int POS_COL = 0;
    const int VEL_COL = 1;
    const int ACC_COL = 2;

    Eigen::VectorXd getCurrentTaskStateAsVector();

};
} // namespace ocra_recipes
#endif // TRAJECTORY_THREAD_H
