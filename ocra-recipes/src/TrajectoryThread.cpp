/*! \file       TrajectoryThread.cpp
 *  \brief      A thread for launching trajectory generators.
 *  \details
 *  \author     [Ryan Lober](http://www.ryanlober.com)
 *  \author     [Antoine Hoarau](http://ahoarau.github.io)
 *  \date       Feb 2016
 *  \copyright  GNU General Public License.
 */
/*
 *  This file is part of ocra-icub.
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

#include <ocra-recipes/TrajectoryThread.h>

#ifndef VAR_THRESH
#define VAR_THRESH 0.99
#endif

using namespace ocra_recipes;

// TrajectoryThread::TrajectoryThread()
// {
//
// }

TrajectoryThread::~TrajectoryThread()
{

}

TrajectoryThread::TrajectoryThread( int period,
                                    const std::string& taskName,
                                    const TRAJECTORY_TYPE trajectoryType,
                                    const TERMINATION_STRATEGY _terminationStrategy)
: RateThread(period)
, trajType(trajectoryType)
, terminationStrategy(_terminationStrategy)
, waypointsHaveBeenSet(false)
{
    task = std::make_shared<TaskConnection>(taskName);
    task->openControlPorts();
    init();
}

TrajectoryThread::TrajectoryThread( int period,
                                    const std::string& taskName,
                                    const Eigen::MatrixXd& waypoints,
                                    const TRAJECTORY_TYPE trajectoryType,
                                    const TERMINATION_STRATEGY _terminationStrategy):
RateThread(period),
userWaypoints(waypoints),
trajType(trajectoryType),
terminationStrategy(_terminationStrategy),
waypointsHaveBeenSet(true)
{
    task = std::make_shared<TaskConnection>(taskName);
    task->openControlPorts();
    init();
}

void TrajectoryThread::init()
{
    // Set up class variables:
    printWaitingNoticeOnce = true;
    errorThreshold = 0.03;
    useVarianceModulation = true;
    deactivationDelay = 0.0;
    deactivationTimeout = 5.0;
    deactivationLatch = false;
    isPaused = false;
    timeElapsedDuringPause = 0.0;

    isTaskCurrentlyActive = task->isActivated();
    weightDimension = task->getTaskDimension();


    switch (trajType)
    {
        case MIN_JERK:
            trajectory = std::make_shared<ocra::MinimumJerkTrajectory>();
            break;
        case LIN_INTERP:
            trajectory = std::make_shared<ocra::LinearInterpolationTrajectory>();
            break;
        case GAUSSIAN_PROCESS:
            #if USING_SMLT
            trajectory = std::make_shared<ocra::GaussianProcessTrajectory>();
            #else
            std::cout << "You need the SMLT libs to use GAUSSIAN_PROCESS type trajectories. I'm gonna make you a MIN_JERK instead." << std::endl;
            trajType = MIN_JERK;
            trajectory = std::make_shared<ocra::MinimumJerkTrajectory>();
            #endif
            break;
    }
}

bool TrajectoryThread::threadInit()
{
    if (trajType==GAUSSIAN_PROCESS)
    {
        desiredVariance = Eigen::VectorXd::Ones(weightDimension);
        varianceThresh = Eigen::ArrayXd::Constant(weightDimension, VAR_THRESH);
    }

    if (waypointsHaveBeenSet) {
        return setTrajectoryWaypoints(userWaypoints);
    }
    else {
        return true;
    }
}

void TrajectoryThread::threadRelease()
{
    task->closeControlPorts();
    std::cout<< "\nTrajectoryThread: Trajectory thread finished.\n";
}

void TrajectoryThread::run()
{
    if (waypointsHaveBeenSet && !isPaused) {
        if (goalAttained() || deactivationLatch)
        {
            switch (terminationStrategy)
            {
                case BACK_AND_FORTH:
                    flipWaypoints();
                    setTrajectoryWaypoints(allWaypoints, true);
                    break;

                case CYCLE:
                    cycleWaypoints();
                    setTrajectoryWaypoints(allWaypoints, true);
                    break;

                case STOP_THREAD:
                    stop();
                    break;
                case STOP_THREAD_DEACTIVATE:
                    if(task->deactivate()){
                        isTaskCurrentlyActive = task->isActivated();
                        stop();
                    }else{
                        std::cout << "[WARNING] Trajectory thread for task: " << task->getTaskName() << " has attained its goal state, but cannot be deactivated." << std::endl;
                        yarp::os::Time::delay(1.0); // try again in one second.
                        deactivationDelay += 1.0;
                        if(deactivationDelay >= deactivationTimeout){
                            std::cout << "[WARNING] Deactivation timeout." << std::endl;
                            stop();
                        }
                    }
                    break;
                case WAIT:
                    if (printWaitingNoticeOnce) {
                        std::cout << "Trajectory thread for task: " << task->getTaskName() << " has attained its goal state. Awaiting new commands." << std::endl;
                        printWaitingNoticeOnce = false;
                    }
                    break;
                case WAIT_DEACTIVATE:
                    if (printWaitingNoticeOnce) {
                        if(task->deactivate()){
                            isTaskCurrentlyActive = task->isActivated();

                            std::cout << "Trajectory thread for task: " << task->getTaskName() << " has attained its goal state. Deactivating task and awaiting new commands." << std::endl;
                            printWaitingNoticeOnce = false;
                            deactivationLatch = true;
                        }else{
                            std::cout << "Trajectory thread for task: " << task->getTaskName() << " has attained its goal state and is awaiting new commands. [WARNING] Could not deactivate the task." << std::endl;
                            yarp::os::Time::delay(1.0); // try again in one second.
                            deactivationDelay += 1.0;
                            if(deactivationDelay >= deactivationTimeout){
                                printWaitingNoticeOnce = false;
                                std::cout << "[WARNING] Deactivation timeout." << std::endl;
                            }
                        }
                    }
                    break;
            }
        }
        else{
            if (!isTaskCurrentlyActive) {
                task->activate();
                isTaskCurrentlyActive = task->isActivated();
            }

            desStateBottle.clear();

            double relativeTime = yarp::os::Time::now() - timeElapsedDuringPause;

            if (trajType==GAUSSIAN_PROCESS)
            {
                Eigen::MatrixXd desiredState_tmp;
                trajectory->getDesiredValues(relativeTime, desiredState_tmp, desiredVariance);
                desiredState << desiredState_tmp;


                for(int i=0; i<desiredState.size(); i++)
                {
                    desStateBottle.addDouble(desiredState(i));
                }
                #if USING_SMLT
                if(useVarianceModulation)
                {
                    Eigen::VectorXd desiredWeights = varianceToWeights(desiredVariance);
                    for(int i=0; i<desiredWeights.size(); i++)
                    {
                        desStateBottle.addDouble(desiredWeights(i));
                    }
                }
                #endif
            }
            else
            {
                desiredState << trajectory->getDesiredValues(relativeTime);
                for(int i=0; i<desiredState.size(); i++)
                {
                    desStateBottle.addDouble(desiredState(i));
                }
            }


            task->sendDesiredStateAsBottle(desStateBottle);
        }
    }
}

void TrajectoryThread::pause()
{
    isPaused = true;
    pauseTime = yarp::os::Time::now();
}

void TrajectoryThread::unpause()
{
    isPaused = false;
    timeElapsedDuringPause = yarp::os::Time::now() - pauseTime;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd TrajectoryThread::varianceToWeights(Eigen::VectorXd& desiredVariance, const double beta)
{
    desiredVariance /= maximumVariance;
    desiredVariance = desiredVariance.array().min(varianceThresh); //limit desiredVariance to 0.99 maximum
    Eigen::VectorXd desiredWeights = (Eigen::VectorXd::Ones(desiredVariance.rows()) - desiredVariance) / beta;
    return desiredWeights;
}

bool TrajectoryThread::goalAttained()
{
    return (goalStateVector - task->getCurrentState().head(weightDimension)).norm() <= errorThreshold;
}

void TrajectoryThread::flipWaypoints()
{
    int nCols = allWaypoints.cols();
    Eigen::MatrixXd tmp(allWaypoints.rows(), nCols);

    int c_tmp = nCols - 1;
    for(int c=0; c<nCols; c++)
    {
        tmp.col(c_tmp) = allWaypoints.col(c);
        c_tmp--;
    }
    allWaypoints = tmp;
    goalStateVector = allWaypoints.rightCols(1);
}

void TrajectoryThread::cycleWaypoints()
{
    int nCols = allWaypoints.cols();
    Eigen::MatrixXd tmp(allWaypoints.rows(), nCols);

    tmp.col(0) = allWaypoints.rightCols(1);
    tmp.rightCols(nCols-1) = allWaypoints.leftCols(nCols-1);
    allWaypoints = tmp;
    goalStateVector = allWaypoints.rightCols(1);
}

bool TrajectoryThread::setTrajectoryWaypoints(const Eigen::MatrixXd& userWaypoints, bool containsStartingWaypoint)
{
    Eigen::MatrixXd _userWaypoints = userWaypoints; // Copy waypoints first in case we use allWaypoints as an arg.
    if (weightDimension==_userWaypoints.rows())
    {
        if(containsStartingWaypoint)
        {
            allWaypoints = _userWaypoints;
        }
        else // Add starting waypoint
        {
            allWaypoints = Eigen::MatrixXd(weightDimension, _userWaypoints.cols()+1);

            startStateVector = task->getCurrentStateRpc();

            desiredState = Eigen::VectorXd::Zero(startStateVector.size());

            allWaypoints.col(0) << task->getCurrentState().head(weightDimension);
            for(int i=0; i<_userWaypoints.cols(); i++)
            {
                allWaypoints.col(i+1) << _userWaypoints.col(i);
            }
        }

        goalStateVector = allWaypoints.rightCols(1);

        trajectory->setWaypoints(allWaypoints);

        #if USING_SMLT
        if (trajType==GAUSSIAN_PROCESS)
        {
            maximumVariance = dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->getMaxVariance();
        }
        #endif

        printWaitingNoticeOnce=true;
        deactivationLatch = false;
        waypointsHaveBeenSet = true;
        return true;
    }
    else
    {
        std::cout << "[ERROR](TrajectoryThread::setTrajectoryWaypoints): The dimension (# DOF) of the waypoints you provided, " << _userWaypoints.rows() << ", does not match the dimension of the task, " << weightDimension <<". Thread not starting." << std::endl;
        return false;
    }
}


bool TrajectoryThread::setDisplacement(double dispDouble)
{
    return setDisplacement(Eigen::VectorXd::Constant(weightDimension, dispDouble));
}

bool TrajectoryThread::setDisplacement(const Eigen::VectorXd& displacementVector)
{
    if (weightDimension == displacementVector.rows())
    {
        startStateVector = task->getCurrentState();
        Eigen::MatrixXd tmpWaypoints = Eigen::MatrixXd::Zero(weightDimension, 2);
        tmpWaypoints.col(0) << startStateVector;
        tmpWaypoints.col(1) << startStateVector + displacementVector;
        setTrajectoryWaypoints(tmpWaypoints, true);
        return true;
    }
    else{
        return false;
    }
}



#if USING_SMLT
void TrajectoryThread::setMeanWaypoints(std::vector<bool>& isMeanWaypoint)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setMeanWaypoints(isMeanWaypoint);
}

void TrajectoryThread::setVarianceWaypoints(std::vector<bool>& isVarWaypoint)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setVarianceWaypoints(isVarWaypoint);
}

void TrajectoryThread::setOptimizationWaypoints(std::vector<bool>& isOptWaypoint)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setOptimizationWaypoints(isOptWaypoint);
}

void TrajectoryThread::setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptimize)
{
    dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->setDofToOptimize(dofToOptimize);
}

Eigen::VectorXd TrajectoryThread::getBayesianOptimizationVariables()
{
    return dynamic_cast<ocra::GaussianProcessTrajectory*>(trajectory)->getBoptVariables();
}
#endif
