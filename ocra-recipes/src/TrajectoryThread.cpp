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

TrajectoryThread::TrajectoryThread( int period,
                                    const std::string& taskName,
                                    const std::list<Eigen::VectorXd>& waypoints,
                                    const TRAJECTORY_TYPE trajectoryType,
                                    const TERMINATION_STRATEGY _terminationStrategy):
RateThread(period),
userWaypointList(waypoints),
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
    returningHome = false;

    isTaskCurrentlyActive = task->isActivated();
    weightDimension = task->getTaskDimension();
    taskType = task->getTaskType();


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
        case TIME_OPTIMAL:
            #if USING_GTTRAJ
            trajectory = std::make_shared<ocra::TimeOptimalTrajectory>();
            #else
            std::cout << "You need the GTTraj libs to use TIME_OPTIMAL type trajectories. I'm gonna make you a MIN_JERK instead." << std::endl;
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
        if (trajType==TIME_OPTIMAL) {
            return setTrajectoryWaypoints(userWaypointList);
        } else {
            return setTrajectoryWaypoints(userWaypoints);
        }
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
                    if (trajType==TIME_OPTIMAL) {
                        setTrajectoryWaypoints(allWaypointList, true);
                    } else {
                        setTrajectoryWaypoints(allWaypoints, true);
                    }
                    break;

                case REVERSE:
                    if (returningHome) {
                        if (printWaitingNoticeOnce) {
                            std::cout << "Trajectory thread for task: " << task->getTaskName() << " has attained its goal state. Awaiting new commands." << std::endl;
                            printWaitingNoticeOnce = false;
                        }
                    } else {
                        flipWaypoints();
                        if (trajType==TIME_OPTIMAL) {
                            setTrajectoryWaypoints(allWaypointList, true);
                        } else {
                            setTrajectoryWaypoints(allWaypoints, true);
                        }
                        returningHome = true;
                    }
                    break;

                case REVERSE_STOP:
                    if (returningHome) {
                        stop();
                    } else {
                        flipWaypoints();
                        if (trajType==TIME_OPTIMAL) {
                            setTrajectoryWaypoints(allWaypointList, true);
                        } else {
                            setTrajectoryWaypoints(allWaypoints, true);
                        }
                        returningHome = true;
                    }
                    break;

                case REVERSE_STOP_DEACTIVATE:
                    if (returningHome) {
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
                    } else {
                        flipWaypoints();
                        if (trajType==TIME_OPTIMAL) {
                            setTrajectoryWaypoints(allWaypointList, true);
                        } else {
                            setTrajectoryWaypoints(allWaypoints, true);
                        }
                        returningHome = true;
                    }
                    break;

                case CYCLE:
                    cycleWaypoints();
                    if (trajType==TIME_OPTIMAL) {
                        setTrajectoryWaypoints(allWaypointList, true);
                    } else {
                        setTrajectoryWaypoints(allWaypoints, true);
                    }
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
            Eigen::MatrixXd desiredState_tmp;

            if (trajType==GAUSSIAN_PROCESS)
            {
                // trajectory->getDesiredValues(relativeTime, desiredState_tmp, desiredVariance);
                // desiredState << desiredState_tmp;


                // for(int i=0; i<desiredState.size(); i++)
                // {
                //     desStateBottle.addDouble(desiredState(i));
                // }
                // #if USING_SMLT
                // if(useVarianceModulation)
                // {
                //     Eigen::VectorXd desiredWeights = varianceToWeights(desiredVariance);
                //     for(int i=0; i<desiredWeights.size(); i++)
                //     {
                //         desStateBottle.addDouble(desiredWeights(i));
                //     }
                // }
                // #endif
            }
            else
            {
                // desiredState << trajectory->getDesiredValues(relativeTime);
                // for(int i=0; i<desiredState.size(); i++)
                // {
                //     desStateBottle.addDouble(desiredState(i));
                // }
                desiredState_tmp = trajectory->getDesiredValues(relativeTime);
            }


            // task->sendDesiredStateAsBottle(desStateBottle);
            // ocra::TaskState state = matrixToTaskState(desiredState_tmp);
            task->setDesiredTaskStateDirect(matrixToTaskState(desiredState_tmp));
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
    return (goalStateVector - getCurrentTaskStateAsVector()).norm() <= errorThreshold;
}

void TrajectoryThread::flipWaypoints()
{
    if (trajType==TIME_OPTIMAL) {
        std::list<Eigen::VectorXd> wptListCopy = allWaypointList;
        allWaypointList.clear();
        for (auto rit = wptListCopy.rbegin(); rit!= wptListCopy.rend(); ++rit){
            allWaypointList.push_back(*rit);
        }
        goalStateVector = *allWaypointList.rbegin();
    } else {
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
}

void TrajectoryThread::cycleWaypoints()
{
    if (trajType==TIME_OPTIMAL) {
        allWaypointList.push_front(*allWaypointList.rbegin());
        allWaypointList.pop_back();
        goalStateVector = *allWaypointList.rbegin();
    } else {
        int nCols = allWaypoints.cols();
        Eigen::MatrixXd tmp(allWaypoints.rows(), nCols);

        tmp.col(0) = allWaypoints.rightCols(1);
        tmp.rightCols(nCols-1) = allWaypoints.leftCols(nCols-1);
        allWaypoints = tmp;
        goalStateVector = allWaypoints.rightCols(1);
    }
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
            allWaypoints.col(0) << getCurrentTaskStateAsVector();
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
            maximumVariance = std::dynamic_pointer_cast<ocra::GaussianProcessTrajectory>(trajectory)->getMaxVariance();
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

bool TrajectoryThread::setTrajectoryWaypoints(const std::list<Eigen::VectorXd>& waypointList, bool containsStartingWaypoint)
{
    if (weightDimension==waypointList.begin()->rows())
    {

        allWaypointList = waypointList;
        if(!containsStartingWaypoint)
        {
            allWaypointList.push_front(getCurrentTaskStateAsVector());
        }

        goalStateVector = *allWaypointList.rbegin();


        trajectory->setWaypoints(allWaypointList);

        printWaitingNoticeOnce=true;
        deactivationLatch = false;
        waypointsHaveBeenSet = true;
        return true;
    }
    else
    {
        std::cout << "[ERROR](TrajectoryThread::setTrajectoryWaypoints): The dimension (# DOF) of the waypoints you provided, " << waypointList.begin()->rows() << ", does not match the dimension of the task, " << weightDimension <<". Thread not starting." << std::endl;
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
        startStateVector = getCurrentTaskStateAsVector();
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

ocra::TaskState TrajectoryThread::matrixToTaskState(const Eigen::MatrixXd& desMat)
{
    ocra::TaskState desState;
    switch (taskType) {
        case ocra::Task::META_TASK_TYPE::UNKNOWN:
        {
            // do nothing
        }break;
        case ocra::Task::META_TASK_TYPE::POSITION:
        {
            desState.setPosition(ocra::util::eigenVectorToDisplacementd(desMat.col(POS_COL)));
            desState.setVelocity(ocra::util::eigenVectorToTwistd(desMat.col(VEL_COL)));
            desState.setAcceleration(ocra::util::eigenVectorToTwistd(desMat.col(ACC_COL)));
        }break;
        case ocra::Task::META_TASK_TYPE::ORIENTATION:
        {
            desState.setPosition(ocra::util::eigenVectorToDisplacementd(desMat.col(POS_COL)));
            desState.setVelocity(ocra::util::eigenVectorToTwistd(desMat.col(VEL_COL)));
            desState.setAcceleration(ocra::util::eigenVectorToTwistd(desMat.col(ACC_COL)));
        }break;
        case ocra::Task::META_TASK_TYPE::POSE:
        {
            desState.setPosition(ocra::util::eigenVectorToDisplacementd(desMat.col(POS_COL)));
            desState.setVelocity(ocra::util::eigenVectorToTwistd(desMat.col(VEL_COL)));
            desState.setAcceleration(ocra::util::eigenVectorToTwistd(desMat.col(ACC_COL)));
        }break;
        case ocra::Task::META_TASK_TYPE::FORCE:
        {
            desState.setWrench(ocra::util::eigenVectorToWrenchd(desMat.col(POS_COL)));
        }break;
        case ocra::Task::META_TASK_TYPE::COM:
        {
            desState.setPosition(ocra::util::eigenVectorToDisplacementd(desMat.col(POS_COL)));
            desState.setVelocity(ocra::util::eigenVectorToTwistd(desMat.col(VEL_COL)));
            desState.setAcceleration(ocra::util::eigenVectorToTwistd(desMat.col(ACC_COL)));
        }break;
        case ocra::Task::META_TASK_TYPE::COM_MOMENTUM:
        {
            desState.setPosition(ocra::util::eigenVectorToDisplacementd(desMat.col(POS_COL)));
            desState.setVelocity(ocra::util::eigenVectorToTwistd(desMat.col(VEL_COL)));
            desState.setAcceleration(ocra::util::eigenVectorToTwistd(desMat.col(ACC_COL)));
        }break;
        case ocra::Task::META_TASK_TYPE::PARTIAL_POSTURE:
        {
            desState.setQ(desMat.col(POS_COL));
            desState.setQd(desMat.col(VEL_COL));
            desState.setQdd(desMat.col(ACC_COL));
        }break;
        case ocra::Task::META_TASK_TYPE::FULL_POSTURE:
        {
            desState.setQ(desMat.col(POS_COL));
            desState.setQd(desMat.col(VEL_COL));
            desState.setQdd(desMat.col(ACC_COL));
        }break;
        case ocra::Task::META_TASK_TYPE::PARTIAL_TORQUE:
        {
            desState.setTorque(desMat.col(POS_COL));
        }break;
        case ocra::Task::META_TASK_TYPE::FULL_TORQUE:
        {
            desState.setTorque(desMat.col(POS_COL));
        }break;
        default:
        {

        }break;
    }

    return desState;
}

Eigen::VectorXd TrajectoryThread::getCurrentTaskStateAsVector()
{
    Eigen::VectorXd startVector = Eigen::VectorXd::Zero(weightDimension);
    ocra::TaskState state = task->getTaskState();

    switch (taskType) {
        case ocra::Task::META_TASK_TYPE::UNKNOWN:
        {
            // do nothing
        }break;
        case ocra::Task::META_TASK_TYPE::POSITION:
        {
            startVector = state.getPosition().getTranslation();
        }break;
        case ocra::Task::META_TASK_TYPE::ORIENTATION:
        {
            Eigen::Rotation3d quat = state.getPosition().getRotation();
            startVector = Eigen::VectorXd(4);
            startVector << quat.w(), quat.x(), quat.y(), quat.z();
        }break;
        case ocra::Task::META_TASK_TYPE::POSE:
        {
            Eigen::Displacementd disp = state.getPosition();
            startVector = Eigen::VectorXd(7);
            startVector << disp.x(), disp.y(), disp.z(), disp.qw(), disp.qx(), disp.qy(), disp.qz();
        }break;
        case ocra::Task::META_TASK_TYPE::FORCE:
        {
            startVector = state.getWrench();
        }break;
        case ocra::Task::META_TASK_TYPE::COM:
        {
            startVector = state.getPosition().getTranslation();

        }break;
        case ocra::Task::META_TASK_TYPE::COM_MOMENTUM:
        {
            startVector = state.getPosition().getTranslation();

        }break;
        case ocra::Task::META_TASK_TYPE::PARTIAL_POSTURE:
        {
            startVector = state.getQ();

        }break;
        case ocra::Task::META_TASK_TYPE::FULL_POSTURE:
        {
            startVector = state.getQ();

        }break;
        case ocra::Task::META_TASK_TYPE::PARTIAL_TORQUE:
        {
            startVector = state.getTorque();

        }break;
        case ocra::Task::META_TASK_TYPE::FULL_TORQUE:
        {
            startVector = state.getTorque();

        }break;
        default:
        {

        }break;
    }
    return startVector;
}

#if USING_SMLT
void TrajectoryThread::setMeanWaypoints(std::vector<bool>& isMeanWaypoint)
{
    std::dynamic_pointer_cast<ocra::GaussianProcessTrajectory>(trajectory)->setMeanWaypoints(isMeanWaypoint);
}

void TrajectoryThread::setVarianceWaypoints(std::vector<bool>& isVarWaypoint)
{
    std::dynamic_pointer_cast<ocra::GaussianProcessTrajectory>(trajectory)->setVarianceWaypoints(isVarWaypoint);
}

void TrajectoryThread::setOptimizationWaypoints(std::vector<bool>& isOptWaypoint)
{
    std::dynamic_pointer_cast<ocra::GaussianProcessTrajectory>(trajectory)->setOptimizationWaypoints(isOptWaypoint);
}

void TrajectoryThread::setDofToOptimize(std::vector<Eigen::VectorXi>& dofToOptimize)
{
    std::dynamic_pointer_cast<ocra::GaussianProcessTrajectory>(trajectory)->setDofToOptimize(dofToOptimize);
}

Eigen::VectorXd TrajectoryThread::getBayesianOptimizationVariables()
{
    return std::dynamic_pointer_cast<ocra::GaussianProcessTrajectory>(trajectory)->getBoptVariables();
}
#endif
