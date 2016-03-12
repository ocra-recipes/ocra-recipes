#ifndef TASK_MANAGER_OPTIONS_H
#define TASK_MANAGER_OPTIONS_H

#include <Eigen/Dense>
#include <Eigen/Lgsm>

namespace ocra
{

class TaskManagerOptions
{
public:
    TaskManagerOptions();
    ~TaskManagerOptions();

    std::string taskName, taskType, segment;
    double kp, kd, weight, mu, margin;
    bool usesYarp, useWeightVectorConstructor;
    int axes;
    Eigen::VectorXd desired;
    Eigen::VectorXd indexDesired;
    Eigen::VectorXd nameDesired;
    Eigen::VectorXd weightVector;
    Eigen::VectorXd indexWeightVector;
    Eigen::VectorXd nameWeightVector;
    Eigen::VectorXi jointIndexes;
    std::vector<std::string> jointNames;
    std::vector<Eigen::VectorXd> offset;


};

} // namespace ocra
#endif //TASK_MANAGER_OPTIONS_H
