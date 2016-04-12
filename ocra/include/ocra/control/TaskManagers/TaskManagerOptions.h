#ifndef TASK_MANAGER_OPTIONS_H
#define TASK_MANAGER_OPTIONS_H

#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <yarp/os/Bottle.h>
#include "ocra/control/TaskManagers/TaskManagerMessageVocab.h"


namespace ocra
{

class TaskManagerOptions
{
public:
    TaskManagerOptions();
    ~TaskManagerOptions();
    void putIntoBottle(yarp::os::Bottle& bottle);
    bool extractFromBottle(yarp::os::Bottle& bottle, int& sizeOfOptions);

    std::string taskName, taskType, segment;
    double kp, kd, weight, mu, margin;
    bool usesYarp, useWeightVectorConstructor;
    int axes;
    int hierarchyLevel;
    Eigen::VectorXd desired;
    Eigen::VectorXd indexDesired;
    Eigen::VectorXd nameDesired;
    Eigen::VectorXd weightVector;
    Eigen::VectorXd indexWeightVector;
    Eigen::VectorXd nameWeightVector;
    Eigen::VectorXi jointIndexes;
    std::vector<std::string> jointNames;
    std::vector<Eigen::VectorXd> offset;


    friend std::ostream& operator<<(std::ostream &out, const TaskManagerOptions& tmOpts)
    {
        out << "Task name:\n" << tmOpts.taskName << std::endl << std::endl;
        out << "Task type:\n" << tmOpts.taskType << std::endl << std::endl;
        out << "Segment:\n" << tmOpts.segment << std::endl << std::endl;
        out << "kp:\n" << tmOpts.kp << std::endl << std::endl;
        out << "kd:\n" << tmOpts.kd << std::endl << std::endl;
        out << "weight:\n" << tmOpts.weight << std::endl << std::endl;
        out << "weights:\n" << tmOpts.weightVector.transpose() << std::endl << std::endl;
        out << "axes:\n" << tmOpts.axes << std::endl << std::endl;
        out << "hierarchyLevel:\n" << tmOpts.hierarchyLevel << std::endl << std::endl;
        out << "offset:\n";
        for(int j=0; j<tmOpts.offset.size(); j++) {
            out << tmOpts.offset[j].transpose() << std::endl;
        }
        out << std::endl;
        out << "jointIndexes:\n" << tmOpts.jointIndexes.transpose() << std::endl << std::endl;
        out << "desired:\n" << tmOpts.desired.transpose() << std::endl << std::endl;
        return out;
    }

private:

    const static int TASK_MANAGER_OPTIONS_BOTTLE = 123;

};

} // namespace ocra
#endif //TASK_MANAGER_OPTIONS_H
