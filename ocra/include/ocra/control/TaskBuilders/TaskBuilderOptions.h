/**
 *  \class TaskBuilderOptions
 *
 *  \brief This class is used to store the "options" that characterize a particular. Examples of these options are Task name, Task type, Segment, kp, kd, weight, weights, axes, hierarchyLevel, offset, jointIndexes and desired. More detailed descriptions are found in the class TaskConstructionManager. For a few examples, please refer to ocra-wbi-plugins/ocra-icub-server/app/robots/icubGazeboSim/taskSets/.
 *
 *
 *  \author [Ryan Lober](https://github.com/rlober)
 *
 */

#ifndef TASK_BUILDER_OPTIONS_H
#define TASK_BUILDER_OPTIONS_H

#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include <ocra/util/Macros.h>
#include <ocra/util/YarpUtilities.h>
#include <ocra/control/ControlEnum.h>
#include <yarp/os/Bottle.h>

namespace ocra
{

class TaskBuilderOptions {
DEFINE_CLASS_POINTER_TYPEDEFS(TaskBuilderOptions)

public:
    TaskBuilderOptions();
    ~TaskBuilderOptions();
    void putIntoBottle(yarp::os::Bottle& bottle);
    bool extractFromBottle(yarp::os::Bottle& bottle, int& sizeOfOptions);

    std::string taskName, taskType, segment;
    double kp, kd, weight, mu, margin;
    bool usesYarp, useWeightVectorConstructor;
    ECartesianDof axes;
    int hierarchyLevel;
    Eigen::VectorXd desired;
    Eigen::VectorXd indexDesired;
    Eigen::VectorXd nameDesired;
    Eigen::VectorXd weightVector;
    Eigen::VectorXd indexWeightVector;
    Eigen::VectorXd nameWeightVector;
    Eigen::VectorXi jointIndexes;
    std::vector<std::string> jointNames;
    Eigen::Displacementd offset;


    friend std::ostream& operator<<(std::ostream &out, const TaskBuilderOptions& tmOpts)
    {
        out << "Task name:\n" << tmOpts.taskName << std::endl << std::endl;
        out << "Task type:\n" << tmOpts.taskType << std::endl << std::endl;
        out << "Segment:\n" << tmOpts.segment << std::endl << std::endl;
        out << "kp:\n" << tmOpts.kp << std::endl << std::endl;
        out << "kd:\n" << tmOpts.kd << std::endl << std::endl;
        out << "weight:\n" << tmOpts.weight << std::endl << std::endl;
        out << "weights:\n" << tmOpts.weightVector.transpose() << std::endl << std::endl;
        out << "axes:\n" << utils::cartesianDofToString(tmOpts.axes) << std::endl << std::endl;
        out << "hierarchyLevel:\n" << tmOpts.hierarchyLevel << std::endl << std::endl;
        out << "offset:\n" << tmOpts.offset << std::endl;
        out << "jointIndexes:\n" << tmOpts.jointIndexes.transpose() << std::endl << std::endl;
        out << "desired:\n" << tmOpts.desired.transpose() << std::endl << std::endl;
        return out;
    }

private:

    const static int TASK_BUILDER_OPTIONS_BOTTLE = 123;

};

} // namespace ocra
#endif //TASK_BUILDER_OPTIONS_H
