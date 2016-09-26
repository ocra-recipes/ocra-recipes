/**
 *  \class TaskConstructionManager
 *
 *  \brief This class is used to parse an XML file with the description of a number of tasks. These tasks can be of type: fullposture, cartesian, com, orientation and pointcontact. For a few examples, please refer to ocra-wbi-plugins/ocra-icub-server/app/robots/icubGazeboSim/taskSets/. 
 *
 *
 *  \author [Ryan Lober](https://github.com/rlober)
 *
 */

#ifndef TASK_CONSTRUCTION_MANAGER_H
#define TASK_CONSTRUCTION_MANAGER_H

#include <iostream>
#include <tinyxml.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <boost/filesystem.hpp>

#include <ocra/control/Model.h>
#include <ocra/control/Controller.h>
#include <ocra/util/Macros.h>
#include <ocra/util/StringUtilities.h>
#include <ocra/util/EigenUtilities.h>
#include <ocra/util/XmlUtilities.h>


#include <ocra/control/TaskBuilders/TaskBuilderOptions.h>
#include <ocra/control/TaskBuilders/TaskBuilder.h>
#include <ocra/control/TaskBuilders/CartesianTaskBuilder.h>
#include <ocra/control/TaskBuilders/PoseTaskBuilder.h>
#include <ocra/control/TaskBuilders/OrientationTaskBuilder.h>
#include <ocra/control/TaskBuilders/ComTaskBuilder.h>
#include <ocra/control/TaskBuilders/FullPostureTaskBuilder.h>
#include <ocra/control/TaskBuilders/PartialPostureTaskBuilder.h>
#include <ocra/control/TaskBuilders/PointContactTaskBuilder.h>

namespace ocra {

class TaskConstructionManager {
DEFINE_CLASS_POINTER_TYPEDEFS(TaskConstructionManager)

public:
    TaskConstructionManager();

    TaskConstructionManager(Model::Ptr model, Controller::Ptr controller, std::vector<TaskBuilderOptions> optionsVector);
    
    TaskConstructionManager(Model::Ptr model, Controller::Ptr controller, const std::string& optionsXmlFilePath);
    
    virtual ~TaskConstructionManager();

    /**
     *  This method is called during the instantiation of an object of type TaskConstructionManager when the corresponding parameters are passed to the constructor. For every task specified through XML it creates a corresponding TaskBuilder object, builds the task, then tells the Controller to add this task to its stack of tasks and finally specifies whether the task is an objective or a constraint, what its level, weight, stiffness and damping. We call this, setting the "task parameters".
     *
     *  @param model         Pointer to ocra model.
     *  @param controller    Pointer to ocra controller.
     *  @param optionsVector A vector of N TaskBuilderOptions elements, where N is the number of tasks described in the taskSet XML file.
     */
    void addTasksToController(Model::Ptr model, Controller::Ptr controller, std::vector<TaskBuilderOptions> optionsVector);

    TaskBuilder::Ptr getBuilder(TaskBuilderOptions options, Model::Ptr model);

    /**
     *  Given an XML description of control tasks it first checks the XML existance and then parses each task description through TaskConstructionManager::parseTaskOptionsFromXml(TiXmlDocument* newTasksFile).
     *
     *  @param optionsXmlFilePath Full path to XML tasks description.
     *
     *  @return A vector of elements of type TaskBuilderOptions.
     *  @see TaskConstructionManager::parseTaskOptionsFromXml(TiXmlDocument* newTasksFile)
     */
    std::vector<TaskBuilderOptions> parseTaskOptionsFromXml(const std::string& optionsXmlFilePath);

private:

    /**
     *  Takes a TiXmlDocument object and parses all the information for each task originally described in the task XML. The information for each task is encapsulated in a TaskBuilderOptions object and these are all pushed into a vector.
     *
     *  @param newTasksFile Pointer to TiXmlDocument from original XML tasks description.
     *
     *  @return Vector of tasks descriptions. Each task is a TaskBuilderOptions element.
     */
    std::vector<TaskBuilderOptions> parseTaskOptionsFromXml(TiXmlDocument* newTasksFile);

    /**
     *  This is the main method from where every possible option (these are harcoded) for a particular task is then parsed.
     *
     *  @param[in]  xmlTask XML element of type TiXmlElement corresponding to a single task.
     *  @param[out] options ocra::TaskBuilderOptions object with the parameters of the specific task.
     */
    bool parseTaskXmlElement(TiXmlElement* xmlTask, TaskBuilderOptions& options);
    
    /**
     *  Checks the task's name and type attributes and parses these values. In particular it converts the task's type attribute to lower case and parses the corresponding values.
     *
     *  @param[in]  xmlTask XML Task element.
     *  @param[out] Vector of TaskBuilderOptions elements.
     *
     *  @return True if name and type attributes are found, false otherwise.
     */
    bool parseTaskNameAndType(TiXmlElement* xmlTask, TaskBuilderOptions& options);
    
    /**
     *  Parses the "params" attribute of an XML task description. These are:
     *  kp: Stiffnesss gain (double). 0.0 by default.
     *  kd: Damping gain (double). If "kp" is greater than zero, the default value is \f$2\sqrt(k_p)\f$ otherwise, it's 0.0 by default.
     *  weight: Scalar weight (double). If no weight is found, it's set to zero and the option useWeightVectorConstructor to false. When defined, the option useWeightVectorConstructor is set to false. 
     *  hierarchyLevel: Hierarchy level (Integer). When not found, this is set to -1 by default. 
     *  axes: Cartesian axes to be controlled. Because ECartesianDof is not a standard type we first get the user's string and then convert it to the proper type. If not found, its value will be ECartesianDof::XYZ, otherwise converts the input string through utils::cartesianDofFromString
     *  mu: Friction coefficient. If not found, defaults to 1.0.
     *  margin: Friction parameter margin. When not found, defaults to 0.05.
     *  usesYarp: To be deprecated.
     *
     *  @param[in]  currentElem A single parameter element from a task description.
     *  @param[out] options     Vector of TaskBuilderOptions objects.
     */
    void parseParamXmlElement(TiXmlElement* currentElem, TaskBuilderOptions& options);
    
    /**
     *  An "offset" element in a particular task makes sense when the task is a "frame-based" task. For example, a task expressed with respect to a segment (link) of the robot might need an offset from the origin reference frame of the corresponding link. In particular, this method will convert the written offset into a displacement through util::stringToVectorXd. If the displacement is given by attributes (x, y, z, qw, qx, qy, qz) corresponding to a cartesian displacement and quaternion orientation, this can also be automatically parsed through util::getDisplacementArgs.
     *
     *  @param[in]  offsetElement Extracted "offset" element in a particular XML task.
     *  @param[out] options       Vector of TaskBuilderOptions objects.
     */
    void parseOffsetXmlElement(TiXmlElement* offsetElement, TaskBuilderOptions& options);
    
    /**
     *  When specified, takes the written "desired" element and converts it to to a vector. This element can be written as a 7-dim vector, or specifying each argument with its corresponding value (x, y, z, qw, qx, qy, qz).
     *
     *  @param[in]  desiredElement Extracted "desired" element from a particular XML task.
     *  @param[out] options        Vector of TaskBuilderOptions objects.
     */
    void parseDesiredXmlElement(TiXmlElement* desiredElement, TaskBuilderOptions& options);
    
    /**
     *  Parses a written robot "segment" for a specific task.
     *
     *  @param[in]  segmentElement Segment/link element.
     *  @param[out] options        Vector of TaskBuilderOptions objects.
     */
    void parseSegmentXmlElement(TiXmlElement* segmentElement, TaskBuilderOptions& options);
    
    /**
     *  Uses util:parseJointIndexesXmElement to convert the written "jointIndexes" into a vector of integers specifying ()
     *
     *  @param[in]  jointIndexesElement Joint Indeces element.
     *  @param[out] options             Vector of TaskBuilderOptions objects.
     */
    void parseJointIndexesXmlElement(TiXmlElement* jointIndexesElement, TaskBuilderOptions& options);
    
    /**
     *  Parses the option "joints" for an XML task description. This is typically found in full posture tasks in which joint by joints configurations are given. Each joint is expected to be specified in the following xml format: <joint name="torso_roll" weight="0.01"/>. A weight for each joint can also be specified.
     *
     *  @param[in]  jointsElement Joints element.
     *  @param[out] options       Vector of TaskBuilderOptions objects.
     */
    void parseJointsXmlElement(TiXmlElement* jointsElement, TaskBuilderOptions& options);
    
    /**
     *  Parses the value of "weights" option for an XML task description. It is expected to be a vector of weights (double).
     *
     *  @param weightsElement Weights element.
     *  @param options        Vector of TaskBuilderOptions objects.
     */
    void parseWeightsXmlElement(TiXmlElement* weightsElement, TaskBuilderOptions& options);
    
    /**
     *  Used by the TaskConstructionManager::getBuilder method to check the consistency of the postural tasks description.
     *
     *  @param options Tasks' options.
     *  @param model   Robot OCRA model.
     */
    void correctArticularVariables(TaskBuilderOptions& options, Model::Ptr model);


};


} // namespace ocra

#endif //TASK_CONSTRUCTION_MANAGER_H
