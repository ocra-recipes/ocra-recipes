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

namespace ocra {

class TaskConstructionManager {
DEFINE_CLASS_POINTER_TYPEDEFS(TaskConstructionManager)

public:
    TaskConstructionManager(Model::Ptr model, Controller::Ptr controller, std::vector<TaskBuilderOptions> optionsVector);
    TaskConstructionManager(Model::Ptr model, Controller::Ptr controller, const std::string& optionsXmlFilePath);
    virtual ~TaskConstructionManager();


    void addTasksToController(Model::Ptr model, Controller::Ptr controller, std::vector<TaskBuilderOptions> optionsVector);

    TaskBuilder::Ptr getBuilder(TaskBuilderOptions options, Model::Ptr model);

    std::vector<TaskBuilderOptions> parseTaskOptionsFromXml(const std::string& optionsXmlFilePath);

private:
    std::vector<TaskBuilderOptions> parseTaskOptionsFromXml(TiXmlDocument* newTasksFile);

    bool parseTaskXmlElement(TiXmlElement* xmlTask, TaskBuilderOptions& options);
    bool parseTaskNameAndType(TiXmlElement* xmlTask, TaskBuilderOptions& options);    
    void parseParamXmlElement(TiXmlElement* currentElem, TaskBuilderOptions& options);
    void parseOffsetXmlElement(TiXmlElement* offsetElement, TaskBuilderOptions& options);
    void parseDesiredXmlElement(TiXmlElement* desiredElement, TaskBuilderOptions& options);
    void parseSegmentXmlElement(TiXmlElement* segmentElement, TaskBuilderOptions& options);
    void parseJointIndexesXmlElement(TiXmlElement* jointIndexesElement, TaskBuilderOptions& options);
    void parseJointsXmlElement(TiXmlElement* jointsElement, TaskBuilderOptions& options);
    void parseWeightsXmlElement(TiXmlElement* weightsElement, TaskBuilderOptions& options);


};


} // namespace ocra

#endif //TASK_CONSTRUCTION_MANAGER_H
