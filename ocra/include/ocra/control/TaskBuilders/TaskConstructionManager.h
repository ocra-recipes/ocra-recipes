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

    Eigen::VectorXd stringToVectorXd(const char * valueString);
    Eigen::VectorXi stringToVectorXi(const char * valueString);
    std::string getDisplacementArgs(TiXmlElement* xmlElem);
    Eigen::Displacementd eigenVectorToDisplacementd(Eigen::VectorXd& eigenVector);
    std::vector<Eigen::Displacementd> eigenVectorToDisplacementd(std::vector<Eigen::VectorXd>& eigenVector);

};


} // namespace ocra

#endif //TASK_CONSTRUCTION_MANAGER_H
