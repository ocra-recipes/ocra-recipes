#include <ocra/control/TaskBuilders/TaskConstructionManager.h>

using namespace ocra;


TaskConstructionManager::TaskConstructionManager(Model::Ptr model, Controller::Ptr controller, std::vector<TaskBuilderOptions> optionsVector)
{
    addTasksToController(model, controller, optionsVector);
}

TaskConstructionManager::TaskConstructionManager(Model::Ptr model, Controller::Ptr controller, const std::string& optionsXmlFilePath)
{
    addTasksToController(model, controller, parseTaskOptionsFromXml(optionsXmlFilePath));
}

TaskConstructionManager::~TaskConstructionManager()
{

}

void TaskConstructionManager::addTasksToController(Model::Ptr model, Controller::Ptr controller, std::vector<TaskBuilderOptions> optionsVector)
{
    for(auto options : optionsVector){
        TaskBuilder::Ptr builder = getBuilder(options, model);

        if (builder) {
            builder->buildTask();
            controller->addTask(builder->getTask());
            builder->setTaskParameters();
        }
    }
}

TaskBuilder::Ptr TaskConstructionManager::getBuilder(TaskBuilderOptions options, Model::Ptr model)
{
    TaskBuilder::Ptr TaskBldrPtr;
    if (options.taskType=="cartesian") {
        TaskBldrPtr = std::make_shared<CartesianTaskBuilder>(options, model);
    }
    // else if (options.taskType=="orientation") {
    //  TaskBldrPtr = std::make_shared<OrientationTaskBuilder>(options, model);
    // }
    else {
        /* Error message */
    }
    return TaskBldrPtr;
}

std::vector<TaskBuilderOptions> TaskConstructionManager::parseTaskOptionsFromXml(const std::string& optionsXmlFilePath)
{
    std::vector<TaskBuilderOptions> emptyOptionsVector;
    if(boost::filesystem::exists(optionsXmlFilePath.c_str()))
    {
        TiXmlDocument newTasksFile( optionsXmlFilePath.c_str() );
        if(!newTasksFile.LoadFile())
        {
            std::cout << "[ERROR] (TaskConstructionManager::parseTaskOptionsFromXml): Could not read the XML file: " << optionsXmlFilePath << std::endl;
            return emptyOptionsVector;
        }
        else
            return parseTaskOptionsFromXml(&newTasksFile);
    }
    else
    {
        std::cout << "[ERROR] (TaskConstructionManager::parseTaskOptionsFromXml): Filepath provided does not exist: "<< optionsXmlFilePath << std::endl;
        return emptyOptionsVector;
    }
}

std::vector<TaskBuilderOptions> TaskConstructionManager::parseTaskOptionsFromXml(TiXmlDocument* newTasksFile)
{
    std::vector<TaskBuilderOptions> optionsVector;

    if(newTasksFile == NULL) {
        std::cout << "[ERROR] newTasksFile arguement is NULL" << std::endl;
    } else {
        for(TiXmlElement* xmlTask = newTasksFile->FirstChildElement("task"); xmlTask != NULL; xmlTask = xmlTask->NextSiblingElement("task")) {
            TaskBuilderOptions options;
            if(parseTaskXmlElement(xmlTask, options)) {
                optionsVector.push_back(options);
            }
        }
    }
    return optionsVector;
}

bool TaskConstructionManager::parseTaskXmlElement(TiXmlElement* xmlTask, TaskBuilderOptions& options)
{
    if ( (xmlTask->Attribute("name") != NULL) && (xmlTask->Attribute("type") != NULL) ) {

        options.taskName = xmlTask->Attribute("name");
        options.taskType = xmlTask->Attribute("type");

        for(TiXmlElement* taskElem = xmlTask->FirstChildElement(); taskElem != NULL; taskElem = taskElem->NextSiblingElement()) {

            std::string currentElemString = taskElem->Value();

            if (currentElemString == "params") {
                parseParamXmlElement(taskElem, options);
            } else if (currentElemString == "offset") {
                parseOffsetXmlElement(taskElem, options);
            } else if (currentElemString == "desired") {
                parseDesiredXmlElement(taskElem, options);
            } else if (currentElemString == "segment") {
                parseSegmentXmlElement(taskElem, options);
            } else if (currentElemString == "jointIndexes") {
                parseJointIndexesXmlElement(taskElem, options);
            } else if (currentElemString == "joints") {
                parseJointsXmlElement(taskElem, options);
            } else if (currentElemString == "weights") {
                parseWeightsXmlElement(taskElem, options);
            } else {
                std::cout << "[WARNING] (parseTaskOptionsFromXml): Found a task manager tag --> "<< currentElemString <<" <-- that doesn't match any known tags. Ignoring." << std::endl;
            }
        }
        return true;
    }
    return false;
}

void TaskConstructionManager::parseParamXmlElement(TiXmlElement* paramElement, TaskBuilderOptions& options)
{

    if (paramElement->QueryDoubleAttribute("kp", &options.kp)==TIXML_NO_ATTRIBUTE) {
        options.kp=0.0;
    }

    if (paramElement->QueryDoubleAttribute("kd", &options.kd)==TIXML_NO_ATTRIBUTE) {
        if (options.kp > 0.0) {
            options.kd=2.0*sqrt(options.kp);
        }else{options.kd=0.0;}
    }

    if (paramElement->QueryDoubleAttribute("weight", &options.weight)==TIXML_NO_ATTRIBUTE) {
        options.weight=0.0; options.useWeightVectorConstructor=false;
    } else {
        options.useWeightVectorConstructor=false;
    }
    if (paramElement->QueryIntAttribute("hierarchyLevel", &options.hierarchyLevel)==TIXML_NO_ATTRIBUTE) {
        options.hierarchyLevel=-1;
    }

//    if (paramElement->QueryIntAttribute("axes", &options.axes)==TIXML_NO_ATTRIBUTE) {
//        options.axes=ocra::XYZ;
//    }
    if (paramElement->QueryDoubleAttribute("mu", &options.mu)==TIXML_NO_ATTRIBUTE) {
        options.mu=1.0;
    }
    if (paramElement->QueryDoubleAttribute("margin", &options.margin)==TIXML_NO_ATTRIBUTE) {
        options.margin=0.05;
    }

    options.axes=ocra::XYZ;

    if (paramElement->Attribute("usesYarp") != NULL) {
        bool yarpBool;
        std::string yarpString = std::string(paramElement->Attribute("usesYarp"));
        if (yarpString=="true" || yarpString=="1") {
            yarpBool = true;
        }
        else if (yarpString=="false" || yarpString=="0") {
            yarpBool = false;
        }
        else {
            yarpBool = true;
        }
        options.usesYarp = yarpBool;
    } else {
        options.usesYarp=true;
    }
}

void TaskConstructionManager::parseOffsetXmlElement(TiXmlElement* offsetElement, TaskBuilderOptions& options)
{
    if (offsetElement->GetText() != NULL) {
        options.offset.push_back(util::stringToVectorXd(offsetElement->GetText()));
    } else {
        options.offset.push_back(util::stringToVectorXd(util::getDisplacementArgs(offsetElement).c_str()));
    }
}

void TaskConstructionManager::parseDesiredXmlElement(TiXmlElement* desiredElement, TaskBuilderOptions& options)
{
    if (desiredElement->GetText() != NULL) {
        options.desired = util::stringToVectorXd(desiredElement->GetText());
    } else {
        options.desired = util::stringToVectorXd(util::getDisplacementArgs(desiredElement).c_str());
    }
}

void TaskConstructionManager::parseSegmentXmlElement(TiXmlElement* segmentElement, TaskBuilderOptions& options)
{
    options.segment = segmentElement->GetText();
}

void TaskConstructionManager::parseJointIndexesXmlElement(TiXmlElement* jointIndexesElement, TaskBuilderOptions& options)
{
    options.jointIndexes = util::stringToVectorXi(jointIndexesElement->GetText());
}

void TaskConstructionManager::parseJointsXmlElement(TiXmlElement* jointsElement, TaskBuilderOptions& options)
{
    std::string jointIndexString, indexDesiredValueString, nameDesiredValueString, indexWeightString, nameWeightString;
    for(TiXmlElement* jointElem = jointsElement->FirstChildElement("joint"); jointElem != NULL; jointElem = jointElem->NextSiblingElement("joint"))
    {
        if ( (jointElem->Attribute("name") != NULL) || (jointElem->Attribute("index") != NULL) )
        {
            if(jointElem->Attribute("index") != NULL)
            {
                jointIndexString += jointElem->Attribute("index");
                jointIndexString += " ";
                if (jointElem->Attribute("des") != NULL)
                {
                    indexDesiredValueString += jointElem->Attribute("des");
                    indexDesiredValueString += " ";
                }else{
                    indexDesiredValueString += "-1000.0 ";
                }
                if (jointElem->Attribute("weight") != NULL)
                {
                    indexWeightString += jointElem->Attribute("weight");
                    indexWeightString += " ";
                    options.useWeightVectorConstructor = true;

                }else{
                    indexWeightString += "-1.0 ";
                }
            }

            else if(jointElem->Attribute("name") != NULL)
            {
                options.jointNames.push_back(jointElem->Attribute("name"));
                if (jointElem->Attribute("des") != NULL)
                {
                    nameDesiredValueString += jointElem->Attribute("des");
                    nameDesiredValueString += " ";
                }else{
                    nameDesiredValueString += "-1000.0 ";
                }
                if (jointElem->Attribute("weight") != NULL)
                {
                    nameWeightString += jointElem->Attribute("weight");
                    nameWeightString += " ";
                    options.useWeightVectorConstructor = true;

                }else{
                    nameWeightString += "-1.0 ";
                }
            }

        }

        options.jointIndexes = util::stringToVectorXi(jointIndexString.c_str());
        options.indexDesired = util::stringToVectorXd(indexDesiredValueString.c_str());
        options.nameDesired = util::stringToVectorXd(nameDesiredValueString.c_str());
        options.indexWeightVector = util::stringToVectorXd(indexWeightString.c_str());
        options.nameWeightVector = util::stringToVectorXd(nameWeightString.c_str());
    }
}

void TaskConstructionManager::parseWeightsXmlElement(TiXmlElement* weightsElement, TaskBuilderOptions& options)
{
    options.useWeightVectorConstructor = true;
    options.weightVector = util::stringToVectorXd(weightsElement->GetText());
}
