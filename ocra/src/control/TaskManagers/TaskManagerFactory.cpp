#include "ocra/control/TaskManagers/TaskManagerFactory.h"


namespace ocra
{

TaskManagerFactory::TaskManagerFactory()
{
}

TaskManagerFactory::~TaskManagerFactory()
{
}

bool TaskManagerFactory::parseTasksXML(const std::string& filePath)
{
    return parseTasksXML(filePath.c_str());
}

bool TaskManagerFactory::parseTasksXML(const char * filePath)
{
    if(boost::filesystem::exists(filePath))
    {
        TiXmlDocument newTasksFile( filePath );
        if(!newTasksFile.LoadFile())
        {
            std::cout << "[ERROR] (TaskManagerFactory::parseTasksXML): Could not read the XML file: " << filePath << std::endl;
            return false;
        }
        else
            return parseTasksXML(&newTasksFile);
    }
    else
    {
        std::cout << "[ERROR] (TaskManagerFactory::parseTasksXML): Filepath provided does not exist: "<< filePath << std::endl;
        return false;
    }
}

bool TaskManagerFactory::parseTasksXML(TiXmlDocument* newTasksFile)
{
    if(newTasksFile == NULL)
    {
        std::cout << "[ERROR] newTasksFile arguement is NULL" << std::endl;
        return false;
    }

    else
    {
        for(TiXmlElement* xmlTask = newTasksFile->FirstChildElement("task"); xmlTask != NULL; xmlTask = xmlTask->NextSiblingElement("task"))
        {

            if ( (xmlTask->Attribute("name") != NULL) && (xmlTask->Attribute("type") != NULL) )
            {
                TaskManagerOptions currentTmArgs;

                currentTmArgs.taskName = xmlTask->Attribute("name");
                currentTmArgs.taskType = xmlTask->Attribute("type");

                std::vector<int> intVector;

                for(TiXmlElement* taskElem = xmlTask->FirstChildElement(); taskElem != NULL; taskElem = taskElem->NextSiblingElement())
                {
                    std::string currentElem = taskElem->Value();
                    if( currentElem == "params" ){
                        if (taskElem->QueryDoubleAttribute("kp", &currentTmArgs.kp)==TIXML_NO_ATTRIBUTE){currentTmArgs.kp=0.0;}
                        if (taskElem->QueryDoubleAttribute("kd", &currentTmArgs.kd)==TIXML_NO_ATTRIBUTE)
                        {
                            if (currentTmArgs.kp > 0.0) {
                                currentTmArgs.kd=2.0*sqrt(currentTmArgs.kp);
                            }else{currentTmArgs.kd=0.0;}
                        }
                        if (taskElem->QueryDoubleAttribute("weight", &currentTmArgs.weight)==TIXML_NO_ATTRIBUTE){currentTmArgs.weight=0.0; currentTmArgs.useWeightVectorConstructor=false;}else{currentTmArgs.useWeightVectorConstructor=false;}
                        if (taskElem->QueryIntAttribute("hierarchyLevel", &currentTmArgs.hierarchyLevel)==TIXML_NO_ATTRIBUTE){currentTmArgs.hierarchyLevel=0;}

                       //if (taskElem->QueryIntAttribute("axes", &currentTmArgs.axes)==TIXML_NO_ATTRIBUTE){currentTmArgs.axes=ocra::XYZ;}
                        if (taskElem->QueryDoubleAttribute("mu", &currentTmArgs.mu)==TIXML_NO_ATTRIBUTE){currentTmArgs.mu=1.0;}
                        if (taskElem->QueryDoubleAttribute("margin", &currentTmArgs.margin)==TIXML_NO_ATTRIBUTE){currentTmArgs.margin=0.05;}
                        currentTmArgs.axes=ocra::XYZ;
                        if (taskElem->Attribute("usesYarp") != NULL){
                            bool yarpBool;
                            std::string yarpString = std::string(taskElem->Attribute("usesYarp"));
                            if (yarpString=="true" || yarpString=="1") {
                                yarpBool = true;
                            }
                            else if (yarpString=="false" || yarpString=="0") {
                                yarpBool = false;
                            }
                            else {
                                yarpBool = true;
                            }
                            currentTmArgs.usesYarp = yarpBool;
                        }else{currentTmArgs.usesYarp=true;}
                    }

                    else if( currentElem == "offset" ){

                        if (taskElem->GetText() != NULL)
                            currentTmArgs.offset.push_back(stringToVectorXd(taskElem->GetText()));

                        else
                            currentTmArgs.offset.push_back(stringToVectorXd(getDisplacementArgs(taskElem)));

                    }

                    else if( currentElem == "desired" ){

                        if (taskElem->GetText() != NULL)
                            currentTmArgs.desired = stringToVectorXd(taskElem->GetText());

                        else
                            currentTmArgs.desired = stringToVectorXd(getDisplacementArgs(taskElem));
                    }

                    else if( currentElem == "segment" ){
                        currentTmArgs.segment = taskElem->GetText();
                    }

                    else if( currentElem == "jointIndexes" ){
                        currentTmArgs.jointIndexes = stringToVectorXi(taskElem->GetText());
                    }



                    else if( currentElem == "joints" ){
                        std::string jointIndexString, indexDesiredValueString, nameDesiredValueString, indexWeightString, nameWeightString;
                        for(TiXmlElement* jointElem = taskElem->FirstChildElement("joint"); jointElem != NULL; jointElem = jointElem->NextSiblingElement("joint"))
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
                                        currentTmArgs.useWeightVectorConstructor = true;

                                    }else{
                                        indexWeightString += "-1.0 ";
                                    }
                                }

                                else if(jointElem->Attribute("name") != NULL)
                                {
                                    currentTmArgs.jointNames.push_back(jointElem->Attribute("name"));
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
                                        currentTmArgs.useWeightVectorConstructor = true;

                                    }else{
                                        nameWeightString += "-1.0 ";
                                    }
                                }

                            }

                            currentTmArgs.jointIndexes = stringToVectorXi(jointIndexString.c_str());
                            currentTmArgs.indexDesired = stringToVectorXd(indexDesiredValueString.c_str());
                            currentTmArgs.nameDesired = stringToVectorXd(nameDesiredValueString.c_str());
                            currentTmArgs.indexWeightVector = stringToVectorXd(indexWeightString.c_str());
                            currentTmArgs.nameWeightVector = stringToVectorXd(nameWeightString.c_str());
                        }

                    }

                    else if( currentElem == "weights" ){
                        currentTmArgs.useWeightVectorConstructor = true;
                        currentTmArgs.weightVector = stringToVectorXd(taskElem->GetText());
                    }


                    else{
                        std::cout << "[WARNING] (parseTasksXML): Found a task manager tag --> "<< currentElem <<" <-- that doesn't match any known tags. Ignoring." << std::endl;
                    }

                }

                tmOptsVector.push_back(currentTmArgs);

            }
        }


        return true;
    }
}

bool TaskManagerFactory::addTaskManagerOptions(TaskManagerOptions& tmOpts)
{
    //TODO: Verify the options are good.
    tmOptsVector.push_back(tmOpts);
    //TODO: return true if success.
    return true;
}


bool TaskManagerFactory::addTaskManagersToSet(std::shared_ptr<ocra::Controller> ctrl, std::shared_ptr<ocra::Model> model, std::shared_ptr<TaskManagerSet> taskSet)
{
    if (tmOptsVector.size() > 0)
    {
        int successCount = 0;

        for(tmOptsIterator tmIt=tmOptsVector.begin(); tmIt != tmOptsVector.end(); ++tmIt)
        {
            std::cout << "\n=== Adding new task \"" << tmIt->taskName<<"\" to the taskSet ===" << std::endl;
            bool addSuccess = taskSet->addTaskManager(tmIt->taskName, constructTaskManager(ctrl, model, tmIt));
            if (!addSuccess) {
                std::cout << "\n[WARNING] Unable to add " << tmIt->taskName << std::endl;
            }
            successCount += addSuccess;
        }

        if (successCount==0)
        {
            std::cout << "[ERROR] (TaskManagerFactory::addTaskManagersToSequence): None of the parsed tasks were added to the sequence." << std::endl;
            return false;
        }
        else if (successCount==tmOptsVector.size())
        {
            return true;
        }
        else
        {
            std::cout << "[WARNING] (TaskManagerFactory::addTaskManagersToSequence): Only " << successCount << " of " << tmOptsVector.size() << " were added to the sequence. Double check XML syntax." << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "[WARNING] (TaskManagerFactory::addTaskManagersToSequence): There are no tasks to add. Doing nothing." << std::endl;
        return false;
    }
}


Eigen::VectorXd TaskManagerFactory::stringToVectorXd(const char * valueString)
{
    std::stringstream valueStream;
    std::vector<double> doubleVector;

    valueStream << valueString;

    do
    {
        // read as many numbers as possible.
        for (double number; valueStream >> number;) {
            doubleVector.push_back(number);
        }
        // consume and discard token from stream.
        if (valueStream.fail())
        {
            valueStream.clear();
            std::string token;
            valueStream >> token;
        }
    }
    while (!valueStream.eof());

    int nRows = doubleVector.size();
    Eigen::VectorXd eigenVector(nRows);
    for (int i=0; i<nRows; i++)
    {
        eigenVector[i] = doubleVector[i];
    }
    return eigenVector;
}


Eigen::VectorXi TaskManagerFactory::stringToVectorXi(const char * valueString)
{
    std::stringstream valueStream;
    std::vector<int> doubleVector;

    valueStream << valueString;

    do
    {
        // read as many numbers as possible.
        for (int number; valueStream >> number;) {
            doubleVector.push_back(number);
        }
        // consume and discard token from stream.
        if (valueStream.fail())
        {
            valueStream.clear();
            std::string token;
            valueStream >> token;
        }
    }
    while (!valueStream.eof());

    int nRows = doubleVector.size();
    Eigen::VectorXi eigenVector(nRows);
    for (int i=0; i<nRows; i++)
    {
        eigenVector[i] = doubleVector[i];
    }
    return eigenVector;
}


void TaskManagerFactory::printTaskArguments()
{
    int nTasks = tmOptsVector.size();
    for (int i=0; i<nTasks; i++)
    {
        std::cout << "\n=== Task " << i+1 << " of " << nTasks << " ===" << std::endl << std::endl;
        std::cout << "Task name:\n" << tmOptsVector[i].taskName << std::endl << std::endl;
        std::cout << "Task type:\n" << tmOptsVector[i].taskType << std::endl << std::endl;
        std::cout << "Segment:\n" << tmOptsVector[i].segment << std::endl << std::endl;
        std::cout << "kp:\n" << tmOptsVector[i].kp << std::endl << std::endl;
        std::cout << "kd:\n" << tmOptsVector[i].kd << std::endl << std::endl;
        std::cout << "hierarchyLevel:\n" << tmOptsVector[i].hierarchyLevel << std::endl << std::endl;
        std::cout << "weight:\n" << tmOptsVector[i].weight << std::endl << std::endl;
        std::cout << "weights:\n" << tmOptsVector[i].weightVector.transpose() << std::endl << std::endl;
        std::cout << "axes:\n" << tmOptsVector[i].axes << std::endl << std::endl;
        std::cout << "offset:\n";
        for(int j=0; j<tmOptsVector[i].offset.size(); j++){std::cout << tmOptsVector[i].offset[j].transpose() << std::endl;}std::cout << std::endl;
        std::cout << "jointIndexes:\n" << tmOptsVector[i].jointIndexes.transpose() << std::endl << std::endl;
        std::cout << "desired:\n" << tmOptsVector[i].desired.transpose() << std::endl << std::endl;


    }
}



const char * TaskManagerFactory::getDisplacementArgs(TiXmlElement* xmlElem)
{
    if(xmlElem != NULL)
    {
        std::string dispString;
        std::vector<std::string> dispAttributes;

        dispAttributes.push_back("x");
        dispAttributes.push_back("y");
        dispAttributes.push_back("z");
        dispAttributes.push_back("qw");
        dispAttributes.push_back("qx");
        dispAttributes.push_back("qy");
        dispAttributes.push_back("qz");

        std::vector<std::string>::iterator stIt;

        for (stIt=dispAttributes.begin(); stIt != dispAttributes.end(); stIt++)
        {
            bool haveAlreadyFoundValue = false;
            for (TiXmlAttribute* xmlAttrib=xmlElem->FirstAttribute(); xmlAttrib != NULL; xmlAttrib = xmlAttrib->Next())
            {
                // std::cout << "stIT " << *stIt  << " xmlAttrib->Value() " << xmlAttrib->Value()<< std::endl;
                if ( (*stIt == xmlAttrib->Name()) && (!haveAlreadyFoundValue) )
                {
                    dispString += xmlAttrib->Value();
                    dispString += " ";
                    haveAlreadyFoundValue = true;
                }
            }
            if (!haveAlreadyFoundValue) {
                std::string fallbackString = "0.0 ";
                if (*stIt == "qw") {
                    fallbackString = "1.0 ";
                }
                dispString += fallbackString;
            }
        }


        return dispString.c_str();
    }
}





void TaskManagerFactory::prepareTaskManagerArguments(std::shared_ptr<ocra::Model> model, tmOptsIterator tmOptsPtr)
{
    int sizeDof = model->nbInternalDofs();
    if (tmOptsPtr->offset.empty()) {
        tmOptsPtr->offset.push_back(Eigen::VectorXd::Zero(3));
    }
    int sizeDesired = tmOptsPtr->desired.rows();
    int sizeIndexDesired = tmOptsPtr->indexDesired.rows();
    int sizeNameDesired = tmOptsPtr->nameDesired.rows();
    int sizeWeightVector = tmOptsPtr->weightVector.rows();
    int sizeIndexWeightVector = tmOptsPtr->indexWeightVector.rows();
    int sizeNameWeightVector = tmOptsPtr->nameWeightVector.rows();
    int sizeJointIndexes = tmOptsPtr->jointIndexes.rows();
    int sizeJointNames = tmOptsPtr->jointNames.size();


    if ((tmOptsPtr->taskType == "FullPostureTaskManager") || (tmOptsPtr->taskType == "PartialPostureTaskManager"))
    {
        int sizeDofConcerned;
        if(tmOptsPtr->taskType == "FullPostureTaskManager")
        {
            sizeDofConcerned = sizeDof;
        }else if(tmOptsPtr->taskType == "PartialPostureTaskManager"){
            sizeDofConcerned = sizeIndexDesired+sizeNameDesired;
        }

        if (sizeDesired == 0)
        {
            if(tmOptsPtr->taskType == "FullPostureTaskManager")
            {
                tmOptsPtr->desired = model->getJointPositions();
            }else if(tmOptsPtr->taskType == "PartialPostureTaskManager"){
                tmOptsPtr->desired = Eigen::VectorXd::Zero(sizeDofConcerned);
            }
        }
        else
        {
            if (sizeDesired == 1)
            {
                tmOptsPtr->desired = Eigen::VectorXd::Constant(sizeDofConcerned, tmOptsPtr->desired[0]);
            }
            else if (sizeDesired != sizeDofConcerned)
            {
                if(tmOptsPtr->taskType == "FullPostureTaskManager")
                {
                    tmOptsPtr->desired = model->getJointPositions();
                }else if(tmOptsPtr->taskType == "PartialPostureTaskManager"){
                    tmOptsPtr->desired = Eigen::VectorXd::Zero(sizeDofConcerned);
                }
            }
        }
        if (tmOptsPtr->useWeightVectorConstructor)
        {
            if (sizeWeightVector != sizeDofConcerned)
            {
                tmOptsPtr->weightVector = Eigen::VectorXd::Constant(sizeDofConcerned, tmOptsPtr->weight);
            }
        }


        if(tmOptsPtr->taskType == "FullPostureTaskManager")
        {
            if (sizeJointIndexes>0 && sizeJointIndexes == sizeIndexDesired)
            {
                for(int i=0; i<sizeJointIndexes; i++)
                {
                    if(tmOptsPtr->indexDesired(i) != -1000.0)
                    {
                        tmOptsPtr->desired(tmOptsPtr->jointIndexes(i)) = tmOptsPtr->indexDesired(i);
                    }
                    if (tmOptsPtr->useWeightVectorConstructor)
                    {
                        if (tmOptsPtr->indexWeightVector(i) >= 0.0)
                        {
                            tmOptsPtr->weightVector(tmOptsPtr->jointIndexes(i)) = tmOptsPtr->indexWeightVector(i);
                        }
                    }
                }
            }
            if (sizeJointNames>0 && sizeJointNames == sizeNameDesired)
            {
                for(int i=0; i<sizeJointNames; i++)
                {
                    if(tmOptsPtr->nameDesired(i) != -1000.0)
                    {
                    tmOptsPtr->desired(model->getDofIndex(tmOptsPtr->jointNames[i])) = tmOptsPtr->nameDesired(i);
                    }
                    if (tmOptsPtr->useWeightVectorConstructor)
                    {
                        if (tmOptsPtr->nameWeightVector(i) >= 0.0)
                        {
                            tmOptsPtr->weightVector(model->getDofIndex(tmOptsPtr->jointNames[i])) = tmOptsPtr->nameWeightVector(i);
                        }
                    }
                }
            }
        }
        else if(tmOptsPtr->taskType == "PartialPostureTaskManager")
        {
            Eigen::VectorXi jointIndexesTemp(sizeDofConcerned);
            int indexCounter = 0;

            if (sizeJointIndexes>0 && sizeJointIndexes == sizeIndexDesired)
            {
                for(int i=0; i<sizeJointIndexes; i++)
                {
                    jointIndexesTemp(indexCounter) = tmOptsPtr->jointIndexes(i);
                    if(tmOptsPtr->indexDesired(i) != -1000.0)
                    {
                        tmOptsPtr->desired(indexCounter) = tmOptsPtr->indexDesired(i);
                    }
                    if (tmOptsPtr->useWeightVectorConstructor)
                    {
                        if (tmOptsPtr->indexWeightVector(i) >= 0.0)
                        {
                            tmOptsPtr->weightVector(indexCounter) = tmOptsPtr->indexWeightVector(i);
                        }
                    }
                    indexCounter++;
                }
            }
            if (sizeJointNames>0 && sizeJointNames == sizeNameDesired)
            {
                for(int i=0; i<sizeJointNames; i++)
                {
                    jointIndexesTemp(indexCounter) = model->getDofIndex(tmOptsPtr->jointNames[i]);
                    if(tmOptsPtr->nameDesired(i) == -1000.0)
                    {
                        tmOptsPtr->desired(indexCounter) = model->getJointPositions()(model->getDofIndex(tmOptsPtr->jointNames[i]));
                    }
                    else{
                        tmOptsPtr->desired(indexCounter) = tmOptsPtr->nameDesired(i);
                    }
                    if (tmOptsPtr->useWeightVectorConstructor)
                    {
                        if (tmOptsPtr->nameWeightVector(i) >= 0.0)
                        {
                            tmOptsPtr->weightVector(indexCounter) = tmOptsPtr->nameWeightVector(i);
                        }
                    }
                    indexCounter++;
                }
            }
            tmOptsPtr->jointIndexes.resize(sizeDofConcerned);
            tmOptsPtr->jointIndexes = jointIndexesTemp;

        }

    }





}




std::shared_ptr<TaskManager> TaskManagerFactory::constructTaskManager(std::shared_ptr<ocra::Controller> ctrl,
                                                                      std::shared_ptr<ocra::Model> model,
                                                                      tmOptsIterator tmOptsPtr)
{




    prepareTaskManagerArguments(model, tmOptsPtr);

    int sizeDof = model->nbInternalDofs();
    int sizeOffset = tmOptsPtr->offset[0].rows();
    int sizeJointIndexes = tmOptsPtr->jointIndexes.rows();
    int sizeDesired = tmOptsPtr->desired.rows();



    std::shared_ptr<TaskManager> newTaskManager;

    if(tmOptsPtr->taskType == "CoMTaskManager")
    {
        if (sizeDesired == 0)
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<CoMTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);            }
            else {
                newTaskManager = std::make_shared<CoMTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);            }
        }
        else
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<CoMTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->desired,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);            }
            else {
                newTaskManager = std::make_shared<CoMTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->desired,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);            }
        }


        return newTaskManager;
    }

    else if(tmOptsPtr->taskType == "ContactSetTaskManager")
    {
        newTaskManager = std::make_shared<ContactSetTaskManager>(*ctrl, *model,
                                                tmOptsPtr->taskName,
                                                tmOptsPtr->segment,
                                                eigenVectorToDisplacementd(tmOptsPtr->offset),
                                                tmOptsPtr->mu,
                                                tmOptsPtr->margin,
                                                tmOptsPtr->hierarchyLevel,
                                                tmOptsPtr->usesYarp);

        return newTaskManager;
    }

    else if(tmOptsPtr->taskType == "ContactTaskManager")
    {

        newTaskManager = std::make_shared<ContactTaskManager>(*ctrl, *model,
                                                tmOptsPtr->taskName,
                                                tmOptsPtr->segment,
                                                eigenVectorToDisplacementd(tmOptsPtr->offset.front()),
                                                tmOptsPtr->mu,
                                                tmOptsPtr->margin,
                                                tmOptsPtr->hierarchyLevel,
                                                tmOptsPtr->usesYarp);
        return newTaskManager;
    }

    else if(tmOptsPtr->taskType == "FullPostureTaskManager")
    {
        if (sizeDesired == 0)
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<FullPostureTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::FullState::INTERNAL,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
            else {
                newTaskManager = std::make_shared<FullPostureTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::FullState::INTERNAL,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
        }
        else
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<FullPostureTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::FullState::INTERNAL,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->desired,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
            else {
                newTaskManager = std::make_shared<FullPostureTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        ocra::FullState::INTERNAL,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->desired,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
        }




        return newTaskManager;
    }

    else if(tmOptsPtr->taskType == "PartialPostureTaskManager")
    {
        if (sizeJointIndexes>0 && sizeJointIndexes<=sizeDof)
        {
            if (sizeDesired==0)
            {
                if (tmOptsPtr->useWeightVectorConstructor) {
                    newTaskManager = std::make_shared<PartialPostureTaskManager>(*ctrl, *model,
                                                            tmOptsPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            tmOptsPtr->jointIndexes,
                                                            tmOptsPtr->kp,
                                                            tmOptsPtr->kd,
                                                            tmOptsPtr->weightVector,
                                                            tmOptsPtr->hierarchyLevel,
                                                            tmOptsPtr->usesYarp);
                }
                else {
                    newTaskManager = std::make_shared<PartialPostureTaskManager>(*ctrl, *model,
                                                            tmOptsPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            tmOptsPtr->jointIndexes,
                                                            tmOptsPtr->kp,
                                                            tmOptsPtr->kd,
                                                            tmOptsPtr->weight,
                                                            tmOptsPtr->hierarchyLevel,
                                                            tmOptsPtr->usesYarp);
                }
            }
            else
            {
                if (tmOptsPtr->useWeightVectorConstructor) {
                    newTaskManager = std::make_shared<PartialPostureTaskManager>(*ctrl, *model,
                                                            tmOptsPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            tmOptsPtr->jointIndexes,
                                                            tmOptsPtr->kp,
                                                            tmOptsPtr->kd,
                                                            tmOptsPtr->weightVector,
                                                            tmOptsPtr->desired,
                                                            tmOptsPtr->hierarchyLevel,
                                                            tmOptsPtr->usesYarp);
                }
                else {
                    newTaskManager = std::make_shared<PartialPostureTaskManager>(*ctrl, *model,
                                                            tmOptsPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            tmOptsPtr->jointIndexes,
                                                            tmOptsPtr->kp,
                                                            tmOptsPtr->kd,
                                                            tmOptsPtr->weight,
                                                            tmOptsPtr->desired,
                                                            tmOptsPtr->hierarchyLevel,
                                                            tmOptsPtr->usesYarp);
                }
            }


            return newTaskManager;
        }
        else
            return NULL;
    }

    else if(tmOptsPtr->taskType == "SegCartesianTaskManager")
    {

        if(sizeOffset!=3)
        {
            tmOptsPtr->offset.front() = Eigen::VectorXd::Zero(3);
        }

        if (sizeDesired!=7)
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<SegCartesianTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->offset.front(),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
            else {
                newTaskManager = std::make_shared<SegCartesianTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->offset.front(),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }

            return newTaskManager;
        }


        else if(sizeDesired == 7)
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<SegCartesianTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->offset.front(),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->desired.head(3),
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
            else {
                newTaskManager = std::make_shared<SegCartesianTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->offset.front(),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->desired.head(3),
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }

            return newTaskManager;
        }


    }

    else if(tmOptsPtr->taskType == "SegOrientationTaskManager")
    {

        if (sizeDesired!=7)
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<SegOrientationTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
            else {
                newTaskManager = std::make_shared<SegOrientationTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }

            return newTaskManager;
        }


        else if(sizeDesired == 7)
        {
            // We have to reorganize the quaternion vector when we use this constructor because it stores the coeffs in reverse order.
            double tmpW = tmOptsPtr->desired(0);
            tmOptsPtr->desired.head(3) = tmOptsPtr->desired.tail(3).eval();
            tmOptsPtr->desired(3) = tmpW;

            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<SegOrientationTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        Eigen::Rotation3d(tmOptsPtr->desired.tail(4).data()),
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
            else {
                newTaskManager = std::make_shared<SegOrientationTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        Eigen::Rotation3d(tmOptsPtr->desired.tail(4).data()),
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }

            return newTaskManager;
        }
    }

    else if(tmOptsPtr->taskType == "SegPoseTaskManager")
    {

        if (sizeDesired!=7)
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<SegPoseTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        eigenVectorToDisplacementd(tmOptsPtr->offset.front()),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }
            else {
                newTaskManager = std::make_shared<SegPoseTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        eigenVectorToDisplacementd(tmOptsPtr->offset.front()),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp);
            }

            return newTaskManager;
        }


        else if(sizeDesired == 7)
        {
            if (tmOptsPtr->useWeightVectorConstructor) {
                newTaskManager = std::make_shared<SegPoseTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        eigenVectorToDisplacementd(tmOptsPtr->offset.front()),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weightVector,
                                                        eigenVectorToDisplacementd(tmOptsPtr->desired),
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp );
            }
            else {
                newTaskManager = std::make_shared<SegPoseTaskManager>(*ctrl, *model,
                                                        tmOptsPtr->taskName,
                                                        tmOptsPtr->segment,
                                                        eigenVectorToDisplacementd(tmOptsPtr->offset.front()),
                                                        ocra::ECartesianDof(tmOptsPtr->axes),//ocra::XYZ,
                                                        tmOptsPtr->kp,
                                                        tmOptsPtr->kd,
                                                        tmOptsPtr->weight,
                                                        eigenVectorToDisplacementd(tmOptsPtr->desired),
                                                        tmOptsPtr->hierarchyLevel,
                                                        tmOptsPtr->usesYarp );
            }


            return newTaskManager;
        }
    }



    else
    {
        std::cout << "[ERROR] (TaskManagerFactory::constructTaskManager): The task type provided --> " << tmOptsPtr->taskType << " <-- doesn't match any valid taskManager type. Ignoring." << std::endl;
        return NULL;
    }


}


Eigen::Displacementd TaskManagerFactory::eigenVectorToDisplacementd(Eigen::VectorXd& eigenVector)
{
    Eigen::VectorXd tmpVector = Eigen::VectorXd::Zero(7);
    tmpVector(3) = 1.0;

    if (eigenVector.rows()==3) {
        tmpVector.head(3) = eigenVector;
    }
    else if (eigenVector.rows()==7) {
        tmpVector = eigenVector;
    }

    return Eigen::Displacementd(tmpVector(0), tmpVector(1), tmpVector(2), tmpVector(3), tmpVector(4), tmpVector(5), tmpVector(6));
}

std::vector<Eigen::Displacementd> TaskManagerFactory::eigenVectorToDisplacementd(std::vector<Eigen::VectorXd>& eigenVector)
{
    std::vector<Eigen::Displacementd> dispVec;
    for(int i=0; i<eigenVector.size(); i++)
    {
        dispVec.push_back(eigenVectorToDisplacementd(eigenVector[i]));
    }
    return dispVec;
}




}
