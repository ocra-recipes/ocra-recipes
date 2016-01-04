#include "wocra/Tasks/wOcraTaskParser.h"


namespace wocra
{
    wOcraTaskParser::~wOcraTaskParser()
    {
    }

    bool wOcraTaskParser::parseTasksXML(const char * filePath)
    {
        if(boost::filesystem::exists(filePath))
        {
            TiXmlDocument newTasksFile( filePath );
            if(!newTasksFile.LoadFile())
            {
                std::cout << "[ERROR] (wOcraTaskParser::parseTasksXML): Could not read the XML file: " << filePath << std::endl;
                return false;
            }
            else
                return parseTasksXML(&newTasksFile);
        }
        else
        {
            std::cout << "[ERROR] (wOcraTaskParser::parseTasksXML): Filepath provided does not exist: "<< filePath << std::endl;
        }
    }

    bool wOcraTaskParser::parseTasksXML(TiXmlDocument* newTasksFile)
    {
        if(newTasksFile == NULL)
            return false;

        else
        {
            for(TiXmlElement* xmlTask = newTasksFile->FirstChildElement("task"); xmlTask != NULL; xmlTask = xmlTask->NextSiblingElement("task"))
            {

                if ( (xmlTask->Attribute("name") != NULL) && (xmlTask->Attribute("type") != NULL) )
                {
                    taskManagerArgs currentTmArgs;

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
                            if (taskElem->QueryIntAttribute("axes", &currentTmArgs.axes)==TIXML_NO_ATTRIBUTE){currentTmArgs.axes=ocra::XYZ;}
                            if (taskElem->QueryDoubleAttribute("mu", &currentTmArgs.mu)==TIXML_NO_ATTRIBUTE){currentTmArgs.mu=1.0;}
                            if (taskElem->QueryDoubleAttribute("margin", &currentTmArgs.margin)==TIXML_NO_ATTRIBUTE){currentTmArgs.margin=0.05;}
                            if (taskElem->QueryIntAttribute("axes", &currentTmArgs.axes)==TIXML_NO_ATTRIBUTE){currentTmArgs.axes=ocra::XYZ;}
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

                    tmArgsVector.push_back(currentTmArgs);

                }
            }


            return true;
        }
    }


    bool wOcraTaskParser::addTaskManagersToSequence(wOcraController& ctrl, const wOcraModel& model, wOcraTaskSequenceBase* sequence)
    {
        if (tmArgsVector.size() > 0)
        {
            int successCount = 0;

            for(tmArgsIt=tmArgsVector.begin(); tmArgsIt != tmArgsVector.end(); tmArgsIt++)
            {
                std::cout << "\n=== Adding new task \"" << tmArgsIt->taskName<<"\" to the sequence ===" << std::endl;
                bool addSuccess = sequence->addTaskManager(tmArgsIt->taskName, constructTaskManager(ctrl, model, tmArgsIt));
                if (!addSuccess) {
                    std::cout << "\n[WARNING] Unable to add " << tmArgsIt->taskName << std::endl;
                }
                successCount += addSuccess;
            }

            if (successCount==0)
            {
                std::cout << "[ERROR] (wOcraTaskParser::addTaskManagersToSequence): None of the parsed tasks were added to the sequence." << std::endl;
                return false;
            }
            else if (successCount==tmArgsVector.size())
            {
                return true;
            }
            else
            {
                std::cout << "[WARNING] (wOcraTaskParser::addTaskManagersToSequence): Only " << successCount << " of " << tmArgsVector.size() << " were added to the sequence. Double check XML syntax." << std::endl;
                return false;
            }
        }
        else
        {
            std::cout << "[WARNING] (wOcraTaskParser::addTaskManagersToSequence): There are no tasks to add. Doing nothing." << std::endl;
            return false;
        }
    }


    Eigen::VectorXd wOcraTaskParser::stringToVectorXd(const char * valueString)
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


    Eigen::VectorXi wOcraTaskParser::stringToVectorXi(const char * valueString)
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


    void wOcraTaskParser::printTaskArguments()
    {
        int nTasks = tmArgsVector.size();
        for (int i=0; i<nTasks; i++)
        {
            std::cout << "\n=== Task " << i+1 << " of " << nTasks << " ===" << std::endl << std::endl;
            std::cout << "Task name:\n" << tmArgsVector[i].taskName << std::endl << std::endl;
            std::cout << "Task type:\n" << tmArgsVector[i].taskType << std::endl << std::endl;
            std::cout << "Segment:\n" << tmArgsVector[i].segment << std::endl << std::endl;
            std::cout << "kp:\n" << tmArgsVector[i].kp << std::endl << std::endl;
            std::cout << "kd:\n" << tmArgsVector[i].kd << std::endl << std::endl;
            std::cout << "weight:\n" << tmArgsVector[i].weight << std::endl << std::endl;
            std::cout << "weights:\n" << tmArgsVector[i].weightVector.transpose() << std::endl << std::endl;
            std::cout << "axes:\n" << tmArgsVector[i].axes << std::endl << std::endl;
            std::cout << "offset:\n";
            for(int j=0; j<tmArgsVector[i].offset.size(); j++){std::cout << tmArgsVector[i].offset[j].transpose() << std::endl;}std::cout << std::endl;
            std::cout << "jointIndexes:\n" << tmArgsVector[i].jointIndexes.transpose() << std::endl << std::endl;
            std::cout << "desired:\n" << tmArgsVector[i].desired.transpose() << std::endl << std::endl;


        }
    }



    const char * wOcraTaskParser::getDisplacementArgs(TiXmlElement* xmlElem)
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





    void wOcraTaskParser::prepareTaskManagerArguments(const wOcraModel& model, std::vector<taskManagerArgs>::iterator argStructPtr)
    {
        int sizeDof = model.nbInternalDofs();
        if (argStructPtr->offset.empty()) {
            argStructPtr->offset.push_back(Eigen::VectorXd::Zero(3));
        }
        int sizeDesired = argStructPtr->desired.rows();
        int sizeIndexDesired = argStructPtr->indexDesired.rows();
        int sizeNameDesired = argStructPtr->nameDesired.rows();
        int sizeWeightVector = argStructPtr->weightVector.rows();
        int sizeIndexWeightVector = argStructPtr->indexWeightVector.rows();
        int sizeNameWeightVector = argStructPtr->nameWeightVector.rows();
        int sizeJointIndexes = argStructPtr->jointIndexes.rows();
        int sizeJointNames = argStructPtr->jointNames.size();


        if ((argStructPtr->taskType == "wOcraFullPostureTaskManager") || (argStructPtr->taskType == "wOcraPartialPostureTaskManager"))
        {
            int sizeDofConcerned;
            if(argStructPtr->taskType == "wOcraFullPostureTaskManager")
            {
                sizeDofConcerned = sizeDof;
            }else if(argStructPtr->taskType == "wOcraPartialPostureTaskManager"){
                sizeDofConcerned = sizeIndexDesired+sizeNameDesired;
            }

            if (sizeDesired == 0)
            {
                if(argStructPtr->taskType == "wOcraFullPostureTaskManager")
                {
                    argStructPtr->desired = model.getJointPositions();
                }else if(argStructPtr->taskType == "wOcraPartialPostureTaskManager"){
                    argStructPtr->desired = Eigen::VectorXd::Zero(sizeDofConcerned);
                }
            }
            else
            {
                if (sizeDesired == 1)
                {
                    argStructPtr->desired = Eigen::VectorXd::Constant(sizeDofConcerned, argStructPtr->desired[0]);
                }
                else if (sizeDesired != sizeDofConcerned)
                {
                    if(argStructPtr->taskType == "wOcraFullPostureTaskManager")
                    {
                        argStructPtr->desired = model.getJointPositions();
                    }else if(argStructPtr->taskType == "wOcraPartialPostureTaskManager"){
                        argStructPtr->desired = Eigen::VectorXd::Zero(sizeDofConcerned);
                    }
                }
            }
            if (argStructPtr->useWeightVectorConstructor)
            {
                if (sizeWeightVector != sizeDofConcerned)
                {
                    argStructPtr->weightVector = Eigen::VectorXd::Constant(sizeDofConcerned, argStructPtr->weight);
                }
            }


            if(argStructPtr->taskType == "wOcraFullPostureTaskManager")
            {
                if (sizeJointIndexes>0 && sizeJointIndexes == sizeIndexDesired)
                {
                    for(int i=0; i<sizeJointIndexes; i++)
                    {
                        if(argStructPtr->indexDesired(i) != -1000.0)
                        {
                            argStructPtr->desired(argStructPtr->jointIndexes(i)) = argStructPtr->indexDesired(i);
                        }
                        if (argStructPtr->useWeightVectorConstructor)
                        {
                            if (argStructPtr->indexWeightVector(i) >= 0.0)
                            {
                                argStructPtr->weightVector(argStructPtr->jointIndexes(i)) = argStructPtr->indexWeightVector(i);
                            }
                        }
                    }
                }
                if (sizeJointNames>0 && sizeJointNames == sizeNameDesired)
                {
                    for(int i=0; i<sizeJointNames; i++)
                    {
                        if(argStructPtr->nameDesired(i) != -1000.0)
                        {
                        argStructPtr->desired(model.getDofIndex(argStructPtr->jointNames[i])) = argStructPtr->nameDesired(i);
                        }
                        if (argStructPtr->useWeightVectorConstructor)
                        {
                            if (argStructPtr->nameWeightVector(i) >= 0.0)
                            {
                                argStructPtr->weightVector(model.getDofIndex(argStructPtr->jointNames[i])) = argStructPtr->nameWeightVector(i);
                            }
                        }
                    }
                }
            }
            else if(argStructPtr->taskType == "wOcraPartialPostureTaskManager")
            {
                Eigen::VectorXi jointIndexesTemp(sizeDofConcerned);
                int indexCounter = 0;

                if (sizeJointIndexes>0 && sizeJointIndexes == sizeIndexDesired)
                {
                    for(int i=0; i<sizeJointIndexes; i++)
                    {
                        jointIndexesTemp(indexCounter) = argStructPtr->jointIndexes(i);
                        if(argStructPtr->indexDesired(i) != -1000.0)
                        {
                            argStructPtr->desired(indexCounter) = argStructPtr->indexDesired(i);
                        }
                        if (argStructPtr->useWeightVectorConstructor)
                        {
                            if (argStructPtr->indexWeightVector(i) >= 0.0)
                            {
                                argStructPtr->weightVector(indexCounter) = argStructPtr->indexWeightVector(i);
                            }
                        }
                        indexCounter++;
                    }
                }
                if (sizeJointNames>0 && sizeJointNames == sizeNameDesired)
                {
                    for(int i=0; i<sizeJointNames; i++)
                    {
                        jointIndexesTemp(indexCounter) = model.getDofIndex(argStructPtr->jointNames[i]);
                        if(argStructPtr->nameDesired(i) == -1000.0)
                        {
                            argStructPtr->desired(indexCounter) = model.getJointPositions()(model.getDofIndex(argStructPtr->jointNames[i]));
                        }
                        else{
                            argStructPtr->desired(indexCounter) = argStructPtr->nameDesired(i);
                        }
                        if (argStructPtr->useWeightVectorConstructor)
                        {
                            if (argStructPtr->nameWeightVector(i) >= 0.0)
                            {
                                argStructPtr->weightVector(indexCounter) = argStructPtr->nameWeightVector(i);
                            }
                        }
                        indexCounter++;
                    }
                }
                argStructPtr->jointIndexes.resize(sizeDofConcerned);
                argStructPtr->jointIndexes = jointIndexesTemp;

            }

        }





    }




    wOcraTaskManagerBase* wOcraTaskParser::constructTaskManager(wOcraController& ctrl, const wOcraModel& model, std::vector<taskManagerArgs>::iterator argStructPtr)
    {




        prepareTaskManagerArguments(model, argStructPtr);

        int sizeDof = model.nbInternalDofs();
        int sizeOffset = argStructPtr->offset[0].rows();
        int sizeJointIndexes = argStructPtr->jointIndexes.rows();
        int sizeDesired = argStructPtr->desired.rows();



        wOcraTaskManagerBase* newTaskManager;

        if(argStructPtr->taskType == "wOcraCoMTaskManager")
        {
            if (sizeDesired == 0)
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraCoMTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->usesYarp);            }
                else {
                    newTaskManager = new wOcraCoMTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->usesYarp);            }
            }
            else
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraCoMTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->desired,
                                                            argStructPtr->usesYarp);            }
                else {
                    newTaskManager = new wOcraCoMTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->desired,
                                                            argStructPtr->usesYarp);            }
            }


            return newTaskManager;
        }

        else if(argStructPtr->taskType == "wOcraContactSetTaskManager")
        {
            newTaskManager = new wOcraContactSetTaskManager(ctrl, model,
                                                    argStructPtr->taskName,
                                                    argStructPtr->segment,
                                                    eigenVectorToDisplacementd(argStructPtr->offset),
                                                    argStructPtr->mu,
                                                    argStructPtr->margin,
                                                    argStructPtr->usesYarp);

            return newTaskManager;
        }

        else if(argStructPtr->taskType == "wOcraContactTaskManager")
        {

            newTaskManager = new wOcraContactTaskManager(ctrl, model,
                                                    argStructPtr->taskName,
                                                    argStructPtr->segment,
                                                    eigenVectorToDisplacementd(argStructPtr->offset.front()),
                                                    argStructPtr->mu,
                                                    argStructPtr->margin,
                                                    argStructPtr->usesYarp);
            return newTaskManager;
        }

        else if(argStructPtr->taskType == "wOcraFullPostureTaskManager")
        {
            if (sizeDesired == 0)
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraFullPostureTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->usesYarp);
                }
                else {
                    newTaskManager = new wOcraFullPostureTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->usesYarp);
                }
            }
            else
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraFullPostureTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->desired,
                                                            argStructPtr->usesYarp);
                }
                else {
                    newTaskManager = new wOcraFullPostureTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            ocra::FullState::INTERNAL,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->desired,
                                                            argStructPtr->usesYarp);
                }
            }




            return newTaskManager;
        }

        else if(argStructPtr->taskType == "wOcraPartialPostureTaskManager")
        {
            if (sizeJointIndexes>0 && sizeJointIndexes<=sizeDof)
            {
                if (sizeDesired==0)
                {
                    if (argStructPtr->useWeightVectorConstructor) {
                        newTaskManager = new wOcraPartialPostureTaskManager(ctrl, model,
                                                                argStructPtr->taskName,
                                                                ocra::FullState::INTERNAL,
                                                                argStructPtr->jointIndexes,
                                                                argStructPtr->kp,
                                                                argStructPtr->kd,
                                                                argStructPtr->weightVector,
                                                                argStructPtr->usesYarp);
                    }
                    else {
                        newTaskManager = new wOcraPartialPostureTaskManager(ctrl, model,
                                                                argStructPtr->taskName,
                                                                ocra::FullState::INTERNAL,
                                                                argStructPtr->jointIndexes,
                                                                argStructPtr->kp,
                                                                argStructPtr->kd,
                                                                argStructPtr->weight,
                                                                argStructPtr->usesYarp);
                    }
                }
                else
                {
                    if (argStructPtr->useWeightVectorConstructor) {
                        newTaskManager = new wOcraPartialPostureTaskManager(ctrl, model,
                                                                argStructPtr->taskName,
                                                                ocra::FullState::INTERNAL,
                                                                argStructPtr->jointIndexes,
                                                                argStructPtr->kp,
                                                                argStructPtr->kd,
                                                                argStructPtr->weightVector,
                                                                argStructPtr->desired,
                                                                argStructPtr->usesYarp);
                    }
                    else {
                        newTaskManager = new wOcraPartialPostureTaskManager(ctrl, model,
                                                                argStructPtr->taskName,
                                                                ocra::FullState::INTERNAL,
                                                                argStructPtr->jointIndexes,
                                                                argStructPtr->kp,
                                                                argStructPtr->kd,
                                                                argStructPtr->weight,
                                                                argStructPtr->desired,
                                                                argStructPtr->usesYarp);
                    }
                }


                return newTaskManager;
            }
            else
                return NULL;
        }

        else if(argStructPtr->taskType == "wOcraSegCartesianTaskManager")
        {

            if(sizeOffset!=3)
            {
                argStructPtr->offset.front() = Eigen::VectorXd::Zero(3);
            }

            if (sizeDesired!=3)
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraSegCartesianTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->offset.front(),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->usesYarp);
                }
                else {
                    newTaskManager = new wOcraSegCartesianTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->offset.front(),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->usesYarp);
                }

                return newTaskManager;
            }


            else if(sizeDesired == 3)
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraSegCartesianTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->offset.front(),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->desired,
                                                            argStructPtr->usesYarp);
                }
                else {
                    newTaskManager = new wOcraSegCartesianTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->offset.front(),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->desired,
                                                            argStructPtr->usesYarp);
                }

                return newTaskManager;
            }


        }

        else if(argStructPtr->taskType == "wOcraSegOrientationTaskManager")
        {

            if (sizeDesired!=4)
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraSegOrientationTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->usesYarp);
                }
                else {
                    newTaskManager = new wOcraSegOrientationTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->usesYarp);
                }

                return newTaskManager;
            }


            else if(sizeDesired == 4)
            {
                // We have to reorganize the quaternion vector when we use this constructor because it stores the coeffs in reverse order.
                double tmpW = argStructPtr->desired(0);
                argStructPtr->desired.head(3) = argStructPtr->desired.tail(3).eval();
                argStructPtr->desired(3) = tmpW;

                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraSegOrientationTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            Eigen::Rotation3d(argStructPtr->desired.data()),
                                                            argStructPtr->usesYarp);
                }
                else {
                    newTaskManager = new wOcraSegOrientationTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            Eigen::Rotation3d(argStructPtr->desired.data()),
                                                            argStructPtr->usesYarp);
                }

                return newTaskManager;
            }
        }

        else if(argStructPtr->taskType == "wOcraSegPoseTaskManager")
        {

            if (sizeDesired!=7)
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraSegPoseTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            eigenVectorToDisplacementd(argStructPtr->offset.front()),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            argStructPtr->usesYarp);
                }
                else {
                    newTaskManager = new wOcraSegPoseTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            eigenVectorToDisplacementd(argStructPtr->offset.front()),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            argStructPtr->usesYarp);
                }

                return newTaskManager;
            }


            else if(sizeDesired == 7)
            {
                if (argStructPtr->useWeightVectorConstructor) {
                    newTaskManager = new wOcraSegPoseTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            eigenVectorToDisplacementd(argStructPtr->offset.front()),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weightVector,
                                                            eigenVectorToDisplacementd(argStructPtr->desired),
                                                            argStructPtr->usesYarp );
                }
                else {
                    newTaskManager = new wOcraSegPoseTaskManager(ctrl, model,
                                                            argStructPtr->taskName,
                                                            argStructPtr->segment,
                                                            eigenVectorToDisplacementd(argStructPtr->offset.front()),
                                                            ocra::ECartesianDof(argStructPtr->axes),//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            eigenVectorToDisplacementd(argStructPtr->desired),
                                                            argStructPtr->usesYarp );
                }


                return newTaskManager;
            }
        }



        else
        {
            std::cout << "[ERROR] (wOcraTaskParser::constructTaskManager): The task type provided --> " << argStructPtr->taskType << " <-- doesn't match any valid taskManager type. Ignoring." << std::endl;
            return NULL;
        }


    }


    Eigen::Displacementd wOcraTaskParser::eigenVectorToDisplacementd(Eigen::VectorXd& eigenVector)
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

    std::vector<Eigen::Displacementd> wOcraTaskParser::eigenVectorToDisplacementd(std::vector<Eigen::VectorXd>& eigenVector)
    {
        std::vector<Eigen::Displacementd> dispVec;
        for(int i=0; i<eigenVector.size(); i++)
        {
            dispVec.push_back(eigenVectorToDisplacementd(eigenVector[i]));
        }
        return dispVec;
    }




}
