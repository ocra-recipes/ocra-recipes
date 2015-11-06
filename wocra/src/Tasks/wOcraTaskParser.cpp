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
                            if (taskElem->QueryDoubleAttribute("kd", &currentTmArgs.kd)==TIXML_NO_ATTRIBUTE){currentTmArgs.kd=0.0;}
                            if (taskElem->QueryDoubleAttribute("weight", &currentTmArgs.weight)==TIXML_NO_ATTRIBUTE){currentTmArgs.weight=0.0; currentTmArgs.useWeightVectorConstructor=false;}else{currentTmArgs.useWeightVectorConstructor=false;}
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
                            std::string jointIndexString, desiredValueString;
                            for(TiXmlElement* jointElem = taskElem->FirstChildElement("joint"); jointElem != NULL; jointElem = jointElem->NextSiblingElement("joint"))
                            {
                                if (jointElem->Attribute("id") != NULL)
                                {
                                    jointIndexString += jointElem->Attribute("id");
                                    jointIndexString += " ";

                                    if (jointElem->Attribute("des") != NULL)
                                    {
                                        desiredValueString += jointElem->Attribute("des");
                                        desiredValueString += " ";
                                    }else{
                                        desiredValueString += "0.0 ";
                                    }
                                }

                                currentTmArgs.jointIndexes = stringToVectorXi(jointIndexString.c_str());
                                currentTmArgs.desired = stringToVectorXd(desiredValueString.c_str());
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
            std::cout << "\n=== Task " << i+1 << " of " << nTasks << " ===" << std::endl;
            std::cout << "Task name: " << tmArgsVector[i].taskName << std::endl;
            std::cout << "Task type: " << tmArgsVector[i].taskType << std::endl;
            std::cout << "Segment: " << tmArgsVector[i].segment << std::endl;
            std::cout << "kp: " << tmArgsVector[i].kp << std::endl;
            std::cout << "kd: " << tmArgsVector[i].kd << std::endl;
            std::cout << "weight: " << tmArgsVector[i].weight << std::endl;
            std::cout << "weights: " << tmArgsVector[i].weightVector << std::endl;
            std::cout << "axes: " << tmArgsVector[i].axes << std::endl;
            std::cout << "offset: " << std::endl;
            for(int j=0; j<tmArgsVector[i].offset.size(); j++){std::cout << tmArgsVector[i].offset[j].transpose() << std::endl;}
            std::cout << "jointIndexes: " << tmArgsVector[i].jointIndexes.transpose() << std::endl;
            std::cout << "desired: " << tmArgsVector[i].desired.transpose() << std::endl;


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










    wOcraTaskManagerBase* wOcraTaskParser::constructTaskManager(wOcraController& ctrl, const wOcraModel& model, std::vector<taskManagerArgs>::iterator argStructPtr)
    {

        int sizeDof = model.nbInternalDofs();
        int sizeDesired = argStructPtr->desired.rows();
        if (argStructPtr->offset.empty()) {
            argStructPtr->offset.push_back(Eigen::VectorXd::Zero(3));
        }
        int sizeOffset = argStructPtr->offset[0].rows();
        int sizeJointIndexes = argStructPtr->jointIndexes.rows();

        wOcraTaskManagerBase* newTaskManager;

        if(argStructPtr->taskType == "wOcraCoMTaskManager")
        {
            if (argStructPtr->useWeightVectorConstructor) {
                newTaskManager = new wOcraCoMTaskManager(ctrl, model,
                                                        argStructPtr->taskName,
                                                        argStructPtr->axes,//ocra::XYZ,
                                                        argStructPtr->kp,
                                                        argStructPtr->kd,
                                                        argStructPtr->weightVector,
                                                        argStructPtr->desired,
                                                        argStructPtr->usesYarp);            }
            else {
                newTaskManager = new wOcraCoMTaskManager(ctrl, model,
                                                        argStructPtr->taskName,
                                                        argStructPtr->axes,//ocra::XYZ,
                                                        argStructPtr->kp,
                                                        argStructPtr->kd,
                                                        argStructPtr->weight,
                                                        argStructPtr->desired,
                                                        argStructPtr->usesYarp);            }

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
            if (
                (sizeDesired == 0)  ||
                ((sizeDesired > 1) && (sizeDesired < sizeDof)) ||
                (sizeDesired > sizeDof)
               )
            {
                argStructPtr->desired = Eigen::VectorXd::Zero(sizeDof);
            }
            else if (sizeDesired == 1)
            {

                argStructPtr->desired = Eigen::VectorXd::Constant(sizeDof, argStructPtr->desired[0]);
            }

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


            return newTaskManager;
        }

        else if(argStructPtr->taskType == "wOcraPartialPostureTaskManager")
        {
            if (sizeJointIndexes>0 && sizeJointIndexes<=sizeDof)
            {
                if (
                    (sizeDesired == 0)  ||
                    ((sizeDesired > 1) && (sizeDesired < sizeJointIndexes)) ||
                    (sizeDesired > sizeJointIndexes)
                   )
                {
                    argStructPtr->desired = Eigen::VectorXd::Zero(sizeJointIndexes);
                }
                else if (sizeDesired == 1)
                {
                    argStructPtr->desired = Eigen::VectorXd::Constant(sizeJointIndexes, argStructPtr->desired[0]);
                }

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

                return newTaskManager;
            }
            else
                return NULL;
        }

        else if(argStructPtr->taskType == "wOcraSegCartesianTaskManager")
        {
            //TODO: convert string axes to ocra::axes

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
                                                            argStructPtr->axes,//ocra::XYZ,
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
                                                            argStructPtr->axes,//ocra::XYZ,
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
                                                            argStructPtr->axes,//ocra::XYZ,
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
                                                            argStructPtr->axes,//ocra::XYZ,
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
                argStructPtr->desired.head(3) = argStructPtr->desired.tail(3);
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
                                                            argStructPtr->axes,//ocra::XYZ,
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
                                                            argStructPtr->axes,//ocra::XYZ,
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
                                                            argStructPtr->axes,//ocra::XYZ,
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
                                                            argStructPtr->axes,//ocra::XYZ,
                                                            argStructPtr->kp,
                                                            argStructPtr->kd,
                                                            argStructPtr->weight,
                                                            eigenVectorToDisplacementd(argStructPtr->desired),
                                                            argStructPtr->usesYarp );
                }


                return newTaskManager;
            }
        }

        // else if(argStructPtr->taskType == "wOcraVariableWeightsTaskManager")
        // {
        //  newTaskManager = new wOcraVariableWeightsTaskManager(ctrl, model,
        //                                          argStructPtr->taskName,
        //                                          argStructPtr->kp,
        //                                          argStructPtr->kd,
        //                                          argStructPtr->weight,
        //                                          argStructPtr->desired,
        //                                          argStructPtr->usesYarp);
        //  return newTaskManager;
        // }

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
