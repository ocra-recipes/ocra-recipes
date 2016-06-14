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

            if ( (xmlTask->Attribute("name") != NULL) && (xmlTask->Attribute("type") != NULL) ) {
                TaskBuilderOptions options;

                options.taskName = xmlTask->Attribute("name");
                options.taskType = xmlTask->Attribute("type");


                std::vector<int> intVector;

                for(TiXmlElement* taskElem = xmlTask->FirstChildElement(); taskElem != NULL; taskElem = taskElem->NextSiblingElement()) {
                    std::string currentElem = taskElem->Value();
                    if( currentElem == "params" ) {
                        if (taskElem->QueryDoubleAttribute("kp", &options.kp)==TIXML_NO_ATTRIBUTE){options.kp=0.0;}
                        if (taskElem->QueryDoubleAttribute("kd", &options.kd)==TIXML_NO_ATTRIBUTE)
                        {
                            if (options.kp > 0.0) {
                                options.kd=2.0*sqrt(options.kp);
                            }else{options.kd=0.0;}
                        }
                        if (taskElem->QueryDoubleAttribute("weight", &options.weight)==TIXML_NO_ATTRIBUTE){options.weight=0.0; options.useWeightVectorConstructor=false;}else{options.useWeightVectorConstructor=false;}
                        if (taskElem->QueryIntAttribute("hierarchyLevel", &options.hierarchyLevel)==TIXML_NO_ATTRIBUTE){options.hierarchyLevel=-1;}

                       //if (taskElem->QueryIntAttribute("axes", &options.axes)==TIXML_NO_ATTRIBUTE){options.axes=ocra::XYZ;}
                        if (taskElem->QueryDoubleAttribute("mu", &options.mu)==TIXML_NO_ATTRIBUTE){options.mu=1.0;}
                        if (taskElem->QueryDoubleAttribute("margin", &options.margin)==TIXML_NO_ATTRIBUTE){options.margin=0.05;}
                        options.axes=ocra::XYZ;
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
                            options.usesYarp = yarpBool;
                        }else{options.usesYarp=true;}
                    }

                    else if( currentElem == "offset" ){

                        if (taskElem->GetText() != NULL){
                            options.offset.push_back(stringToVectorXd(taskElem->GetText()));
                        }
                        else{
                            options.offset.push_back(stringToVectorXd(getDisplacementArgs(taskElem).c_str()));
                        }
                    }

                    else if( currentElem == "desired" ){

                        if (taskElem->GetText() != NULL)
                            options.desired = stringToVectorXd(taskElem->GetText());

                        else
                            options.desired = stringToVectorXd(getDisplacementArgs(taskElem).c_str());
                    }

                    else if( currentElem == "segment" ){
                        options.segment = taskElem->GetText();
                    }

                    else if( currentElem == "jointIndexes" ){
                        options.jointIndexes = stringToVectorXi(taskElem->GetText());
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

                            options.jointIndexes = stringToVectorXi(jointIndexString.c_str());
                            options.indexDesired = stringToVectorXd(indexDesiredValueString.c_str());
                            options.nameDesired = stringToVectorXd(nameDesiredValueString.c_str());
                            options.indexWeightVector = stringToVectorXd(indexWeightString.c_str());
                            options.nameWeightVector = stringToVectorXd(nameWeightString.c_str());
                        }

                    }

                    else if( currentElem == "weights" ){
                        options.useWeightVectorConstructor = true;
                        options.weightVector = stringToVectorXd(taskElem->GetText());
                    }


                    else{
                        std::cout << "[WARNING] (parseTaskOptionsFromXml): Found a task manager tag --> "<< currentElem <<" <-- that doesn't match any known tags. Ignoring." << std::endl;
                    }

                }
                optionsVector.push_back(options);
            }
        }
    }
    return optionsVector;
}

Eigen::VectorXd TaskConstructionManager::stringToVectorXd(const char * valueString)
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


Eigen::VectorXi TaskConstructionManager::stringToVectorXi(const char * valueString)
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


std::string TaskConstructionManager::getDisplacementArgs(TiXmlElement* xmlElem)
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

        return dispString;
    }
}


Eigen::Displacementd TaskConstructionManager::eigenVectorToDisplacementd(Eigen::VectorXd& eigenVector)
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

std::vector<Eigen::Displacementd> TaskConstructionManager::eigenVectorToDisplacementd(std::vector<Eigen::VectorXd>& eigenVector)
{
    std::vector<Eigen::Displacementd> dispVec;
    for(int i=0; i<eigenVector.size(); i++)
    {
        dispVec.push_back(eigenVectorToDisplacementd(eigenVector[i]));
    }
    return dispVec;
}
