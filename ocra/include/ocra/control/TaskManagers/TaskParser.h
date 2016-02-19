#ifndef TASKPARSER_H
#define TASKPARSER_H

#include <iostream>
#include <tinyxml.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <boost/filesystem.hpp>

#include "ocra/control/Model.h"
#include "ocra/control/Controller.h"
#include "ocra/control/TaskManagers/TaskManager.h"
#include "ocra/control/TaskManagers/TaskSequence.h"

#include <Eigen/Dense>

#include <yarp/os/Bottle.h>

namespace ocra
{
    typedef struct
    {
        std::string taskName, taskType, segment;
        double kp, kd, weight, mu, margin;
        bool usesYarp, useWeightVectorConstructor;
        int axes;
        Eigen::VectorXd desired;
        Eigen::VectorXd indexDesired;
        Eigen::VectorXd nameDesired;
        Eigen::VectorXd weightVector;
        Eigen::VectorXd indexWeightVector;
        Eigen::VectorXd nameWeightVector;
        Eigen::VectorXi jointIndexes;
        std::vector<std::string> jointNames;
        std::vector<Eigen::VectorXd> offset;


    }taskManagerArgs;

    class TaskParser
    {
        public:
            ~TaskParser();

            Eigen::VectorXd stringToVectorXd(const char * valueString);
            Eigen::VectorXi stringToVectorXi(const char * valueString);
            bool parseTasksXML(const char * filePath);
            bool parseTasksXML(TiXmlDocument* newTasksFile);
            void printTaskArguments();

            bool addTaskManagersToSequence(ocra::Controller& ctrl, const ocra::Model& model, TaskSequence* sequence);

            // bool parseTasksYarp(yarp::os::Bottle* yarpMessage);
            // bool xmlToYarp(const char* filePath, yarp::os::Bottle* yarpMessage);
            // bool yarpToXML(yarp::os::Bottle* yarpMessage, char* filePath);




        private:
            std::vector<taskManagerArgs> tmArgsVector;
            std::vector<taskManagerArgs>::iterator tmArgsIt;

            const char * getDisplacementArgs(TiXmlElement* xmlElem);

            TaskManager* constructTaskManager(ocra::Controller& ctrl, const ocra::Model& model, std::vector<taskManagerArgs>::iterator argStructPtr);

            Eigen::Displacementd eigenVectorToDisplacementd(Eigen::VectorXd& eigenVector);

            std::vector<Eigen::Displacementd> eigenVectorToDisplacementd(std::vector<Eigen::VectorXd>& eigenVector);

            void prepareTaskManagerArguments(const ocra::Model& model, std::vector<taskManagerArgs>::iterator argStructPtr);


    };

}
#endif // TASKPARSER_H
