#ifndef wOcraTASKPARSER_H
#define wOcraTASKPARSER_H

#include <iostream>
#include <tinyxml.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <boost/filesystem.hpp>

#include "ocra/control/Model.h"
#include "wocra/wOcraController.h"
#include "wocra/Tasks/wOcraTaskManagerBase.h"
#include "wocra/Tasks/wOcraTaskSequenceBase.h"

#include <Eigen/Dense>

#include <yarp/os/Bottle.h>

namespace wocra
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

    class wOcraTaskParser
    {
        public:
            ~wOcraTaskParser();

            Eigen::VectorXd stringToVectorXd(const char * valueString);
            Eigen::VectorXi stringToVectorXi(const char * valueString);
            bool parseTasksXML(const char * filePath);
            bool parseTasksXML(TiXmlDocument* newTasksFile);
            void printTaskArguments();

            bool addTaskManagersToSequence(wOcraController& ctrl, const ocra::Model& model, wOcraTaskSequenceBase* sequence);

            // bool parseTasksYarp(yarp::os::Bottle* yarpMessage);
            // bool xmlToYarp(const char* filePath, yarp::os::Bottle* yarpMessage);
            // bool yarpToXML(yarp::os::Bottle* yarpMessage, char* filePath);




        private:
            std::vector<taskManagerArgs> tmArgsVector;
            std::vector<taskManagerArgs>::iterator tmArgsIt;

            const char * getDisplacementArgs(TiXmlElement* xmlElem);

            wOcraTaskManagerBase* constructTaskManager(wOcraController& ctrl, const ocra::Model& model, std::vector<taskManagerArgs>::iterator argStructPtr);

            Eigen::Displacementd eigenVectorToDisplacementd(Eigen::VectorXd& eigenVector);

            std::vector<Eigen::Displacementd> eigenVectorToDisplacementd(std::vector<Eigen::VectorXd>& eigenVector);

            void prepareTaskManagerArguments(const ocra::Model& model, std::vector<taskManagerArgs>::iterator argStructPtr);


    };

}
#endif // wOcraTASKPARSER_H
