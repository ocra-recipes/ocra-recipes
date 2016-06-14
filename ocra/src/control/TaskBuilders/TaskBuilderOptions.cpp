#include <ocra/control/TaskBuilders/TaskBuilderOptions.h>

using namespace ocra;

TaskBuilderOptions::TaskBuilderOptions()
{
}

TaskBuilderOptions::~TaskBuilderOptions()
{
}

bool TaskBuilderOptions::extractFromBottle(yarp::os::Bottle& bottle, int& sizeOfOptions)
{
    int i = 0;
    if (bottle.get(i).asInt() == TASK_MANAGER_OPTIONS_BOTTLE)
    {
        taskName    = bottle.get(++i).asString();
        taskType    = bottle.get(++i).asString();
        segment     = bottle.get(++i).asString();

        kp          = bottle.get(++i).asDouble();
        kd          = bottle.get(++i).asDouble();
        weight      = bottle.get(++i).asDouble();
        mu          = bottle.get(++i).asDouble();
        margin      = bottle.get(++i).asDouble();

        usesYarp = bottle.get(++i).asBool();;
        useWeightVectorConstructor = bottle.get(++i).asBool();
        axes = bottle.get(++i).asInt();
        hierarchyLevel = bottle.get(++i).asInt();

        int indexesToSkip;

        desired = pourBottleIntoEigenVector(trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        indexDesired = pourBottleIntoEigenVector(trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        nameDesired = pourBottleIntoEigenVector(trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        weightVector = pourBottleIntoEigenVector(trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        indexWeightVector = pourBottleIntoEigenVector(trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        nameWeightVector = pourBottleIntoEigenVector(trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;

        jointIndexes = pourBottleIntoEigenVectorXi(trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;

        int numberOfJointNames = bottle.get(++i).asInt();
        for (auto j = 0; j < numberOfJointNames; ++j) {
            jointNames.push_back(bottle.get(++i).asString());
        }


        int numberOfOffsets = bottle.get(++i).asInt();
        for (auto j = 0; j < numberOfOffsets; ++j) {
            offset.push_back(pourBottleIntoEigenVector(trimBottle(bottle, i+1), indexesToSkip));
            i += indexesToSkip;
        }
        sizeOfOptions = i;
        return true;
    }
    return false;

}

void TaskBuilderOptions::putIntoBottle(yarp::os::Bottle& bottle)
{
    bottle.addInt(TASK_MANAGER_OPTIONS_BOTTLE);

    bottle.addString(taskName);
    bottle.addString(taskType);
    bottle.addString(segment);

    bottle.addDouble(kp);
    bottle.addDouble(kd);
    bottle.addDouble(weight);
    bottle.addDouble(mu);
    bottle.addDouble(margin);

    bottle.addInt(usesYarp);
    bottle.addInt(useWeightVectorConstructor);
    bottle.addInt(axes);
    bottle.addInt(hierarchyLevel);

    pourEigenVectorIntoBottle(desired, bottle);
    pourEigenVectorIntoBottle(indexDesired, bottle);
    pourEigenVectorIntoBottle(nameDesired, bottle);
    pourEigenVectorIntoBottle(weightVector, bottle);
    pourEigenVectorIntoBottle(indexWeightVector, bottle);
    pourEigenVectorIntoBottle(nameWeightVector, bottle);

    pourEigenVectorXiIntoBottle(jointIndexes, bottle);

    bottle.addInt(jointNames.size());
    for(auto name : jointNames){
        bottle.addString(name);
    }

    bottle.addInt(offset.size());
    for(auto vec : offset) {
        pourEigenVectorIntoBottle(vec, bottle);
    }
}
