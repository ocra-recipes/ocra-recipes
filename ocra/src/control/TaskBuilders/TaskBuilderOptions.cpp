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
    if (bottle.get(i).asInt() == TASK_BUILDER_OPTIONS_BOTTLE)
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
        axes = ECartesianDof(bottle.get(++i).asInt());
        hierarchyLevel = bottle.get(++i).asInt();

        int indexesToSkip;

        desired = util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        indexDesired = util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        nameDesired = util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        weightVector = util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        indexWeightVector = util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;
        nameWeightVector = util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;

        jointIndexes = util::pourBottleIntoEigenVectorXi(util::trimBottle(bottle, i+1), indexesToSkip); i += indexesToSkip;

        int numberOfJointNames = bottle.get(++i).asInt();
        for (auto j = 0; j < numberOfJointNames; ++j) {
            jointNames.push_back(bottle.get(++i).asString());
        }


        int numberOfOffsets = bottle.get(++i).asInt();
        for (auto j = 0; j < numberOfOffsets; ++j) {
            offset.push_back(util::pourBottleIntoEigenVector(util::trimBottle(bottle, i+1), indexesToSkip));
            i += indexesToSkip;
        }
        sizeOfOptions = i;
        return true;
    }
    return false;

}

void TaskBuilderOptions::putIntoBottle(yarp::os::Bottle& bottle)
{
    bottle.addInt(TASK_BUILDER_OPTIONS_BOTTLE);

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

    util::pourEigenVectorIntoBottle(desired, bottle);
    util::pourEigenVectorIntoBottle(indexDesired, bottle);
    util::pourEigenVectorIntoBottle(nameDesired, bottle);
    util::pourEigenVectorIntoBottle(weightVector, bottle);
    util::pourEigenVectorIntoBottle(indexWeightVector, bottle);
    util::pourEigenVectorIntoBottle(nameWeightVector, bottle);

    util::pourEigenVectorXiIntoBottle(jointIndexes, bottle);

    bottle.addInt(jointNames.size());
    for(auto name : jointNames){
        bottle.addString(name);
    }

    bottle.addInt(offset.size());
    for(auto vec : offset) {
        util::pourEigenVectorIntoBottle(vec, bottle);
    }
}
