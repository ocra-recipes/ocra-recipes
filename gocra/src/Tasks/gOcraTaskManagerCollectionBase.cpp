#include "gocra/Tasks/gOcraTaskManagerCollectionBase.h"
#include "gocra/GHCJTController.h"
#include "gocra/Model.h"

namespace gocra
{
    gOcraTaskManagerCollectionBase::~gOcraTaskManagerCollectionBase()
    {
    }

    void gOcraTaskManagerCollectionBase::init(GHCJTController& ctrl, gocra::gOcraModel& model)
    {
        doInit(ctrl, model);
    }

    void gOcraTaskManagerCollectionBase::update(double time, gocra::gOcraModel& state, void** args)
    {
        doUpdate(time, state, args);
    }
}
