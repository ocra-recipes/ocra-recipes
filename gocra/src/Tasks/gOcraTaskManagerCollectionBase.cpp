#include "gocra/Tasks/gOcraTaskManagerCollectionBase.h"
#include "gocra/GHCJTController.h"

namespace gocra
{
    gOcraTaskManagerCollectionBase::~gOcraTaskManagerCollectionBase()
    {
    }

    void gOcraTaskManagerCollectionBase::init(GHCJTController& ctrl, ocra::Model& model)
    {
        doInit(ctrl, model);
    }

    void gOcraTaskManagerCollectionBase::update(double time, ocra::Model& state, void** args)
    {
        doUpdate(time, state, args);
    }
}
