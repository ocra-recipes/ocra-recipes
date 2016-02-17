#ifndef gOcraTASKMANAGERBASE_H
#define gOcraTASKMANAGERBASE_H

#include "ocra/control/Model.h"
#include "gocra/GHCJTController.h"

#include <Eigen/Dense>

namespace gocra
{

/** \brief Task Manager for the Center of Mass (CoM) task with the gOcra Controllers
 *
 */
class gOcraTaskManagerBase
{
    public:
        gOcraTaskManagerBase(gocra::GHCJTController& ctrl, const ocra::Model& model, const std::string& name);

        virtual void activate() = 0;
        virtual void deactivate() = 0;

        virtual VectorXd getTaskError();
        double getTaskErrorNorm();
 
    protected:
        gocra::GHCJTController&        ctrl;
        const ocra::Model&       model;
        const std::string&              name;
        //gocra::GHCJTTask              task;
};

}

#endif // gOcraTASKMANAGERBASE_H
