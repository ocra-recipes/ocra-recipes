
#include <iostream>
#include <map>

#include <Eigen/Eigen>

#include "Model3T.h"
#include "gocra/OrthonormalFamily.h"

#include "gocra/GHCJTController.h"
#include "gocra/Solvers/OneLevelSolver.h"
#include "ocra/control/Feature.h"
#include "ocra/control/FullState.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/control/ControlEnum.h"




int main(int argc, char** argv)
{
    std::cout<<"SET PARAMETERS\n";

    bool useGSHC = true;
    bool useGrav = true;

    gocra::OneLevelSolverWithQuadProg   internalSolver;
    
    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER\n";
    Model3T                         model("Model3T");
    gocra::GHCJTController         ctrl("myCtrl", model, internalSolver, useGrav);


    //CREATE SOME TASKS
    std::cout<<"CREATE SOME TASKS\n";
    ocra::FullModelState   FMS("fullTask.FModelState" , model, ocra::FullState::INTERNAL); //INTERNAL, FREE_FLYER
    ocra::FullTargetState  FTS("fullTask.FTargetState", model, ocra::FullState::INTERNAL);
    ocra::FullStateFeature feat("fullTask.feat", FMS);
    ocra::FullStateFeature featDes("fullTask.featDes", FTS);
    FTS.set_q(Eigen::VectorXd::Zero(model.nbInternalDofs()));

    gocra::GHCJTTask& accTask = ctrl.createGHCJTTask("fullTask", feat, featDes);

    ctrl.addTask(accTask);
    accTask.activateAsObjective();
    accTask.setStiffness(9);
    accTask.setDamping(6);
    accTask.setWeight(1.);

    ocra::SegmentFrame        SF("frameTask.SFrame", model, "Model3T.segment_3", Eigen::Displacementd());
    ocra::TargetFrame         TF("frameTask.TFrame", model);
    ocra::PositionFeature feat2("frameTask.feat", SF, ocra::XYZ);
    ocra::PositionFeature featDes2("frameTask.featDes", TF, ocra::XYZ);

    TF.setPosition(Eigen::Displacementd(0.2,0.3,0.4));
    TF.setVelocity(Eigen::Twistd());
    TF.setAcceleration(Eigen::Twistd());


    gocra::GHCJTTask& accTask2 = ctrl.createGHCJTTask("frameTask", feat2, featDes2);

    ctrl.addTask(accTask2);
    accTask2.activateAsObjective();
    accTask2.setStiffness(9);
    accTask2.setDamping(6);
    accTask2.setWeight(1);

    ctrl.setActiveTaskVector();

    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
    int nt = ctrl.getNbActiveTask();

    MatrixXd param_priority(nt,nt);
//        std::vector< gocra::GHCJTTask* >& activeTask = ctrl.getActiveTask();

    param_priority<<0,1,0,0;
    ctrl.setTaskProjectors(param_priority);


    //SIMULATE
    VectorXd q      = Eigen::VectorXd::Constant(model.nbInternalDofs(), 0.1);
    VectorXd dq     = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd tau    = Eigen::VectorXd::Zero(model.nbInternalDofs());
    VectorXd ddq;
    double dt = 0.01;

    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        if (i==600)
        {
            param_priority(0,1)=0;
            param_priority(1,0)=1;
        }
        ctrl.setTaskProjectors(param_priority);
        ctrl.doUpdateProjector();
        ctrl.computeOutput(tau);
        ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getLinearTerms() - model.getGravityTerms());

        dq += ddq * dt;
        q  += dq  * dt;



        model.setJointPositions(q);
        model.setJointVelocities(dq);
        if (i==1||i==599||i==999)
        {
            std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";

            std::cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";

        }

    }


    return 0;
}



