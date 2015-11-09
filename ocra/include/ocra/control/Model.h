/** @file Model.h
  * @brief Declaration file of the Model class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/08/02
  */

#ifndef _OCRACONTROL_MODEL_H_
#define _OCRACONTROL_MODEL_H_

// includes
#include "ocra/MathTypes.h"
#include "ocra/optim/ObserverSubject.h"
#include "ocra/optim/Variable.h"
#include "ocra/optim/NamedInstance.h"
#include "ocra/control/ModelContacts.h"

#include <Eigen/Lgsm>

#include <string>
#include <iostream>

namespace ocra
{
  /** @class Model
    *	@brief %Model class.
    *	@warning None
    *  
    * TODO: complete description
    *
    * terms of the dynamic equation are given so that the equation writes this way : 
    * M\dot{T} + N + G = L tau - J_c^T f
    */
  class Model : public ObserverSubject, public NamedInstance
  {
  public:
    Model(const std::string& name, int ndofs, bool freeRoot, 
          const std::string& jointTorqueVariableName = "tau",
          const std::string& forceVariableName = "f",
          const std::string& configurationVariableName = "q",
          const std::string& internalDofsSuffix = "_int",
          const std::string& externalDofsSuffix = "_root");
  public:
    virtual ~Model();

    // ------------------------ public interface --------------------------------
  public:
    //set/get configuration
    void  setJointPositions(const Eigen::VectorXd& q);
    void  setJointVelocities(const Eigen::VectorXd& q_dot);
    void  setFreeFlyerPosition(const Eigen::Displacementd& H_root);
    void  setFreeFlyerVelocity(const Eigen::Twistd& T_root);
    void  setState(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot);
    void  setState(const Eigen::Displacementd& H_root, const Eigen::VectorXd& q, const Eigen::Twistd& T_root, const Eigen::VectorXd& q_dot);
    virtual const Eigen::VectorXd& getJointPositions()         const = 0;
    virtual const Eigen::VectorXd& getJointVelocities()        const = 0;
    virtual const Eigen::Displacementd& getFreeFlyerPosition() const = 0;
    virtual const Eigen::Twistd& getFreeFlyerVelocity()        const = 0;

    //get whole body data
      //dofs
    int                       nbDofs()              const;
    int                       nbInternalDofs()      const;
    bool                      hasFixedRoot()        const;
    virtual int               nbSegments()          const = 0;
    virtual const Eigen::VectorXd&   getActuatedDofs()     const = 0;
    virtual const Eigen::VectorXd&   getJointLowerLimits() const = 0;
    virtual const Eigen::VectorXd&   getJointUpperLimits() const = 0;
      //CoM    
    virtual double            getMass()             const = 0;
    virtual const Eigen::Vector3d&   getCoMPosition()      const = 0;
    virtual const Eigen::Vector3d&   getCoMVelocity()      const = 0;
    virtual const Eigen::Vector3d&   getCoMAngularVelocity()      const{ std::cout << "getCoMAngularVelocity() Not implemented" << std::endl; }
    virtual const Eigen::Vector3d&   getCoMJdotQdot()      const = 0;
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobian()      const = 0;
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMAngularJacobian() const{ std::cout << "getCoMAngularVelocity() Not implemented" << std::endl; }
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobianDot()   const = 0;
      //dynamic/static equation terms
    virtual const Eigen::MatrixXd&   getInertiaMatrix()            const = 0;
    virtual const Eigen::MatrixXd&   getInertiaMatrixInverse()     const = 0;
    virtual const Eigen::MatrixXd&   getDampingMatrix()            const = 0;
    virtual const Eigen::VectorXd&   getNonLinearTerms()           const = 0;
    virtual const Eigen::VectorXd&   getLinearTerms()              const = 0;
    virtual const Eigen::VectorXd&   getGravityTerms()             const = 0;
    
    //segment data
    virtual const Eigen::Displacementd&  getSegmentPosition(int index) const = 0;
    virtual const Eigen::Twistd&         getSegmentVelocity(int index) const = 0;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&     getSegmentJacobian(int index) const = 0;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&     getSegmentJdot(int index)     const = 0;
    virtual const Eigen::Twistd&         getSegmentJdotQdot(int index) const = 0;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>& getJointJacobian(int index) const = 0;
    virtual double                       getSegmentMass(int index) const = 0;
    virtual const Eigen::Vector3d&       getSegmentCoM(int index) const = 0;
    virtual const Eigen::Matrix<double, 6, 6>& getSegmentMassMatrix(int index) const = 0;
    virtual const Eigen::Vector3d&       getSegmentMomentsOfInertia(int index) const = 0;
    virtual const Eigen::Rotation3d&     getSegmentInertiaAxes(int index) const = 0;

    void setJointDamping(const Eigen::VectorXd& damping);
    const Eigen::VectorXd& getJointDamping() const;

    //variables
    Variable& getConfigurationVariable()  const;
    Variable& getVelocityVariable()       const;
    Variable& getAccelerationVariable()   const;
    Variable& getJointTorqueVariable()    const;

    Variable& getRootConfigurationVariable()      const;
    Variable& getInternalConfigurationVariable()  const;
    Variable& getRootVelocityVariable()           const;
    Variable& getInternalVelocityVariable()       const;
    Variable& getRootAccelerationVariable()       const;
    Variable& getInternalAccelerationVariable()   const;

    //subModels
    ModelContacts&       getModelContacts() const;

    //utils
    int getSegmentIndex(const std::string& name) const;
    const std::string& getSegmentName(int index) const;

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void  doSetJointPositions(const Eigen::VectorXd& q) = 0;
    virtual void  doSetJointVelocities(const Eigen::VectorXd& q_dot) = 0;
    virtual void  doSetFreeFlyerPosition(const Eigen::Displacementd& H_root) = 0;
    virtual void  doSetFreeFlyerVelocity(const Eigen::Twistd& T_root) = 0;

    virtual int                 doGetSegmentIndex(const std::string& name)  const = 0;
    virtual const std::string&  doGetSegmentName(int index)                 const = 0;

    virtual void doInvalidate() {}

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:
    void invalidate(int timestamp);

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:
    int         _dofs;
    bool        _fixedRoot;
    VectorXd    _jointDamping;
    Variable*   _tau;
    Variable*   _q;
    Variable*   _q_dot;
    Variable*   _q_ddot;


    ModelContacts* _modelContacts;
    // ------------------------ private static members --------------------------
  private:
    static const int   FREE_DOFS = 6;

    // ------------------------ friendship declarations -------------------------
  };
}


#endif //_OCRACONTROL_MODEL_H_

// cmake:sourcegroup=Api
