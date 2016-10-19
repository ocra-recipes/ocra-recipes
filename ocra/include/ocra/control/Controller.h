/*!
\file Controller.h
\brief Controller interface.

Copyright (C) 2010 CEA/DRT/LIST/DTSI/SRCI

\author Evrard Paul
\author Escande Adrien
\date 2010/10/03

File history:
*/

#ifndef _OCRA_CONTROLLER_H_
#define _OCRA_CONTROLLER_H_

#include "ocra/optim/NamedInstance.h"
#include <Eigen/Core>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <vector>
#include <map>
#include <ocra/util/Macros.h>

#include "ocra/control/Controller.h"
#include "ocra/control/Task.h"
#include "ocra/control/TaskYarpInterface.h"
#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"
#include <map>
#include <fstream>
#include <iostream>



namespace ocra
{
  class Task;
  class Feature;
  class PointContactFeature;
  class ContactSet;
  class Model;
}

namespace ocra
{
  //! Interface for controllers.
  /*!
  This class can be derived to implement control laws for a manikin.
  */
  class Controller
    : public NamedInstance
    , boost::noncopyable
  {
  DEFINE_CLASS_POINTER_TYPEDEFS(Controller)
  protected:
    Controller(const std::string& name, Model& model);

  public: // control
    virtual ~Controller() = 0;

    void printInfo(int level, const std::string& filename);

    void setMaxJointTorques(const Eigen::VectorXd& tau_max);
    const Eigen::VectorXd& getMaxJointTorques() const;

    void addTask(std::shared_ptr<Task> task);
    void addTasks(const std::vector<std::shared_ptr<Task>>& tasks);

    void removeTask(const std::string& taskName);
    void removeTasks(const std::vector<std::string> tasks);

    std::vector<std::string> getTaskNames();
    std::string getTaskPortName(const std::string& taskName);
    std::vector<std::string> getTaskPortNames();


    void addContactSet(const ContactSet& contacts);
    std::shared_ptr<Task> getTask(const std::string& name);
    const std::shared_ptr<Task> getTask(const std::string& name) const;
    const std::map<std::string, std::shared_ptr<Task>>& getTasks() const;

    //! Computation of output torques based on the tasks added to the controller.
    /*!
    Calls the method doComputeOutput, which can be overloaded to implement specific control
    laws to realize the added tasks.
    */
    void computeOutput(Eigen::VectorXd& tau);
    const Eigen::VectorXd& computeOutput();

    //@{
    //! Error handling
    enum ErrorFlag
    {
      SUCCESS = 0,
      CRITICAL_ERROR = 1, // Flag for errors that trigger the emergency procedure
      STATIC_EQ_LOSS = 2,
      DYN_EQ_LOSS = 4,
      INSTABILITY = 8 + 1, // Instability is a critical error
      OTHER = 16
    };

    void enableErrorHandling();
    void disableErrorHandling();
    bool isErrorHandlingEnabled() const;
    const std::string& getErrorMessage() const;
    void clearErrorFlag();
    int getErrorFlag() const;
    void setMaxJointTorqueNorm(double maxTau);
    double getMaxJointTorqueNorm() const;
    void  setFixedLinkForOdometry(std::string newFixedLink);
    void  setUseOdometry(bool useOdometry) { this->_useOdometry = useOdometry; }
    void  getFixedLinkForOdometry(std::string& currentFixedLink) { currentFixedLink = this->_fixedLink; }
    void setContactState(int isInLeftSupport, int isInRightSupport) { this->_isInLeftSupport = isInLeftSupport; this->_isInRightSupport = isInRightSupport;}
    void getContactState(int& leftSupport, int& rightSupport);
      
    //@}

  public: // factory
    //@{
    //! Generic task creation
    std::shared_ptr<Task> createTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const;
    std::shared_ptr<Task> createTask(const std::string& name, Feature::Ptr feature) const;
    //@}

    //! Creates a contact task
    /*!
    The parameters describe the friction cone parameters for the force applied by the manikin on the environment.
    */
    std::shared_ptr<Task> createContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const;

  protected:
    const std::vector<std::shared_ptr<Task>>& getActiveTasks() const;
    void setErrorFlag(int eflag);
    void setErrorMessage(const std::string& msg);

  protected:
    virtual void doComputeOutput(Eigen::VectorXd& tau) = 0;
    virtual void doAddTask(std::shared_ptr<Task> task) = 0;
    virtual void doAddContactSet(const ContactSet& contacts) = 0;
    virtual void doSetMaxJointTorques(const Eigen::VectorXd& tauMax); // Does nothing if not overloaded

  protected: // factory
    virtual std::shared_ptr<Task> doCreateTask(const std::string& name, Feature::Ptr feature, Feature::Ptr featureDes) const = 0;
    virtual std::shared_ptr<Task> doCreateTask(const std::string& name, Feature::Ptr feature) const = 0;
    virtual std::shared_ptr<Task> doCreateContactTask(const std::string& name, PointContactFeature::Ptr feature, double mu, double margin) const = 0;

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
    bool _useOdometry;
    std::string _fixedLink;
    int _isInLeftSupport;
    int _isInRightSupport;
  };
}

#endif

// cmake:sourcegroup=Api
