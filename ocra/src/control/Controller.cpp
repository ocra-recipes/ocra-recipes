#include "Controller.h"
#include "ocra/control/Task.h"
#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"
#include <map>
#include <fstream>
#include <iostream>

namespace
{
  template<class T>
  class checked_map
  {
  private:
    std::map<std::string, T*> data;
    const std::string id;

  public:
    checked_map(const std::string& id_): id(id_) {}

    const T& get(const std::string& name) const
    {
      typename std::map<std::string, T*>::const_iterator it = data.find(name);
      if(it == data.end())
        throw std::runtime_error("[Controller::"+id+" set]: element with name "+name+" not found!");
      return *it->second;
    }

    T& get(const std::string& name)
    {
      typename std::map<std::string, T*>::iterator it = data.find(name);
      if(it == data.end())
        throw std::runtime_error("[Controller::"+id+" set]: element with name "+name+" not found!");
      return *it->second;
    }

    void add(const std::string& name, T& elm)
    {
      typename std::map<std::string, T*>::const_iterator it = data.find(name);
      if(it != data.end())
        throw std::runtime_error("[Controller::"+id+" set]: element with name "+name+" already registered!");
      data[name] = &elm;
    }

    const std::map<std::string, T*>& getData() const
    {
      return data;
    }

    template<class P>
    void getIf(P predicate, std::vector<T*>& result) const
    {
      typename std::map<std::string, T*>::const_iterator it = data.begin();
      for(; it != data.end(); ++it)
      {
        if(predicate(it->second))
          result.push_back(it->second);
      }
    }
  };


  struct isTaskActive
  {
    bool operator()(const ocra::Task* task)
    {
      return (task->isActiveAsObjective() ||task->isActiveAsConstraint());
    }
  };
}

namespace ocra
{
  struct Controller::Pimpl
  {
    const Model& model;
    VectorXd tau_max;
    VectorXd tau;
    checked_map<Task> tasks;
    std::vector<Task*> activeTasks;
    std::string errorMessage;
    double maxTau;
    int errorFlag;
    bool handleError;

    Pimpl(const Model& m)
      : model(m)
      , tau_max( VectorXd::Constant(m.nbInternalDofs(), std::numeric_limits<double>::max()) )
      , tau( VectorXd::Constant(m.nbInternalDofs(), 0.) )
      , tasks("tasks")
      , activeTasks()
      , errorMessage("")
      , maxTau(500.)
      , errorFlag(Controller::SUCCESS)
      , handleError(false)
    {}
  };

  Controller::Controller(const std::string& name, Model& model)
    : NamedInstance(name)
    , pimpl(new Pimpl(model))
  {
  }

  Controller::~Controller()
  {
  }

  void Controller::printInfo(int level, const std::string& filename)
  {
    std::ofstream file;
    if(!filename.empty())
      file.open(filename.c_str(), std::ios::app);
    std::ostream& os = filename.empty() ? std::cout : file;

    os << "Controller Info {\nController name: " << getName() << std::endl;

    if(level > 1)
    {
      os << "\tError flag: " << getErrorFlag() << std::endl;
      os << "\tError message: " << getErrorMessage() << std::endl;
    }

    const std::vector<Task*>& activeTasks = getActiveTasks();

    if(activeTasks.empty())
      os << "\tNo active task" << std::endl;
    else
      os << "\tList of active tasks:" << std::endl;

    for(size_t i = 0; i < activeTasks.size(); ++i)
    {
      Task& t = *activeTasks[i];
      os << "\t- " << t.getName() << ":" << std::endl;
      if(level > 1)
      {
        os << "\t\tLast output: " << t.getOutput().transpose() << std::endl;
        os << "\t\tError: " << t.getError().transpose() << std::endl;
        os << "\t\tErrorDot: " << t.getErrorDot().transpose() << std::endl;
        os << "\t\tJacobian\n" << t.getJacobian() << std::endl;
      }
      if(level > 0)
      {
        os << "\t\tWeight: " << t.getWeight().transpose() << std::endl;
        os << "\t\tDesired mass\n" << t.getDesiredMass() << std::endl;
        os << "\t\tApparent mass changed: " << !t.isDesiredMassTheActualOne() << std::endl;
        os << "\t\tDamping\n" << t.getDamping() << std::endl;
        os << "\t\tStiffness\n" << t.getStiffness() << std::endl;
        os << "\t\tRef force: " << t.getEffort().transpose() << std::endl;
        os << "\t\tContact task: " << t.isContactModeActive() << std::endl;
        if(t.isContactModeActive())
        {
          os << "\t\t\tmu: " << t.getFrictionCoeff() << std::endl;
          os << "\t\t\tmargin: " << t.getMargin() << std::endl;
        }
      }
    }

    os << "}" << std::endl;
  }

  void Controller::setMaxJointTorques(const VectorXd& tau_max)
  {
    pimpl->tau_max = tau_max;
    doSetMaxJointTorques(tau_max);
  };
  void Controller::doSetMaxJointTorques(const VectorXd& tau_max) {}

  const VectorXd& Controller::getMaxJointTorques() const
  {
    return pimpl->tau_max;
  }

  void Controller::addTask(Task& task)
  {
    pimpl->tasks.add(task.getName(), task);
    doAddTask(task);
  }

  void Controller::addTasks(const std::vector<Task*>& tasks)
  {
    for(size_t i = 0; i < tasks.size(); ++i)
      addTask(*tasks[i]);
  }

  void Controller::removeTask(const std::string taskName)
  {
    pimpl->tasks.getData().erase(taskName);
  }

  void Controller::removeTasks(const std::vector<std::string> tasks)
  {
    for(size_t i = 0; i < tasks.size(); ++i)
      removeTask(tasks[i]);
  }

  void Controller::addContactSet(const ContactSet& contacts)
  {
    doAddContactSet(contacts);
  }

  Task& Controller::getTask(const std::string& name)
  {
    return pimpl->tasks.get(name);
  }

  const Task& Controller::getTask(const std::string& name) const
  {
    return pimpl->tasks.get(name);
  }

  const Eigen::VectorXd& Controller::computeOutput()
  {
    computeOutput(pimpl->tau);
    return pimpl->tau;
  }

  void Controller::computeOutput(Eigen::VectorXd& tau)
  {
    if(pimpl->errorFlag & CRITICAL_ERROR)
    {
      if(pimpl->handleError)
      {
        tau.resize(pimpl->model.nbInternalDofs());
        tau.setZero();
        return;
      }
    }

    doComputeOutput(tau);

    if(pimpl->errorFlag & CRITICAL_ERROR)
    {
      if(pimpl->handleError)
      {
        tau.resize(pimpl->model.nbInternalDofs());
        tau.setZero();
        return;
      }
    }

    const double tauNorm = tau.norm();
    if(tauNorm > pimpl->maxTau)
    {
      if(pimpl->handleError)
      {
        tau.resize(pimpl->model.nbInternalDofs());
        tau.setZero();
      }

      std::stringstream ss;
      ss << "Instability suspected: norm of joint torques is " << tauNorm << " Nm.\n"
        << "Maximum authorized is " << pimpl->maxTau << " Nm." << std::endl;

      setErrorMessage(ss.str());
      setErrorFlag(INSTABILITY);
    }
  }

  void Controller::enableErrorHandling()
  {
    pimpl->handleError = true;
  }

  void Controller::disableErrorHandling()
  {
    pimpl->handleError = false;
  }

  bool Controller::isErrorHandlingEnabled() const
  {
    return pimpl->handleError;
  }

  const std::string& Controller::getErrorMessage() const
  {
    return pimpl->errorMessage;
  }

  void Controller::clearErrorFlag()
  {
    pimpl->errorFlag = SUCCESS;
    pimpl->errorMessage = "";
  }

  int Controller::getErrorFlag() const
  {
    return pimpl->errorFlag;
  }

  void Controller::setMaxJointTorqueNorm(double maxTau)
  {
    pimpl->maxTau = maxTau;
  }

  double Controller::getMaxJointTorqueNorm() const
  {
    return pimpl->maxTau;
  }

  Task& Controller::createTask(const std::string& name, const Feature& feature, const Feature& featureDes) const
  {
    Task* task = doCreateTask(name, feature, featureDes);
    task->setDesiredMassToActualOne();
    task->setDamping(0.);
    task->setStiffness(0.);
    task->setWeight(1.);
    return *task;
  }

  Task& Controller::createTask(const std::string& name, const Feature& feature) const
  {
    Task* task = doCreateTask(name, feature);
    task->setDesiredMassToActualOne();
    task->setDamping(0.);
    task->setStiffness(0.);
    task->setWeight(1.);
    return *task;
  }

  Task& Controller::createContactTask(const std::string& name, const PointContactFeature& feature, double mu, double margin) const
  {
    Task* task = doCreateContactTask(name, feature, mu, margin);
    task->setDesiredMassToActualOne();
    task->setDamping(0.);
    task->setStiffness(0.);
    task->setFrictionCoeff(mu);
    task->setMargin(margin);
    task->setWeight(1.);
    task->activateContactMode();
    return *task;
  }

  const std::vector<Task*>& Controller::getActiveTasks() const
  {
    pimpl->activeTasks.clear();
    pimpl->tasks.getIf(isTaskActive(), pimpl->activeTasks);
    return pimpl->activeTasks;
  }

  void Controller::setErrorFlag(int eflag)
  {
    pimpl->errorFlag = eflag;
  }

  void Controller::setErrorMessage(const std::string& msg)
  {
    pimpl->errorMessage = msg;
  }

  const std::map<std::string, Task*>& Controller::getTasks() const
  {
    return pimpl->tasks.getData();
  }
}

// cmake:sourcegroup=Api
