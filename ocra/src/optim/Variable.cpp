#include "Variable.h"
#include "ocra/optim/ocra_assert.h"

#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <iterator>


namespace
{
  struct get_address
  {
    const double* operator()(const double& element) const { return &element; }
  };

  struct get_value
  {
    double operator()(const double* element) const { return *element; }
  };
}

namespace ocra
{
  VariableParenthood::VariableParenthood() throw()
    : startIndexInParentMap(0)
  {
  }


  // ------------------------------------------------------------
  // --- VARIABLE -----------------------------------------------
  // ------------------------------------------------------------

  Variable::Variable(const std::string& name)
    : memoryMap_()
    , name_(name)
    , timeDerivative_(0x0, false)
    , timePrimitive_(0x0, false)
    , value_()
    , updateValue_(true)
  {
  }

  Variable::~Variable()
  {
    if(timePrimitive_.var_ && timePrimitive_.owns_)
    {
      delete timePrimitive_.var_;
      timePrimitive_ = VariablePtr(0x0, false);
    }
    else if(timePrimitive_.var_)
      timePrimitive_.var_->timeDerivative_ = VariablePtr(0x0, false);

    if(timeDerivative_.var_ && timeDerivative_.owns_)
    {
      delete timeDerivative_.var_;
      timeDerivative_ = VariablePtr(0x0, false);
    }
    else if(timeDerivative_.var_)
      timeDerivative_.var_->timePrimitive_ = VariablePtr(0x0, false);
  }

  double Variable::operator[](size_t i) const
  {
    return *memoryMap_[i];
  }

  double Variable::at(size_t i) const
  {
    if (i >= memoryMap_.size())
    {
      std::stringstream err;
      err << "[ocra::Variable::at] index " << i << " out of bound in variable " << name_ << "; size is " << memoryMap_.size();
      throw std::runtime_error(err.str());
    }
    return *memoryMap_[i];
  }

  int Variable::getSize() const
  {
    ocra_assert(static_cast<int>(memoryMap_.size()) >= 0 && "This is really bad, we have such a huge memory block that an int can't hold its size");
    return static_cast<int>(memoryMap_.size());
  }

  Variable::operator const VectorXd& () const
  {
    if(updateValue_)
      updateValue();
    return value_;
  }

  const VectorXd& Variable::getValue() const
  {
    return *this;
  }

  void Variable::setValue(const VectorXd& value)
  {
    do_setValue(value);
    updateValue_ = true;
    propagate<EVT_CHANGE_VALUE>();
  }

  Variable& Variable::getTimeDerivative()
  {
    if(!timeDerivative_)
      createTimeDerivative(name_ + "_dot");
    return *timeDerivative_.var_;
  }

  Variable& Variable::getTimePrimitive()
  {
    if(!timePrimitive_)
      createTimePrimitive("int_" + name_);
    return *timePrimitive_.var_;
  }

  bool Variable::hasTimeDerivative() const
  {
    return timeDerivative_;
  }

  bool Variable::hasTimePrimitive() const
  {
    return timePrimitive_;
  }

  void Variable::createTimeDerivative(const std::string& name)
  {
    if(timeDerivative_)
    {
      std::stringstream err;
      err << "[ocra::Variable::createTimeDerivative] This variable already has a derivative, named " << timeDerivative_.var_->name_;
      throw std::runtime_error(err.str());
    }

    timeDerivative_.var_ = do_createTimeDerivative(name);
    timeDerivative_.owns_ = true;
    ocra_assert(timeDerivative_ && "The derived class violated its contract: do_createTimeDerivative must return non null pointer!");

    timeDerivative_.var_->timePrimitive_.var_ = this;
    timeDerivative_.var_->timePrimitive_.owns_ = false;
  }

  void Variable::createTimePrimitive(const std::string& name)
  {
    if(timePrimitive_)
    {
      std::stringstream err;
      err << "[ocra::Variable::createTimePrimitive] This variable already has a primitive, named " << timePrimitive_.var_->name_;
      throw std::runtime_error(err.str());
    }

    timePrimitive_.var_ = do_createTimePrimitive(name);
    timePrimitive_.owns_ = true;
    ocra_assert(timePrimitive_ && "The derived class violated its contract: do_createTimePrimitive must return non null pointer!");

    timePrimitive_.var_->timeDerivative_.var_ = this;
    timePrimitive_.var_->timeDerivative_.owns_ = false;
  }

  const std::string& Variable::getName() const
  {
    return name_;
  }

  bool Variable::isBaseVariable() const
  {
    return false;
  }


  void Variable::getRelativeMappingOf(const Variable& subVariable, std::vector<int>& mapping) const
  {
    // we need to remove the const for performing the 'trick' to obtain the permutation
    // yet we must undo all modifications done on subVariable before the end of the function
    // having a const subVariable is the intended behavior!
    Variable& subVar = const_cast<Variable&>(subVariable);
    Variable& thisVar = const_cast<Variable&>(*this);

    // we create storage to save the current values for further restoration
    std::vector<double> subVar_backup, thisVar_backup;
    std::transform(subVar.memoryMap_.begin(), subVar.memoryMap_.end(), std::back_inserter(subVar_backup), get_value());
    std::transform(thisVar.memoryMap_.begin(), thisVar.memoryMap_.end(), std::back_inserter(thisVar_backup), get_value());

    try {
      double** subVarMemoryMap = const_cast<double**>(&subVar.memoryMap_[0]);
      double** thisVarMemoryMap = const_cast<double**>(&thisVar.memoryMap_[0]);

      for(int i = 0; i < subVar.getSize(); ++i)
        *subVarMemoryMap[i] = -1;

      for(int i = 0; i < thisVar.getSize(); ++i)
        *thisVarMemoryMap[i] = static_cast<double>(i);

      mapping.clear();
      mapping.reserve(subVar.getSize());
      for(int i = 0; i < subVar.getSize(); ++i)
      {
        int d = static_cast<int>(subVar[i]);
        if(d < 0)
        {
          mapping.clear();
          break;
        }
        mapping.push_back(d);
      }
    }
    catch(...) {}// whatever happens, we have to restore and thus always execute the end of the method! We said 'const'...

    double** subVarMemoryMap = const_cast<double**>(&subVar.memoryMap_[0]);
    double** thisVarMemoryMap = const_cast<double**>(&thisVar.memoryMap_[0]);

    for(int i = 0; i < subVar.getSize(); ++i)
      *subVarMemoryMap[i] = subVar_backup[i];

    for(int i = 0; i < thisVar.getSize(); ++i)
      *thisVarMemoryMap[i] = thisVar_backup[i];
  }

  size_t Variable::getNumberOfChildren() const
  {
    return do_getNumberOfChildren();
  }

  void Variable::onAttachedParent(const parenthood_t& parent)
  {
    connect<EVT_CHANGE_VALUE>(parent.getParent(), &CompositeVariable::onChildValueChanged);
    connect<EVT_RESIZE>(parent.getParent(), &CompositeVariable::onChildValueChanged);
    connect<EVT_CHANGE_DEPENDENCIES>(parent.getParent(), &CompositeVariable::onChildValueChanged);

    const parenthood_t* prevChild = parent.getPreviousChild();
    if(!prevChild)
      return;

    parent.getInfo().startIndexInParentMap =
      prevChild->getInfo().startIndexInParentMap + prevChild->getChild().memoryMap_.size();
  }

  void Variable::onDetachedParent(const parenthood_t& parent)
  {
    disconnect<EVT_CHANGE_VALUE>(parent.getParent(), &CompositeVariable::onChildValueChanged);
    disconnect<EVT_RESIZE>(parent.getParent(), &CompositeVariable::onChildValueChanged);
    disconnect<EVT_CHANGE_DEPENDENCIES>(parent.getParent(), &CompositeVariable::onChildValueChanged);

    parent.getInfo().startIndexInParentMap = 0;
  }

  void Variable::printNode(int depth, std::ostream& os) const
  {
    static const int shiftPerDepthLevel = 3;
    const int numSpaces = depth * shiftPerDepthLevel;

    for(int i = 0; i < numSpaces; ++i)
      os << " ";

    os << name_ << "(" << memoryMap_.size() << ")" << std::endl;
  }

  void Variable::insertInMemoryMap(const parenthood_t& child, size_t whereInChild, std::vector<const double*>::const_iterator start, std::vector<const double*>::const_iterator end)
  {
    size_t indexInMap = child.getInfo().startIndexInParentMap + whereInChild;
    memoryMap_.insert(memoryMap_.begin() + indexInMap, start, end);

    parenthood_t* nextChild = child.getNextChild();
    while(nextChild)
    {
      nextChild->getInfo().startIndexInParentMap += std::distance(start, end);
      nextChild = nextChild->getNextChild();
    }

    for(size_t i = 0; i < getNumParenthoods(); ++i)
      getParenthood(i).getParent().insertInMemoryMap(getParenthood(i), indexInMap, start, end);
  }

  void Variable::callInsertInMemoryMap(Variable& obj,
      const parenthood_t& child, size_t whereInChild, std::vector<const double*>::const_iterator start, std::vector<const double*>::const_iterator end)
  {
    obj.insertInMemoryMap(child, whereInChild, start, end);
  }

  void Variable::removeFromMemoryMap(const parenthood_t& child, size_t whereInChild, size_t numElements)
  {
    size_t indexInMap = child.getInfo().startIndexInParentMap + whereInChild;
    memoryMap_.erase(memoryMap_.begin() + indexInMap, memoryMap_.begin() + indexInMap + numElements);

    parenthood_t* nextChild = child.getNextChild();
    while(nextChild)
    {
      nextChild->getInfo().startIndexInParentMap -= numElements;
      nextChild = nextChild->getNextChild();
    }

    for(size_t i = 0; i < getNumParenthoods(); ++i)
      getParenthood(i).getParent().removeFromMemoryMap(getParenthood(i), indexInMap, numElements);
  }

  void Variable::callRemoveFromMemoryMap(Variable& obj, const parenthood_t& child, size_t whereInChild, size_t numElements)
  {
    obj.removeFromMemoryMap(child, whereInChild, numElements);
  }

  std::vector<const double*>& Variable::getMemoryMap(Variable& obj)
  {
    return obj.memoryMap_;
  }

  void Variable::updateValue() const
  {
    if(!memoryMap_.size())
      throw std::runtime_error("[ocra::Variable::updateValue] cannot update value when the size of the variable is 0.");

    value_.resize( static_cast<int>(memoryMap_.size()) );
    for (int i = 0; i < static_cast<int>(memoryMap_.size()); ++i)
      value_[i] = *memoryMap_[i];
    updateValue_ = false;
  }

  void Variable::onChildValueChanged(int timestamp)
  {
    updateValue_ = true;
  }


  // ------------------------------------------------------------
  // --- BASE VARIABLE ------------------------------------------
  // ------------------------------------------------------------

  BaseVariable::BaseVariable(const std::string& name, size_t size)
    : Variable(name)
    , memory_(size)
  {
    std::vector<const double*>& memoryMap = getMemoryMap(*this);
    std::transform(memory_.begin(), memory_.end(), std::back_inserter(memoryMap), get_address());
  }

  bool BaseVariable::isBaseVariable() const
  {
    return true;
  }

  void BaseVariable::resize(size_t newSize)
  {
    std::vector<const double*>& memoryMap = getMemoryMap(*this);

    for(iterator it = parents_begin(); it != parents_end(); ++it)
      callRemoveFromMemoryMap((*it)->getParent(), **it, 0, memoryMap.size());

    memory_.resize(newSize);
    memoryMap.clear();
    std::transform(memory_.begin(), memory_.end(), std::back_inserter(memoryMap), get_address());

    for(iterator it = parents_begin(); it != parents_end(); ++it)
      callInsertInMemoryMap((*it)->getParent(), **it, 0, memoryMap.begin(), memoryMap.end());

    propagate<EVT_RESIZE>();
  }

  void BaseVariable::do_setValue(const VectorXd& value)
  {
    if (value.size() != memory_.size())
      throw std::runtime_error("[ocra::BaseVariable::setValue] Sizes don't match");

    for(int i = 0; i < value.size(); ++i)
      memory_[i] = value[i];
  }

  BaseVariable& BaseVariable::getTimeDerivative()
  {
    return static_cast<BaseVariable&>(Variable::getTimeDerivative());
  }

  BaseVariable& BaseVariable::getTimePrimitive()
  {
    return static_cast<BaseVariable&>(Variable::getTimePrimitive());
  }

  int BaseVariable::isAncestorOf(const Variable& /*unused*/) const
  {
    return 0;
  }

  void BaseVariable::printSubTree(int depth, std::ostream& os) const
  {
    printTree_impl(depth, os);
  }

  Variable* BaseVariable::do_createTimeDerivative(const std::string& name)
  {
    return new BaseVariable(name, getSize()); // throws bad_alloc if new fails hence we know we won't return 0x0
  }

  Variable* BaseVariable::do_createTimePrimitive(const std::string& name)
  {
    return new BaseVariable(name, getSize()); // throws bad_alloc if new fails hence we know we won't return 0x0
  }

  size_t BaseVariable::do_getNumberOfChildren() const
  {
    return 0;
  }


  // ------------------------------------------------------------
  // --- COMPOSITE VARIABLE -------------------------------------
  // ------------------------------------------------------------

  CompositeVariable::CompositeVariable(const std::string& name)
    : Variable(name)
  {
  }

  CompositeVariable::CompositeVariable(const std::string& name, Variable& var)
    : Variable(name)
  {
    add(var);
  }

  CompositeVariable::CompositeVariable(const std::string& name, Variable& var1, Variable& var2)
    : Variable(name)
  {
    add(var1);
    add(var2);
  }

  CompositeVariable::CompositeVariable(const std::string& name, const std::vector<Variable*>& vars)
    : Variable(name)
  {
    for(size_t i = 0; i < vars.size(); ++i)
      add(*vars[i]);
  }

  void CompositeVariable::clear()
  {
    while(children_begin() != children_end())
      remove((*children_begin())->getChild());
  }

  void CompositeVariable::do_setValue(const VectorXd& value)
  {
    if (value.size() != getSize())
      throw std::runtime_error("[ocra::BaseVariable::setValue] Sizes don't match");

    int start = 0;
    for(iterator it = children_begin(); it != children_end(); ++it)
    {
      const int size = (*it)->getChild().getSize();
      (*it)->getChild().setValue(value.segment(start, size));
      start += size;
    }
  }

  CompositeVariable& CompositeVariable::getTimeDerivative()
  {
    return static_cast<CompositeVariable&>(Variable::getTimeDerivative());
  }

  CompositeVariable& CompositeVariable::getTimePrimitive()
  {
    return static_cast<CompositeVariable&>(Variable::getTimePrimitive());
  }

  Variable& CompositeVariable::operator()(size_t i)
  {
    return getChildhood(i).getChild();
  }

  const Variable& CompositeVariable::operator()(size_t i) const
  {
    return getChildhood(i).getChild();
  }

  CompositeVariable& CompositeVariable::addByMerge(Variable& v)
  {
    if(v.getNumberOfChildren() == 0)
    {
      if(!v.isDescendantOf(*this))
        add(v);
    }
    else
    {
      CompositeVariable* compo_v = static_cast<CompositeVariable*>(&v);
      for(CompositeVariable::iterator it = compo_v->children_begin(); it != compo_v->children_end(); ++it)
        addByMerge((*it)->getChild());
    }

    return *this;
  }

  void CompositeVariable::onAttachedChild(const parenthood_t& child)
  {
    const std::vector<const double*>& childMemoryMap = getMemoryMap(child.getChild());
    callInsertInMemoryMap(*this, child, 0, childMemoryMap.begin(), childMemoryMap.end());
    propagate<EVT_CHANGE_DEPENDENCIES>();
    if (child.getChild().getSize()>0)
      propagate<EVT_RESIZE>();
  }

  void CompositeVariable::onDetachedChild(const parenthood_t& child)
  {
    callRemoveFromMemoryMap(*this, child, 0, child.getChild().getSize());
    propagate<EVT_CHANGE_DEPENDENCIES>();
    if (child.getChild().getSize()>0)
      propagate<EVT_RESIZE>();
  }

  int CompositeVariable::isAncestorOf(const Variable& var) const
  {
    return isAncestorOf_impl(var);
  }

  void CompositeVariable::printSubTree(int depth, std::ostream& os) const
  {
    printTree_impl(depth, os);
  }

  Variable* CompositeVariable::do_createTimeDerivative(const std::string& name)
  {
   CompositeVariable* d = new CompositeVariable(name);
   bool exceptionCaught = false;
   std::runtime_error toRethrow(""); // we want to re-throw the first exception we encounter...

   for(iterator it = children_begin(); it != children_end(); ++it)
   {
     try {
       Variable& dChild = (*it)->getChild().getTimeDerivative();
       d->attach(dChild);
     }
     catch(std::runtime_error& e) {
       if(!exceptionCaught)
         toRethrow = e;
       exceptionCaught = true;
     }
   }

   // if any exception caught, we'll have some subtrees created, but that's not enough
   // to consider that the time derivative of the current node is complete so we delete it and throw.
   // Note that deleting the derivative will automatically detach it from the derivatives of the children,
   // so the only side-effect is the creation of the derivative of the children.
   if(exceptionCaught)
   {
     delete d;
     throw toRethrow;
   }

   return d;
  }

  Variable* CompositeVariable::do_createTimePrimitive(const std::string& name)
  {
   CompositeVariable* p = new CompositeVariable(name);
   bool exceptionCaught = false;
   std::runtime_error toRethrow(""); // we want to re-throw the first exception we encounter...

   for(iterator it = children_begin(); it != children_end(); ++it)
   {
     try {
       Variable& pChild = (*it)->getChild().getTimePrimitive();
       p->attach(pChild);
     }
     catch(std::runtime_error& e) {
       if(!exceptionCaught)
         toRethrow = e;
       exceptionCaught = true;
     }
   }

   // see CompositeVariable::do_createTimeDerivative for details...
   if(exceptionCaught)
   {
     delete p;
     throw toRethrow;
   }

   return p;
  }

  CompositeVariable& CompositeVariable::add(Variable& child)
  {
    CompositeVariable* ptr = this;
    Variable* childPtr = &child;

    while(ptr->hasTimePrimitive() && childPtr->hasTimePrimitive())
    {
      ptr = &ptr->getTimePrimitive();
      childPtr = &childPtr->getTimePrimitive();
    }
    if(ptr->hasTimePrimitive()) // it means that childPtr has not... which we do not tolerate!
      throw std::runtime_error("[ocra::CompositeVariable::do_add] The child variable must have at least the same max order of integration as the parent");

    while(ptr && childPtr)
    {
      ptr = ptr->hasTimeDerivative() ? static_cast<CompositeVariable*>(&ptr->getTimeDerivative()) : 0x0;
      childPtr = childPtr->hasTimeDerivative() ? &childPtr->getTimeDerivative() : 0x0;
    }
    if(ptr) // it means childPtr is 0, which we refuse!
      throw std::runtime_error("[ocra::CompositeVariable::do_add] The child variable must have at least the same max order of differentiation as the parent");

    ptr = this;
    childPtr = &child;

    while(ptr->hasTimePrimitive() && childPtr->hasTimePrimitive())
    {
      ptr = &ptr->getTimePrimitive();
      childPtr = &childPtr->getTimePrimitive();
    }

    while(ptr && childPtr)
    {
      ptr->attach(*childPtr);
      ptr = ptr->hasTimeDerivative() ? static_cast<CompositeVariable*>(&ptr->getTimeDerivative()) : 0x0;
      childPtr = childPtr->hasTimeDerivative() ? &childPtr->getTimeDerivative() : 0x0;
    }

    return *this;
  }

  CompositeVariable& CompositeVariable::remove(Variable& child)
  {
    CompositeVariable* ptr = this;
    Variable* childPtr = &child;

    while(ptr->hasTimePrimitive() && childPtr->hasTimePrimitive())
    {
      ptr = &ptr->getTimePrimitive();
      childPtr = &childPtr->getTimePrimitive();
    }

    while(ptr && childPtr)
    {
      ptr->detach(*childPtr);
      ptr = ptr->hasTimeDerivative() ? static_cast<CompositeVariable*>(&ptr->getTimeDerivative()) : 0x0;
      childPtr = childPtr->hasTimeDerivative() ? &childPtr->getTimeDerivative() : 0x0;
    }

    return *this;
  }

  size_t CompositeVariable::do_getNumberOfChildren() const
  {
    return getNumChildhoods();
  }
}

// cmake:sourcegroup=Variable
