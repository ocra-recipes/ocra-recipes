/*!
\file Variable.h
\brief Declaration file of the Variable class.

Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI

\author Escande Adrien
\author Brisset Julien
\date 09/04/14

File history:
- 2010/05/26: Evrard Paul - Documentation updated and code reviewed.
- 2010/06/09: Evrard Paul - New implementation based on ocra::Composite, minor interface changes.
*/

#ifndef _OCRABASE_VARIABLE_H_
#define _OCRABASE_VARIABLE_H_

#include "ocra/optim/ObserverSubject.h"
#include "ocra/optim/Composite.h"

#include <Eigen/Eigen>
namespace ocra{using namespace Eigen;}

#include <boost/noncopyable.hpp>

#include <string>
#include <vector>

namespace ocra
{
  class CompositeVariable;
  // ------------------------------------------------------------
  // --- HELPERS ------------------------------------------------
  // ------------------------------------------------------------

  //! \internal Prevents the instantiation of derived classes.
  /*!
  \internal Prevents the instantiation of derived classes of Variable
  other than BaseVariable and CompositeVariable.
  If you attempt to modify this behavior, you cannot pretend
  you didn't do it on purpose... You are being watched...
  */
  class VariableHasRestrictedClassDerivation
  {
    VariableHasRestrictedClassDerivation() {}
    friend class Variable;
    friend class BaseVariable;
    friend class CompositeVariable;
  };

  //! This class stores information about a composite-component relationship.
  /*!
  \sa class ocra::Component.
  */
  struct VariableParenthood
  {
    size_t startIndexInParentMap;

    //! Initializes startIndexInParentMap to 0
    VariableParenthood() throw();
  };


  // ------------------------------------------------------------
  // --- VARIABLE -----------------------------------------------
  // ------------------------------------------------------------

  //! This class represents a variable in a mathematical sense.
  /*!
  This class represents the mathematical concept of variable. A variable
  has a name, and a value. A variable can be created by composition of
  existing variables (\sa ocra::CompositeVariable).

  This class is an abstract class and cannot be instantiated. Variables that
  can be instantiated are:
  - BaseVariable,
  - CompositeVariable.

  A variable is a tree whose leaves are instances of BaseVariable;
  the other nodes are instances of CompositeVariable. Any node (leaf or not) can be
  manipulated using the Variable interface. Note, however, that the behavior of the
  functions can differ depending on what instances of variables compose the tree.

  Example:
  \code
  ocra::BaseVariable b1("b1", 3);
  ocra::BaseVariable b2("b2", 2);
  ocra::CompositeVariable c("c");
  c.add(b1).add(b2);
  \endcode
  The resulting structure is:

  .                         |- BaseVariable("b1")
  .CompositeVariable("c") --|
  .                         |- BaseVariable("b2")

  An access to c[2] is equivalent to an access to b1[2] and an access to c[4] is
  equivalent to an access to b2[1]: the variables b1 and b2 have been concatenated
  to produce the composite variable c.

  \warning DO NOT ATTEMPT TO DERIVE THIS CLASS! The only intended derived classes are
  BaseVariable, and CompositeVariable.
  */
  class Variable
    // --- Base classes ---
    : public Component<Variable, CompositeVariable, VariableParenthood>
    , public ObserverSubject
    // --- Constraints ---
    , boost::noncopyable
    , virtual VariableHasRestrictedClassDerivation
  {

    // --- Type ---------------------------------------------------
  public:
    typedef Component<Variable, CompositeVariable, VariableParenthood>::parenthood_t parenthood_t;
    // --- Construction/Destruction, accessors --------------------
  public:
    Variable(const std::string& name = "");

    //! The variable is automatically detached from parents and children at destruction.
    virtual ~Variable() = 0;

    virtual const std::string& getName() const;

    /** Return true if this variable is a base variable, false otherwise. */
    virtual bool isBaseVariable() const;

    // --- Variables as functions in R^n --------------------------
  public:
    //@{
    /*!
    Get the value of the component i of the variable. The method Variable::at performs
    an additional verifications and throws std::runtime_error if i is out of bounds.
    */
    double operator[](size_t i) const;
    double at(size_t i) const;
    //@}

    int getSize() const;

    //! Implicit conversion to an Eigen vector.
    operator const VectorXd& () const;

    const VectorXd& getValue() const;
    void setValue(const VectorXd& value);

    //@{
    //! Get the time derivative/primitive of the variable.
    /*!
    If the derivative/primitive doesn't exist, it will be created automatically at the first call to this method.
    \internal These methods are declared virtual to allow covariant overloading.
    */
    virtual Variable& getTimeDerivative();
    virtual Variable& getTimePrimitive();
    //@}

    //@{
    //! Creates a time derivative/primitive with a given name.
    /*!
    Throws std::runtime_error if the derivative/primitive already exist.
    */
    void createTimeDerivative(const std::string& name);
    void createTimePrimitive(const std::string& name);
    //@}

    bool hasTimeDerivative() const;
    bool hasTimePrimitive() const;

    // --- Variables as trees (composites) ------------------------
  public:
    //! Returns the indexes of a subvariable in the variable.
    /*!
    Returns a mapping such that (*this)[mapping[i]] == subVariable[i].
    Example:
    \begincode
    ocra::BaseVariable("b1", 3);
    ocra::BaseVariable("b2", 2);
    ocra::CompositeVariable c;
    c.add(b1).add(b2);
    std::vector<int> mapping_of_b2_in_c;
    c.getRelativeMappingOf(b2, mapping_of_b2_in_c); // mapping_of_b2_in_c is [3, 4]
    \endcode
    */
    void getRelativeMappingOf(const Variable& subVariable, std::vector<int>& mapping) const;

    //! Returns 0 in BaseVariable and the number of childhoods otherwise.
    size_t getNumberOfChildren() const;

    //! Prints the name of the variables; preprends 3*depth blank spaces.
    void printNode(int depth, std::ostream& os) const;

  protected:
    //@{
    //! \internal Sets the appropriate value of startIndexInParentMap of the parenthood info.
    /*!
    \internal Overloaded from Component. \sa ocra::Component.
    */
    void onAttachedParent(const parenthood_t& parent);
    void onDetachedParent(const parenthood_t& parent);
    //@}

    // --- To overload in children classes ------------------------
  protected:
    //! This method will attempt to assign a given value to the memory map.
    virtual void do_setValue(const VectorXd& value) = 0;

    //@{
    //! These methods must be overloaded in children classes to perform instantiation and return pointers to them.
    /*!
    These methods shall not return null pointers!
    \note This will be asserted if the preprocessor symbol OCRA_ASSERT_ACTIVE is defined.
    */
    virtual Variable* do_createTimeDerivative(const std::string& name) = 0;
    virtual Variable* do_createTimePrimitive(const std::string& name) = 0;
    //@}

    //! Returns 0 in {Wrap|Base}Variable and the number of childhoods otherwise.
    /*!
    Other behaviors will result in an assertion failure in DEBUG or if OCRA_ASSERT_ACTIVE is defined.
    */
    virtual size_t do_getNumberOfChildren() const = 0;

    // --- Additional interface for derived classes ---------------
  protected:
    //@{
    //! Proxy to the implementation of memory maps update.
    /*!
    \internal These methods are static and take a reference to the Variable to which {insertIn|removeFrom}MemoryMap
    must be applied. This design allows a derived class to indirectly call the protected methods {insertIn|removeFrom}MemoryMap
    on a reference to a Variable. This solution avoids making the children classes friends of the Base class, or making
    {insertIn|removeFrom}MemoryMap public methods. This is the price to pay for encapsulation.
    */
    static void callInsertInMemoryMap
      (Variable& obj, const parenthood_t& child, size_t whereInChild,
      std::vector<const double*>::const_iterator start, std::vector<const double*>::const_iterator end);

    static void callRemoveFromMemoryMap(Variable& obj, const parenthood_t& child, size_t whereInChild, size_t numElements);

    static std::vector<const double*>& getMemoryMap(Variable& obj);
    //@}

  private:
    /*!
    \internal Used to associate a Variable instance with its time derivative and primitive. Stores a pointer
    to a Variable and a boolean whose value is true if the pointed variable has been created by this instance,
    which is the case when the time derivative/primitive is automatically created. If a variable owns its
    derivative and/or its primitive, it is then responsible for memory allocation and release.
    */
    struct VariablePtr
    {
      Variable* var_;
      bool owns_;
      VariablePtr(Variable* var, bool owns): var_(var), owns_(owns) {}
      operator bool() const { return (var_ != 0x0); }
    };

    //@{
    //! Implementation of memory maps update.
    void insertInMemoryMap(const parenthood_t& child, size_t whereInChild, std::vector<const double*>::const_iterator start, std::vector<const double*>::const_iterator end);
    void removeFromMemoryMap(const parenthood_t& child, size_t whereInChild, size_t numElements);
    //@}

    void updateValue() const;

    //! \internal callback used to trigger parent variables when one of their child changes value.
    /*!
    \internal This callback is necessary to change updateValue_ to false, so that calls to getValue() won't send
    a wrong value.
    */
    void onChildValueChanged(int timestamp);

  private:
    std::vector<const double*> memoryMap_;
    std::string name_;
    VariablePtr timeDerivative_;
    VariablePtr timePrimitive_;
    mutable VectorXd value_;
    mutable bool updateValue_;
  };


  // ------------------------------------------------------------
  // --- BASE VARIABLE ------------------------------------------
  // ------------------------------------------------------------

  //! Implements a basic variable.
  /*
  This class is part of the Variable hierarchy and correspond to the leaves of a tree
  of CompositeVariable instances. They embed the memory pointed by the composite variables
  which they are part of.

  All the methods here are documented in ocra::Variable.
  \sa ocra::Variable

  \warning When a BaseVariable is part of several trees (it has more than one parent),
  then modifying one parent variable will result in modifying the other one(s), since
  all parents share the same memory (the one embedded in BaseVariable).

  \internal Most of the behavior is implemented in ocra::Variable. This class implements
  the abstract methods defined in ocra::Variable and overloads the methods that handle
  the internal memory of the variable.
  */
  class BaseVariable
    : public Leaf<BaseVariable>
    , public Variable
  {
  public:
    //! Builds a leaf variable, i.e. a variable that is not composed of other variables.
    /*!
    \internal initializes Variable::memoryMap_.
    */
    BaseVariable(const std::string& name, size_t size);

    /** Return true if this variable is a base variable, false otherwise. */
    bool isBaseVariable() const;

    void resize(size_t newSize);

    BaseVariable& getTimeDerivative();
    BaseVariable& getTimePrimitive();

    int isAncestorOf(const Variable& var) const;
    void printSubTree(int depth, std::ostream& os) const;

  protected:
    void do_setValue(const VectorXd& value);

    //@{
    //! \sa class ocra::Variable
    Variable* do_createTimeDerivative(const std::string& name);
    Variable* do_createTimePrimitive(const std::string& name);
    //@}

    //! Always return 0.
    size_t do_getNumberOfChildren() const;

  private:
    std::vector<double> memory_;
  };


  // ------------------------------------------------------------
  // --- COMPOSITE VARIABLE -------------------------------------
  // ------------------------------------------------------------

  //! A concatenation of base variables and other composite variables.
  /*
  This class is part of the Variable hierarchy and correspond to the nodes of a tree
  of CompositeVariable instances.

  Most methods declared here are documented in ocra::Variable.
  \sa ocra::Variable

  \internal Most of the behavior is implemented in ocra::Variable.
  */
  class CompositeVariable
    : public Composite<Variable, CompositeVariable, VariableParenthood>
    , public Variable
  {
  public:
    CompositeVariable(const std::string& name);
    CompositeVariable(const std::string& name, Variable& var);
    CompositeVariable(const std::string& name, Variable& var1, Variable& var2);
    CompositeVariable(const std::string& name, const std::vector<Variable*>& vars);

    //! Detaches all children.
    void clear();

    Variable& operator()(size_t i);

    CompositeVariable& addByMerge(Variable& v);

    CompositeVariable& getTimeDerivative();
    CompositeVariable& getTimePrimitive();

    //@{
    //! Attach/detach the child to/from this node.
    CompositeVariable& add(Variable& child);
    CompositeVariable& remove(Variable& child);
    //@}
    const Variable& operator()(size_t i) const;

    int isAncestorOf(const Variable& var) const;
    void printSubTree(int depth, std::ostream& os) const;

  protected:
    void do_setValue(const VectorXd& value);

  protected:
    //@{
    /*!
    \internal Updates the memory map as well as elder brothers memory maps; also
    ask parents to update their maps. Overloaded from Composite. \sa ocra::Composite.
    */
    void onAttachedChild(const parenthood_t& child);
    void onDetachedChild(const parenthood_t& child);
    //@}

  protected:
    //@{
    //! \sa class ocra::Variable
    Variable* do_createTimeDerivative(const std::string& name);
    Variable* do_createTimePrimitive(const std::string& name);
    //@}

    //! Always return getNumChildhoods.
    size_t do_getNumberOfChildren() const;
  };
}

#endif //_OCRABASE_VARIABLE_H_

// cmake:sourcegroup=Variable
