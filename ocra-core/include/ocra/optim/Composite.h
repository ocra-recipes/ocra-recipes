/*!
\file Composite.h
\brief Utility classes and functions to implement the Composite design pattern.

Copyright (C) 2010 CEA/DRT/LIST/DTSI/SRCI

\author Evrard Paul
\author Escande Adrien
\author Brisset Julien
\date 2010/05/31

File history:
*/

#ifndef _OCRA_COMPOSITE_H_
#define _OCRA_COMPOSITE_H_

#ifdef _MSC_VER
# pragma once
#endif

#include "ocra/optim/ocra_assert.h"

#include <boost/noncopyable.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/has_nothrow_constructor.hpp>

#include <vector>
#include <algorithm>
#include <iosfwd>
#include <stdexcept>

namespace ocra
{
  template<class ComponentDerived, class CompositeDerived, class ParenthoodInfo> class Component;
  template<class ComponentDerived, class CompositeDerived, class ParenthoodInfo> class Composite;

  //! Empty information block to attach to a parenthood relationship between a Composite and a Component.
  /*!
  Use this type as last template argument in Component and Composite if you don't need to store any
  additional information about the parent/child relationships in your composite tree.
  */
  struct NoInfo {};


  // ------------------------------------------------------------
  // --- PARENTHOOD ---------------------------------------------
  // ------------------------------------------------------------

  //! Stores information about a Parent/Child relationship in the Composite design pattern.
  /*!
  Composite objects are built upon a tree of objects, whose nodes implement the type Component. Component
  is specialized in:
  - Composite: Composite nodes have children
  - Leaf.

  See the wikipedia article about the Composite design pattern for more information.
  
  This class represents the bond between a parent and a child node in the tree. It contains:
  - the parent and the child,
  - the position of the child among the parent's children
  - the position of the parent among the child's parents,
  - additional, custom information, whose type is given by the last template argument (ParenthoodInfo).

  In this implementation, a node can be part of several trees and can therefore have several parents.
  However, cycles are forbidden.

  \tparam ComponentDerived is the abstract type used to manipulate both nodes and leaves without
  knowledge of whether they are composite or not.
  \tparam CompositeDerived is the type of the composite objects, i.e. objects that are composed of
  several subobjects.
  \tparam ParenthoodInfo is an additional type to associate additional information with a parenthood
  relationship. It must be default constructible with a default constructor (it can be the one
  defined by the compiler).

  \pre CompositeDerived is a publicly derived class of ComponentDerived.
  \pre ParenthoodInfo is default constructible with a no-throw default constructor.
  \pre ComponentDerived publicly derives Component<ComponentDerived, CompositeDerived, ParenthoodInfo>.
  \pre CompositeDerived publicly derives Composite<ComponentDerived, CompositeDerived, ParenthoodInfo>.

  The preconditions on the template parameters will be enforced at compile-time. If a BOOST_STATIC_ASSERT
  is fired when compiling your composite classes, please have a look at the name of the parameter of the
  failing assertion: it will give you a hint about how to correct your code.

  \invariant &parent.getChildhood(getRankOfChild()) == &child.getParenthood(getRankOfParent()) && &parent.getChildhood(getRankOfChild()) == this
  \invariant this->getParent().isAncestorOf(this->getChild()) == true
  \invariant this->getChild().isAncestorOf(this->getParent()) == false

  Invariants will be checked at runtime in DEBUG mode at construction and destruction.
  */
  template<class ComponentDerived, class CompositeDerived, class ParenthoodInfo = NoInfo>
  class Parenthood
    : boost::noncopyable
  {
  public:
    typedef Parenthood<ComponentDerived, CompositeDerived, ParenthoodInfo>    parenthood_t;
    typedef ComponentDerived                                                  component_t;
    typedef CompositeDerived                                                  parent_t;

  private:
    // Composite objects can create and destroy parenthood bonds to attach children.
    // To keep this operation private, Composite has been made friend of Parenthood.
    friend class Composite<ComponentDerived, CompositeDerived, ParenthoodInfo>;

    //! Create the parenthood relationship between two instances; can only be called through ocra::attach.
    /*!
    Once a relationship has been created between two instances (the parent is necessarily an instance
    of CompositeDerived, while the child might be either a CompositeDerived or a LeafDerived), it
    cannot be modified, i.e. it is impossible to change the child or parent. Their respective positions
    can however be modified. The parenthood relationship is registered in both the parent and the child.
    \pre parent.isAncestorOf(child) == false
    \pre child.isAncestorOf(parent) == false
    \pre &parent != &child
    \post &parent.getChildhood(getRankOfChild()) == &child.getParenthood(getRankOfParent()) && &parent.getChildhood(getRankOfChild()) == this
    \post parent.isAncestorOf(child) == true
    \post child.isAncestorOf(parent) == false
    \note Pre- and Post-conditions will be checked in DEBUG mode only.
    */
    Parenthood(parent_t& parent, component_t& child)
      : parent_(parent), child_(child)
      , rankOfParent_(child.parents_.size()), rankOfChild_(parent.children_.size())
      , info_()
    {
      // Check preconditions
      ocra_assert(!parent.isAncestorOf(child) && "Parent is already an ancestor of child!");
      ocra_assert(!child.isAncestorOf(parent) && "Parent is already a descendant of child!");
      ocra_assert(&child != &parent && "Cannot attach a node to itself!");

      parent_.children_.push_back(this);
      child_.parents_.push_back(this);

      // Call the callbacks to inform the parent and child about their new bond.
      child_.onAttachedParentI(*this);
      parent_.onAttachedChildI(*this);

      // Check post-conditions.
      ocra_assert(&parent.getChildhood(getRankOfChild()) == &child.getParenthood(getRankOfParent()) &&
        &parent.getChildhood(getRankOfChild()) == this &&
        "The same bond is supposed to be registered in both the parent and the child");
      ocra_assert(parent_.isAncestorOf(child_) && "Parent-child relationship not acknowledged");
      ocra_assert(!child_.isAncestorOf(parent_) && "Reversed parent-child relationship");
    }

    //! Detaches the parent from the child; can only be called through ocra::detach.
    /*!
    \internal Called by ocra::detach to update the tree when a node is detached.
    When a node is removed, its 'younger brothers', i.e. brothers with a higher rankOfChild.
    must be updated because their positions in the children list is shifted one step to the left.
    When a parent with 3 children loses its second child, then the third child becomes the new second child.
    The same goes when a parent disappears.
    \post std::find_if(this->child_.begin_parents(), this->child_.end_parents(), child_t::parenthood_t::parentIs(&this->parent_)) == this->child_.end_parents()
    \post std::find_if(this->parents_.begin_children(), this->parents_.end_children(), child_t::parenthood_t::childIs(&this->child_)) == this->parents_.end_children()

    \note For developpers (take a sheet of paper and a pen to draw before reading and fasten your seatbelt!):
    care must be taken if you edit this destructor. It is automatically triggered when the child
    or the parent dies. Therefore, these objects could be partially or totally deleted. Here are the possible cases
    when reaching this destructor:
    - no objects have been destroyed. Cool, everything's safe!
    - the parent has been destroyed. This necessarily leads to a call to the destructor of Composite, and we reach
    ~Parenthood through ~Composite. Therefore, the parent is partially complete: the Composite part still exists. We can
    use all Composite members and the only care to take is not to call any virtual function. The only virtual function
    is isAncestorOf(const Component&), which we do not call (we use the static version isAncestorOf_impl). We also
    don't use Component::isDescendantOf(const Component&) which calls the virtual version of Component::isAncesterOf.
    - the child has been destroyed and it was a derived of Leaf. Therefore this destructor is called through ~Component. Since
    we only use the Component part of child_, we are OK! Just never call child_.isAncesterOf(parent_) because it's virtual and
    child_.isDescendantOf(const Component&) since it calls a virtual function (hence the trick in invariants checks).
    - the child has been destroyed and it was a derived of Composite. This destructor is called a first time from ~Composite
    to detach the node from its children, this is the same case as when the parent is destroyed. It is then called a second
    time from ~Component to detach the parents; it's the same case as when child is a Leaf.
    Whew, we are done!
    Just take care if you edit this destructor!
    */
    ~Parenthood()
    {
      typename std::vector<parenthood_t*>::iterator itChild = parent_.children_.begin() + rankOfChild_;
      typename std::vector<parenthood_t*>::iterator itParent = child_.parents_.begin() + rankOfParent_;

      // Check invariants.
      ocra_assert(this == *itChild && this == *itParent && "Class invariant broken: the bond is not properly registered in the parent and the child.");
      ocra_assert(parent_.isAncestorOf_impl(child_) && "Class invariant broken");
      // !child_.isAncestorOf(parent_) cannot be tested here since it is a virtual function, so we use a special version.
      ocra_assert(!childIsAncestorOfParent_forDestructor() && "Class invariant broken");

      // Inform child and parents they are no longer bound to each other.
      child_.onDetachedParentI(*this);
      parent_.onDetachedChildI(*this);

      parent_.children_.erase(parent_.children_.begin() + rankOfChild_);
      child_.parents_.erase(child_.parents_.begin() + rankOfParent_);

      // Update rank of all younger brothers of child.
      for(itChild = parent_.children_.begin() + rankOfChild_; itChild != parent_.children_.end(); ++itChild)
      {
        ocra_assert((*itChild)->rankOfChild_ && "rankOfChild_ is not supposed to be decreased when already 0");
        --(*itChild)->rankOfChild_;
      }

      // Update rank of all child's parents younger than parent_.
      for(itParent = child_.parents_.begin() + rankOfParent_; itParent != child_.parents_.end(); ++itParent)
      {
        ocra_assert((*itParent)->rankOfParent_ && "rankOfParent_ is not supposed to be decreased when already 0");
        --(*itParent)->rankOfParent_;
      }
    }

    int childIsAncestorOfParent_forDestructor() // see invariants check in destructor for explanation
    {
      parent_t* child_or_null = dynamic_cast<parent_t*>(&child_);

      if(child_or_null) // we have a composite, just check using the static isAncestorOf_impl!
        return child_or_null->isAncestorOf_impl(parent_);

      return false; // we have a leaf, so of course it's no ancestor!
    }

  public:
    //@{
    //! Access to parenthood info, parent, child and their ranks, see constructor postconditions and class description.
    CompositeDerived& getParent() const { return static_cast<CompositeDerived&>(parent_); }
    size_t getRankOfParent() const { return rankOfParent_; }

    ComponentDerived& getChild() const { return static_cast<ComponentDerived&>(child_); }
    size_t getRankOfChild() const { return rankOfChild_; }

    ParenthoodInfo& getInfo() const { return info_; }
    //@}

    //@{
    //! These methods return the parent of this->getChild() at position this->getRankOfParent()-1 (+1 respectively).
    /*!
    These methods return a null pointer if called on the first (last) parenthood.
    */
    Parenthood* getPreviousParent() const
    {
      ocra_assert(rankOfParent_ < child_.parents_.size() && "rankOfParent_ >= child_->parents_.size() should never happen");
      ocra_assert(*(child_.parents_.begin() + rankOfParent_) == this && "Class invariant broken: parenthood not in the expected place in child's parents");

      if(!rankOfParent_)
        return 0x0;
      return child_.parents_[rankOfParent_ - 1];
    }

    Parenthood* getNextParent() const
    {
      ocra_assert(rankOfParent_ < child_.parents_.size() && "rankOfParent_ >= child_->parents_.size() should never happen");
      ocra_assert(*(child_.parents_.begin() + rankOfParent_) == this && "Class invariant broken: parenthood not in the expected place in child's parents");

      size_t rankOfNextParent = rankOfParent_ + 1;
      if(rankOfNextParent >= child_.parents_.size())
        return 0x0;
      return child_.parents_[rankOfNextParent];
    }
    //@}

    //@{
    //! \sa Parenthood::getPreviousParent, Parenthood::getNextParent
    Parenthood* getPreviousChild() const
    {
      ocra_assert(rankOfChild_ < parent_.children_.size() && "rankOfChild_ >= parent_->children_.size() should never happen");
      ocra_assert(*(parent_.children_.begin() + rankOfChild_) == this && "Class invariant broken: parenthood not in the expected place in parent's children");

      if(!rankOfChild_)
        return 0x0;
      return parent_.children_[rankOfChild_ - 1];
    }

    Parenthood* getNextChild() const
    {
      ocra_assert(rankOfChild_ < parent_.children_.size() && "rankOfChild_ >= parent_->children_.size() should never happen");
      ocra_assert(*(parent_.children_.begin() + rankOfChild_) == this && "Class invariant broken: parenthood not in the expected place in parent's children");

      size_t rankOfNextChild = rankOfChild_ + 1;
      if(rankOfNextChild >= parent_.children_.size())
        return 0x0;
      return parent_.children_[rankOfNextChild];
    }
    //@}

  public:
    //@{
    //! Functors to locate a parent or a child.
    /*!
    Example:
    \begincode
    class MyComposite : public MyComponent, public Composite<MyComponent, MyComposite> { ... };
    MyComposite node;
    ...
    MyComponent child;
    attach(node, child);

    MyComponent::iterator it = std::find_if(node.begin_children(), node.end_children(), MyComponent::parenthood_t::childIs(&child));
    // now it points to the parenthood objects that binds node and child
    ocra_assert(&it->getParent() == &node);
    ocra_assert(&it->getChild() == &child);
    \endcode
    \note The input range is must not contain null pointers!
    */
    struct parentIs
    {
      bool operator()(const parenthood_t* p) const { ocra_assert(p && "null pointer refused here"); return &p->getParent() == parent_; }
      parentIs(const parent_t* parent): parent_(parent) {}
      const parent_t* parent_;
    };

    struct childIs
    {
      bool operator()(const parenthood_t* p) const { ocra_assert(p && "null pointer refused here"); return &p->getChild() == child_; }
      childIs(const component_t* child): child_(child) {}
      const component_t* child_;
    };
    //@}

  private:
    // The state of a parenthood is represented by which instances are the child and parent, and by
    // their ranks. The states of the child and parent as well as the additional info are nor part of
    // the parenthood's state and are therefore mutable.
    // Note that the only methods which can change the state of a parenthood are decreaseRankOfChild
    // and decreaseRankOfParent, which are private and only accessible in ocra::detach. All other methods
    // will preserve the state of the class.

    mutable parent_t& parent_;
    mutable component_t& child_;
    size_t rankOfParent_;
    size_t rankOfChild_;
    mutable ParenthoodInfo info_;

  private: // enforce preconditions
    typedef boost::is_base_of<ComponentDerived, CompositeDerived>
      ComponentDerived_must_be_base_of_CompositeDerived;

    typedef boost::is_convertible<CompositeDerived*, ComponentDerived*>
      CompositeDerived_must_publicly_derive_ComponentDerived;

    typedef boost::has_nothrow_default_constructor<ParenthoodInfo>
      ParenthoodInfo_must_have_default_nothrow_constructor;

    typedef Component<ComponentDerived, CompositeDerived, ParenthoodInfo> component_base_t_;

    typedef boost::is_base_of<component_base_t_, ComponentDerived>
      component_base_t_must_be_base_of_ComponentDerived;
    
    typedef boost::is_convertible<ComponentDerived*, component_base_t_*>
      ComponentDerived_must_publicly_derive_component_base_t_;

    typedef Composite<ComponentDerived, CompositeDerived, ParenthoodInfo> composite_base_t_;

    typedef boost::is_base_of<composite_base_t_, CompositeDerived>
      composite_base_t_must_be_base_of_CompositeDerived;
    
    typedef boost::is_convertible<CompositeDerived*, composite_base_t_*>
      CompositeDerived_must_publicly_derive_composite_base_t_;

    BOOST_STATIC_ASSERT( ComponentDerived_must_be_base_of_CompositeDerived          ::value );
    BOOST_STATIC_ASSERT( CompositeDerived_must_publicly_derive_ComponentDerived     ::value );
    BOOST_STATIC_ASSERT( ParenthoodInfo_must_have_default_nothrow_constructor       ::value );
    BOOST_STATIC_ASSERT( component_base_t_must_be_base_of_ComponentDerived          ::value );
    BOOST_STATIC_ASSERT( ComponentDerived_must_publicly_derive_component_base_t_    ::value );
    BOOST_STATIC_ASSERT( composite_base_t_must_be_base_of_CompositeDerived          ::value );
    BOOST_STATIC_ASSERT( CompositeDerived_must_publicly_derive_composite_base_t_    ::value );
  };


  // ------------------------------------------------------------
  // --- COMPONENT ----------------------------------------------
  // ------------------------------------------------------------

  //! Base class for the Component class of the Composite pattern.
  /*!
  Composite objects are built upon a tree of objects, whose nodes implement the type Component. Component
  is specialized in:
  - Composite: Composite nodes have children
  - Leaf.

  See the wikipedia article about the Composite design pattern for more information.
  
  This class can be used as a base class to implement a Composite pattern. To use it,
  you have to take the following steps:
  1. Create an interface or a base class for your component. This interface will
  be used to manipulate your objects without knowing whether they are one-piece objects or composite objects.
  2. Derive two classes from it: one to implement the composite components, and the other one to
  implement the leaves (one-piece components).
  3. Assuming the aformentionned classes are named MyComponent, MyComposite and MyLeaf, make
  them derive from respectively: Component<MyComponent, MyComposite>, Composite<MyComponent, MyComposite>, and
  Leaf<MyLeaf>. An additional template parameter can be used for MyComponent and MyComposite to specify
  additional information about the bonds between a Composite object and its children (see class ocra::Parenthood).
  4. Implement isAncestorOf and other abstract methods in MyComposite and MyLeaf (see the doc of isAncestorOf or the example below);
      The methods to implement in your children classes are declared together in this class in a dedicated section so
      that you can easily spot them.
  5. Implement your stuff.
  6. Instantiate your objects and attach them to each other.

  The following example illustrates the simpliest form of these steps. In your code, you will of course use
  classes instead of structs if necessary...
  \begincode
  #include "Composite.h"

  struct GroupOfShapes;

  struct Shape : public ocra::Component<Shape, GroupOfShapes>
  {
  virtual void resize() = 0;
  };

  struct SingleShape : public Shape, public ocra::Leaf<SingleShape>
  {
  void resize() {}
  int isAncestorOf(const Shape& shape) const { return 0; }
  };

  struct GroupOfShapes : public Shape, public ocra::Composite<Shape, GroupOfShapes>
  {
  void resize() {}
  int isAncestorOf(const Shape& shape) const { return isAncestorOf_impl(shape); }
  };

  int main()
  {
  SingleShape leaf1;
  SingleShape leaf2;
  GroupOfShapes node;
  node.attach(leaf1);
  node.attach(leaf2);
  node.detach(leaf1);
  }
  \endcode

  In this implementation, a node can be part of several trees and can therefore have several parents.
  However, cycles are forbidden.

  \tparam ComponentDerived is the abstract type used to manipulate both nodes and leaves without
  knowledge of whether they are composite or not.
  \tparam CompositeDerived is the type of the composite objects, i.e. objects that are composed of
  several subobjects.
  \tparam ParenthoodInfo is an additional type to associate additional information with a parenthood
  relationship. It must be default constructible with a default constructor (it can be the one
  defined by the compiler).

  \pre CompositeDerived is a publicly derived class of ComponentDerived.
  \pre ParenthoodInfo is default constructible with a no-throw default constructor.
  \pre ComponentDerived publicly derives Component<ComponentDerived, CompositeDerived, ParenthoodInfo>.
  \pre CompositeDerived publicly derives Composite<ComponentDerived, CompositeDerived, ParenthoodInfo>.

  The preconditions on the template parameters will be enforced at compile-time by the Parenthood class.
  If a BOOST_STATIC_ASSERT is fired when compiling your composite classes, please have
  a look at the name of the parameter of the failing assertion: it will give you a hint about how to
  correct your code.
  */
  template<class ComponentDerived, class CompositeDerived, class ParenthoodInfo = NoInfo>
  class Component
  {
  public:
    //@{
    //! Inherited typedefs
    typedef ComponentDerived                                                    component_t;
    typedef CompositeDerived                                                    parent_t;
    typedef Parenthood<ComponentDerived, CompositeDerived, ParenthoodInfo>      parenthood_t;
    typedef typename std::vector<parenthood_t*>::const_iterator                 const_iterator;
    typedef typename std::vector<parenthood_t*>::iterator                       iterator;
    //@}

  public:
    //@{
    //! Basic access to the parents
    size_t getNumParenthoods() const { return parents_.size(); }
    const parenthood_t& getParenthood(size_t i) const { return *parents_[i]; }
    //@}

    //@{
    //! Iterator range on the set of parents
    const_iterator parents_begin() const { return parents_.begin(); }
    iterator parents_begin() { return parents_.begin(); }

    const_iterator parents_end() const { return parents_.end(); }
    iterator parents_end() { return parents_.end(); }
    //@}

    //@{
    //! Returns the number of levels that separates the component from a potential parent.
    /*!
    If the object is not a descendant of node, returns 0. If it is a child of node, returns 1.
    If it is a grand-child, returns 2, and so on...
    */
    int isDescendantOf(const CompositeDerived& node) const { return isDescendantOf(node, 1); }
    int isDescendantOf(const ComponentDerived& node) const { return node.isAncestorOf(*static_cast<const ComponentDerived*>(this)); }
    //@}

    bool isChildOf(const CompositeDerived& node)
    {
      for(const_iterator it = parents_begin(); it != parents_end(); ++it)
        if(&(*it)->getParent() == &node)
          return true;
      return false;
    }

    void printTree(std::ostream& os) { printSubTree(0, os); }

    // --- To implement in CompositeDerived and LeafDerived -------
  public:
    //! Returns the number of levels that separate the component from a potential child.
    /*
    If the object is not a ancestor of node, returns 0. If it is a parent of node, returns 1.
    If it is a grand-parent, returns 2, and so on...
    You must overload this method in the two concrete classes that derive Leaf and Composite (see the
    documentation of the class). Let's call these classes MyLeaf and MyComposite.
    - MyComposite::isAncesterOf must implement the behavior just described. simple way to do it is to forward
    the call to Composite::isAncestorOf_impl, as shown in the example in the class documentation.
    - MyLeaf::isAncesterOf must always return 0.
    \sa class Component.
    */
    virtual int isAncestorOf(const ComponentDerived& node) const = 0;

    //! Overload in ComponentDerived and CompositeDerived to simply call printTree_impl().
    /*!
    You can also choose to reimplement the while function rather than calling the proposed printTree_impl method.
    \param depth is the current depth of the node.
    */
    virtual void printSubTree(int depth, std::ostream& os) const = 0;

    //! Overload in ComponentDerived and CompositeDerived to print information about the tree node.
    /*!
    The method must not attempt to print the tree; it must only print the current node and ignore parent
    and children. Printing the tree is the role of printTree (a surprise, to be sure!).
    \param depth is the current depth of the node.
    */
    virtual void printNode(int depth, std::ostream& os) const = 0;

    // --- To overload if derived class (optional) ----------------
  protected:
    //@{
    //! Default implementation of the callbacks, to overload in class ComponentDerived.
    /*!
    These callbacks are called once a parent and a child have been attached/detached. When the callback is called,
    the parenthood has already been registered/unregistered by the child and parent (this is guaranteed by the
    ocra::attach/ detach function).
    */
    virtual void onAttachedParent(const parenthood_t& parent) {}
    virtual void onDetachedParent(const parenthood_t& parent) {}
    //@}

  protected:
    //@{
    //! \internal The constructor and destructor are protected to avoid polymorphic creation and destruction.
    /*!
    \internal Therefore, this class can only be used through the derivatives of Composite and Leaf.
    */
    Component(): componentBeingDestroyed_(false) // this boolean is to avoid CRTP virtual calls in the destructor.
    {}

    ~Component()
    {
      componentBeingDestroyed_ = true; // therefore, no (CRTP) virtual call is possible!
      while(parents_.size())
        parents_.back()->getParent().Composite<ComponentDerived, CompositeDerived, ParenthoodInfo>::remove(*static_cast<ComponentDerived*>(this));
    }
    //@}

  private:
    /*!
    \internal
    \param node is the node we are looking for in the ancestors.
    \param level is the level in the hierarchy starting from the node on which this method was
    first called: 1 if we are looking at the parents, 2 at the grand-parents etc.
    \return level of the hierarchy of ancestors at which node has been found.
    */
    int isDescendantOf(const CompositeDerived& node, int level) const
    {
      int result = 0;

      for(size_t i = 0; !result && (i < parents_.size()); ++i)
      {
        const parent_t& parent = parents_[i]->getParent();
        if(&node == &parent)
          return level;

        result = parent.isDescendantOf(node, level + 1);
      }

      return result;
    }

    //@{
    //! \internal Forward the call to the derived class.
    void onAttachedParentI(const parenthood_t& parent)
    {
      if(!componentBeingDestroyed_) // If entered destructor, do not forward the call to the derived class (already destructed!).
        onAttachedParent(parent);
    }

    void onDetachedParentI(const parenthood_t& parent)
    {
      if(!componentBeingDestroyed_)
        onDetachedParent(parent);
    }
    //@}

  private:
    std::vector<parenthood_t*> parents_;
    bool componentBeingDestroyed_;

    // http://stackoverflow.com/questions/392120/why-cant-i-declare-a-friend-through-a-typedef
    // friend class parenthood_t; // Rather make the mediator a friend rather than open the internals!
    friend class Parenthood<ComponentDerived, CompositeDerived, ParenthoodInfo>;
  };


  // ------------------------------------------------------------
  // --- COMPOSITE ----------------------------------------------
  // ------------------------------------------------------------

  //! Base class for the Composite class of the Composite pattern.
  /*!
  See the wikipedia article about the Composite design pattern for more information.
  \sa class ocra::Component
  \sa class ocra::Leaf

  \tparam ComponentDerived is the abstract type used to manipulate both nodes and leaves without
  knowledge of whether they are composite or not.
  \tparam CompositeDerived is the type of the composite objects, i.e. objects that are composed of
  several subobjects.
  \tparam ParenthoodInfo is an additional type to associate additional information with a parenthood
  relationship. It must be default constructible with a default constructor (it can be the one
  defined by the compiler).

  \pre CompositeDerived is a publicly derived class of ComponentDerived.
  \pre ParenthoodInfo is default constructible with a no-throw default constructor.
  \pre ComponentDerived publicly derives Component<ComponentDerived, CompositeDerived, ParenthoodInfo>.
  \pre CompositeDerived publicly derives Composite<ComponentDerived, CompositeDerived, ParenthoodInfo>.

  The preconditions on the template parameters will be enforced at compile-time by the Parenthood class.
  If a BOOST_STATIC_ASSERT is fired when compiling your composite classes, please have
  a look at the name of the parameter of the failing assertion: it will give you a hint about how to
  correct your code.
  */
  template<class ComponentDerived, class CompositeDerived, class ParenthoodInfo = NoInfo>
  class Composite
  {
  private:
    typedef Parenthood<ComponentDerived, CompositeDerived, ParenthoodInfo>                          parenthood_t_;
    typedef typename Component<ComponentDerived, CompositeDerived, ParenthoodInfo>::component_t     component_t_;
    typedef typename Component<ComponentDerived, CompositeDerived, ParenthoodInfo>::parent_t        parent_t_;
    typedef typename std::vector<parenthood_t_*>::const_iterator                                    const_iterator_;
    typedef typename std::vector<parenthood_t_*>::iterator                                          iterator_;

  public:
    bool isParentOf(const ComponentDerived& node)
    {
      for(const_iterator_ it = children_begin(); it != children_end(); ++it)
        if(&(*it)->getChild() == &node)
          return true;
      return false;
    }

    //@{
    //! Basic access to the children.
    size_t getNumChildhoods() const { return children_.size(); }
    const parenthood_t_& getChildhood(size_t i) const { return *children_[i]; }
    //@}

    //@{
    //! Iterator range on the children.
    const_iterator_ children_begin() const { return children_.begin(); }
    iterator_ children_begin() { return children_.begin(); }

    const_iterator_ children_end() const { return children_.end(); }
    iterator_ children_end() { return children_.end(); }
    //@}

    //@{
    //! Append and detach a child.
    /*!
    Default implementation forwards the call to the protected methods attach and detach;
    you can overload these methods to wrap attach and detach with precondition tests and
    additional operations.
    */
    virtual CompositeDerived& add(ComponentDerived& child) { attach(child); return *static_cast<CompositeDerived*>(this); }
    virtual CompositeDerived& remove(ComponentDerived& child) { detach(child); return *static_cast<CompositeDerived*>(this); }
    //@}

  protected:
    //@{
    //! Default implementation of the callbacks, to overload in the private part of class CompositeDerived.
    /*!
    These callbacks are called once a parent and a child have been attached/detached. When the callback is called,
    the parenthood has already been registered/unregistered by the child and parent (this is guaranteed by the
    ocra::attach/ detach function).
    */
    virtual void onAttachedChild(const parenthood_t_& child) {}
    virtual void onDetachedChild(const parenthood_t_& child) {}
    //@}

  protected:
    Composite(): compositeBeingDestroyed_(false) // see Component for an explanation about this bool
    {}

    ~Composite()
    {
      compositeBeingDestroyed_ = true;
      while(children_.size())
        detach(children_.back()->getChild());
    }

  protected:
    //! Attaches a Component to a Composite by simply creating a Parenthood bond between them.
    void attach(ComponentDerived& child)
    {
      new parenthood_t_(*static_cast<CompositeDerived*>(this), child);
    }

    //! Erases a Parenthood bond between a Composite and a Component.
    void detach(ComponentDerived& child)
    {
      typename std::vector<parenthood_t_*>::iterator itParent =
        std::find_if(child.parents_begin(), child.parents_end(), typename parenthood_t_::parentIs(static_cast<CompositeDerived*>(this)));

      if(itParent == child.parents_end())
        throw std::runtime_error("[ocra::Composite::detach]: parent not found in child's parents... Are the nodes really bound together?");

      delete *itParent;
    }

    int isAncestorOf_impl(const ComponentDerived& node) const { return node.isDescendantOf(*static_cast<const CompositeDerived*>(this)); }

    void printTree_impl(int depth, std::ostream& os) const
    {
      static_cast<const CompositeDerived*>(this)->printNode(depth, os);
      for(size_t i = 0; i < children_.size(); ++i)
        static_cast<ComponentDerived*>(&children_[i]->getChild())->printSubTree(depth + 1, os);
    }

  private:
    //@{
    //! \internal Forward the call to the derived class.
    void onAttachedChildI(const parenthood_t_& child)
    {
      if(!compositeBeingDestroyed_)
        onAttachedChild(child);
    }

    void onDetachedChildI(const parenthood_t_& child)
    {
      if(!compositeBeingDestroyed_)
        onDetachedChild(child);
    }
    //@}

  private:
    std::vector<parenthood_t_*> children_;
    bool compositeBeingDestroyed_;

    // see line 603 (why-cant-i-declare-a-friend-through-a-typedef)
    friend class Parenthood<ComponentDerived, CompositeDerived, ParenthoodInfo>;
  };


  // ------------------------------------------------------------
  // --- LEAF ---------------------------------------------------
  // ------------------------------------------------------------

  //! Base class for the Leaf class of the Composite pattern.
  /*!
  See the wikipedia article about the Composite design pattern for more information.
  \sa class ocra::Component
  \sa class ocra::Composite

  This class does nothing except allowing discrimination between leaves and other kind of nodes.

  \tparam LeafDerived is the type of the leaves.

  \pre LeafDerived publicly derives Leaf<LeafDerived>
  */
  template<class LeafDerived>
  class Leaf
  {
  public:
    typedef LeafDerived leaf_t;

  protected:
    Leaf()
    {
      // enforce preconditions
      typedef Leaf<LeafDerived> leaf_base_t_;

      typedef boost::is_base_of<leaf_base_t_, LeafDerived>
        leaf_base_t_must_be_base_of_LeafDerived;

      typedef boost::is_convertible<LeafDerived*, leaf_base_t_*>
        LeafDerived_must_publicly_derive_leaf_base_t_;

      BOOST_STATIC_ASSERT( leaf_base_t_must_be_base_of_LeafDerived          ::value );
      BOOST_STATIC_ASSERT( LeafDerived_must_publicly_derive_leaf_base_t_    ::value );
    }

    ~Leaf() {}

    void printTree_impl(int depth, std::ostream& os) const
    {
      static_cast<const LeafDerived*>(this)->printNode(depth, os);
    }
  };
}

#endif // _OCRA_COMPOSITE_H_

// cmake:sourcegroup=Utils
