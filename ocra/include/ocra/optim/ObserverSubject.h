/*!
\file ObserverSubject.h
\brief Declaration file of the Observer, Subject and ObserverSubject classes.

Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI

\author Escande Adrien
\date 09/04/08

File history:
- 2010/05/14: Evrard Paul - New implementation based on Observer Subject classes.
*/

#ifndef _OCRABASE_OBSERVER_SUBJECT_H_
#define _OCRABASE_OBSERVER_SUBJECT_H_

#include "ocra/optim/ObserverSubjectBase.h"
#include "ocra/optim/ocra_events.h"

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/is_class.hpp>

namespace ocra
{
  class Subject
    : public SubjectBase<EVT_RESIZE>
    , public SubjectBase<EVT_CHANGE_DEPENDENCIES>
    , public SubjectBase<EVT_CHANGE_VALUE>
  {
  public:
    virtual ~Subject() {}

    //! Call this method to register a non-static method as a callback.
    template<int EVT, class Derived, class Base>
    void connect(Derived& object, void (Base::*newCallback)(int)) const
    {
      typedef boost::is_class<Derived>                 you_must_pass_the_first_arg_by_reference;
      typedef boost::is_base_of<Base, Derived>         callback_must_be_a_method_of_observer_or_must_be_inherited;
      typedef boost::is_convertible<Derived*, Base*>   if_inherited_callback_then_public_inheritance_needed;

      BOOST_STATIC_ASSERT(you_must_pass_the_first_arg_by_reference                     ::value);
      BOOST_STATIC_ASSERT(callback_must_be_a_method_of_observer_or_must_be_inherited   ::value);
      BOOST_STATIC_ASSERT(if_inherited_callback_then_public_inheritance_needed         ::value);

      Base& objectBase = object;
      SubjectBase<EVT>::connect(objectBase, newCallback);
    }

    //! Call this method to register a free function as a callback.
    template<int EVT>
    void connect(void (*newCallback)(int)) const
    {
      SubjectBase<EVT>::connect(newCallback);
    }

    //! Disconnect non-static method.
    template<int EVT, class Derived, class Base>
    void disconnect(Derived& object, void (Base::*callbackToErase)(int)) const
    {
      typedef boost::is_class<Derived>                 you_must_pass_the_first_arg_by_reference;
      typedef boost::is_base_of<Base, Derived>         callback_must_be_a_method_of_observer_or_must_be_inherited;
      typedef boost::is_convertible<Derived*, Base*>   if_inherited_callback_then_public_inheritance_needed;

      BOOST_STATIC_ASSERT(you_must_pass_the_first_arg_by_reference                     ::value);
      BOOST_STATIC_ASSERT(callback_must_be_a_method_of_observer_or_must_be_inherited   ::value);
      BOOST_STATIC_ASSERT(if_inherited_callback_then_public_inheritance_needed         ::value);

      Base& objectBase = object;
      SubjectBase<EVT>::disconnect(objectBase, callbackToErase);
    }

    //! Disconnect free function.
    template<int EVT>
    void disconnect(void (*callbackToErase)(int)) const
    {
      SubjectBase<EVT>::disconnect(callbackToErase);
    }

    //! \sa ocra::ObserverBase.
    template<int EVT>
	  void propagate() const    
    {
      SubjectBase<EVT>::propagate();
    }

    //! \sa ocra::ObserverBase.
    template<int EVT>
	  void propagate(int timestamp) const    
    {
      SubjectBase<EVT>::propagate(timestamp);
    }
  };

  class Observer
    : public ObserverBase<EVT_RESIZE>
    , public ObserverBase<EVT_CHANGE_DEPENDENCIES>
    , public ObserverBase<EVT_CHANGE_VALUE>
  {
  public:
    virtual ~Observer() {}

    //! Call this method to automatically propagate observed events to observers connected to the subject given in argument.
    template<int EVT>
    void bind(SubjectBase<EVT>& subject)
    {
      ObserverBase<EVT>::bind(subject);
    }

    template<int EVT>
    void stopPropagation()
    {
      ObserverBase<EVT>::stopPropagation();
    }
  };

  class ObserverSubject
    : public Observer
    , public Subject
  {
  public:
    ObserverSubject()
    {
      bind<EVT_RESIZE>(*this);
      bind<EVT_CHANGE_DEPENDENCIES>(*this);
      bind<EVT_CHANGE_VALUE>(*this);
    }

    virtual ~ObserverSubject() {}
  };
}

#endif//_OCRABASE_OBSERVER_SUBJECT_H_

// cmake:sourcegroup=Utils
