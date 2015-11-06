/*!
\file ObserverSubjectBase.h
\brief Declaration file of the Observer and Subject base classes.

Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI

\author Evrard Paul
\author Escande Adrien
\date 10/06/14

File history:
*/

#ifndef _OCRA_OBSERVER_SUBJECT_BASE_H_
#define _OCRA_OBSERVER_SUBJECT_BASE_H_

#include <functional>
#include <algorithm>

#include <boost/any.hpp>
#include <boost/type_traits/is_class.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/static_assert.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

namespace ocra
{
  // ------------------------------------------------------------
  // --- INVOKER ------------------------------------------------
  // ------------------------------------------------------------

  template<int EVT> class ObserverBase;

  //! \internal Interface to invoke a callback wrapper.
  /*!
  \internal This class is used by class ocra::SubjectBase to manipulate the callbacks registered
  by observers. We distinguish between 3 kinds of callbacks (who take a timestamp as argument):
  - Free functions with one int arguments: void (*)(int),
  - Methods from classes T that derive ObserverBase: void (T::*)(int),
  - Methods from arbitrary classes with one integer argument.
  An issue with the first two kinds of callback is that we have to store a typed pointer to member,
  hence we need a template class. However, template class cannot be stored in containers, hence we
  have to abstract the template class behind the interface InvokerBase.
  */
  template<int EVT>
  class InvokerBase
  {
  protected:
    typedef ObserverBase<EVT> observer_base_type;
    typedef void (observer_base_type::*member_callback_type)(int);
    typedef void (*free_callback_type)(int);

  public:
    virtual ~InvokerBase() {} // Invoker instances are polymorphically destroyed by SubjectBase.

    //@{
    //! \internal Compares wrapped object and callback to an (object,method) pair.
    template<class T>
    bool isObserverEqualTo(std::pair<T*, void (T::*)(int)> observer)
    {
      return observer.first == getObject() && isCallbackEqualTo(observer.second);
    }

    template<class T>
    bool isCallbackEqualTo(void (T::*callback)(int))
    {
      bool result;
      try {
        result = callback == boost::any_cast<void (T::*)(int)>(getCallback());
      }
      catch(boost::bad_any_cast&) {
        result = false;
      }
      return result;
    }

    //! \internal Compares wrapped callback to a free function.
    bool isCallbackEqualTo(void (*callback)(int))
    {
      bool result;
      try {
        result = callback == boost::any_cast<void (*)(int)>(getCallback());
      }
      catch(boost::bad_any_cast&) {
        result = false;
      }
      return result;
    }
    //@}

  public:
    //! \internal Invokes the callback; possibly through ObserverBase.
    /*!
    If the wrapped callback is a method of a derived class of ObserverBase, forwards the
    call to ObserverBase so that the propagation mechanism is properly invoked.
    Else simply invokes the callback. \sa ocra::ObserverBase.
    */
    virtual void trigger(int timestamp) = 0;

  protected:
    virtual boost::any getCallback() = 0;
    virtual void* getObject() = 0;
  };


  // ------------------------------------------------------------

  //! \internal General implementation of InvokerBase cannot be defined...
  template<class T, int EVT, bool T_isDerivedFromObserverBase> class Invoker
  {
  };


  // ------------------------------------------------------------

  //! \internal Implementation of InvokerBase for the derived-of-ObserverBase callbacks.
  template<class T, int EVT>
  class Invoker<T, EVT, true /*meaning: ObserverBase<EVT> is base of T*/>
    : public InvokerBase<EVT>
  {
  public:
    Invoker(T& object, void (T::*callback)(int)): object_(object), callback_(callback), timestamp_(-1) {}

    void trigger(int timestamp)
    {
      object_.InvokerBase<EVT>::observer_base_type::trigger(object_, callback_, timestamp, timestamp_);
    }

  protected:
    boost::any getCallback() { return boost::any(callback_); }
    void* getObject() { return &object_; }

  private:
    T& object_;
    void (T::*callback_)(int);
    int timestamp_;
  };


  // ------------------------------------------------------------

  //! \internal Implementation of InvokerBase for the arbitrary class callbacks.
  template<class T, int EVT>
  class Invoker<T, EVT, false /*meaning: ObserverBase<EVT> is NOT a base of T*/>
    : public InvokerBase<EVT>
  {
  public:
    Invoker(T& object, void (T::*callback)(int)): object_(object), callback_(callback) {}

    void trigger(int timestamp) { if(callback_) (object_.*callback_)(timestamp); }

  protected:
    boost::any getCallback() { return boost::any(callback_); }
    void* getObject() { return &object_; }

  private:
    T& object_;
    void (T::*callback_)(int);
  };


  // ------------------------------------------------------------

  //! \internal Implementation of InvokerBase for the free function callbacks.
  template<int EVT>
  class Invoker<void, EVT, false /*true would have no meaning in the free function case*/>
    : public InvokerBase<EVT>
  {
  public:
    Invoker(void (*callback)(int)): callback_(callback) {}

    void trigger(int timestamp) { if(callback_) (*callback_)(timestamp); }

  protected:
    boost::any getCallback() { return boost::any(callback_); }
    void* getObject() { return 0x0; }

  private:
    void (*callback_)(int);
  };

  // ------------------------------------------------------------

  //! \internal We never want to see this...
  template<int EVT>
  class Invoker<void, EVT, true>
  {
  };


  // ------------------------------------------------------------
  // --- OBSERVER_BASE ------------------------------------------
  // ------------------------------------------------------------

  template<int EVT> class SubjectBase;

  //! Base class for Observers with propagation system.
  /*!
  Use this class as a public base class to observe subjects and propagate the observed
  events. Classes who derive this class can connect to Subjects so that if these
  subjects raise an event with a timestamp greater than the last event's timestamp, then
  the registered callback is called; else nothing happens. Other subjects can be bound to
  an Observer so that the event is automatically propagated to these subjects; in this case,
  observers who observe these bound subjects will be triggered, except if the method
  ObserverBase::stopPropagation() is called within the callback.

  For observers who derive ObserverBase:
  When a callback is triggered because a subject has raised an event with timestamp t0,
  any event with timestamp t <= t0 will have no effect; the callback will be triggered for
  events with timestamps t > t0. This is to avoid infinite looping when an observer
  is connected to a subject and propagates the event through a cycle to this same
  subject. This also allows calling a callback more than once for the same events when
  a callback can be triggered for the same event with two different pathes.

  \warning For the time being, ensuring that observers and subjects are alive is left to
  the client code. No automatic unregistration of a dying observer or subject is performed.

  Example:
  \begincode
  // Creation of a chain of observers and subjects for an event identified with the integer constant MY_EVENT_ID.
  class MySubject : public SubjectBase<MY_EVENT_ID> { ... };
  class MyObserverSubject : public ObserverBase<MY_EVENT_ID>, public SubjectBase<MY_EVENT_ID> { ... };
  class MyObserver : public ObserverBase<MY_EVENT_ID> { ... };
  ...
  MySubject subject;
  MyObserverSubject obsSubject;
  MyObserver observer;
  subject.connect(obsSubject, &MyObserverSubject::theMethodYouWant);
  obsSubject.bind(obsSubject); // so that when obsSubject receives an event from subject, it can propagate it and be observed.
  obsSubject.connect(observer, &MyObserver::anyMethodYouThinkIsSuitable);
  subject.propagate(); // will trigger obsSubject.theMethodYouWant(), which propagates to observer.anyMethodYouThinkIsSuitable()...
  \endcode
  */
  template<int EVT>
  class ObserverBase
  {
  protected:
    typedef SubjectBase<EVT>          subject_type;
    typedef InvokerBase<EVT>          invoker_type;

  public:
    //! Call this method to automatically propagate observed events to observers connected to the subject given in argument.
    void bind(subject_type& subject) { subject_ = &subject; }

  protected:
    //! Call this method from your callbacks to avoid propagation to the bound subject (if any).
    void stopPropagation() { propagate_ = false; }

    //@{
    //! \internal This class must be derived to be used, and instances cannot be deleted polymorphically.
    ObserverBase(): subject_(0x0), propagate_(true) {}
    ~ObserverBase() {}
    //@}

  private:
    //! \internal This method is supposed to be called through the Invoker only.
    template<class T>
    void trigger(T& object, void (T::*callback)(int), int timestamp, int& savedTimestamp)
    {
      if(timestamp > savedTimestamp)
      {
        savedTimestamp = timestamp;
        propagate_ = true;
        if(callback)
          (object.*callback)(timestamp);
        if(propagate_ && subject_)
          subject_->propagate(timestamp);
      }
    }

    subject_type* subject_;
    bool propagate_;

    template<class, int, bool> friend class Invoker;
  };


  // ------------------------------------------------------------
  // --- SUBJECT_BASE -------------------------------------------
  // ------------------------------------------------------------

  //! Base class for objects who can raise events, \sa ocra::ObserverBase for an example.
  /*!
  \warning For the time being, ensuring that observers and subjects are alive is left to
  the client code. No automatic unregistration of a dying observer or subject is performed.
  */

  template<int EVT> struct SubjectBaseTraitsBase {
    typedef InvokerBase<EVT>                                                  invoker_base_type;
  };
  
  template<int EVT, class T>
  struct SubjectBaseTraits : SubjectBaseTraitsBase<EVT>
  {
    typedef typename SubjectBaseTraitsBase<EVT>::invoker_base_type            invoker_base_type;
    typedef                                                                   void (T::*callback_type)(int);
    typedef Invoker<T, EVT,boost::is_base_of<ObserverBase<EVT>, T>::value>    invoker_type;
    typedef std::pair<T*, callback_type>                                      invoker_id_type;
    typedef std::mem_fun1_ref_t<bool, invoker_base_type, invoker_id_type>     invoker_comparator_type;
  };

  template<int EVT> 
  struct SubjectBaseTraits<EVT, void> : SubjectBaseTraitsBase<EVT>
  {
    typedef typename SubjectBaseTraitsBase<EVT>::invoker_base_type            invoker_base_type;
    typedef                                                                   void (*callback_type)(int);
    typedef Invoker<void, EVT,false>                                          invoker_type;
    typedef callback_type                                                     invoker_id_type;
    typedef std::mem_fun1_ref_t<bool, invoker_base_type, invoker_id_type>     invoker_comparator_type;
  };

  template<int EVT>
  class SubjectBase
  {
  private:
    typedef typename SubjectBaseTraitsBase<EVT>::invoker_base_type            invoker_base_type;
  protected:
    //@{
    //! \internal This class must be derived to be used, and instances cannot be deleted polymorphically.
    SubjectBase() {}
    ~SubjectBase() {}
    //@}

  public:
    //! Call this method to register a non-static method as a callback.
    template<class T>
    void connect(T& object, typename SubjectBaseTraits<EVT, T>::callback_type newCallback) const
    {
      BOOST_STATIC_ASSERT(boost::is_class<T>::value); // Avoid passing (void*,int*,char*,...)
      observers_.push_back(new typename SubjectBaseTraits<EVT, T>::invoker_type(object, newCallback));
    }

    //! Call this method to register a free function as a callback.
    void connect(typename SubjectBaseTraits<EVT, void>::callback_type newCallback) const
    {
      observers_.push_back(new typename  SubjectBaseTraits<EVT, void>::invoker_type(newCallback));
    }

    //! Disconnect non-static method.
    template<class T>
    void disconnect(T& object, typename SubjectBaseTraits<EVT, T>::callback_type callback) const
    {
      BOOST_STATIC_ASSERT(boost::is_class<T>::value);

      typename SubjectBaseTraits<EVT, T>::invoker_comparator_type comparator = std::mem_fun_ref(&invoker_base_type::template isObserverEqualTo<T>);
      typename SubjectBaseTraits<EVT, T>::invoker_id_type elementToErase(&object, callback);

      observers_.erase_if( std::bind2nd(comparator, elementToErase) );
    }

    //! Disconnect free function.
    void disconnect(typename SubjectBaseTraits<EVT, void>::callback_type callbackToErase) const
    {
      typename SubjectBaseTraits<EVT, void>::invoker_comparator_type comparator =  
        std::mem_fun_ref<bool, invoker_base_type, void (*)(int)>(&invoker_base_type::isCallbackEqualTo);
      observers_.erase_if( std::bind2nd(comparator, callbackToErase) );
    }

    //! \sa ocra::ObserverBase.
    void propagate(int timestamp) const
    {
      std::mem_fun1_ref_t<void, invoker_base_type, int> trigger = std::mem_fun_ref(&invoker_base_type::trigger);
      std::for_each(observers_.begin(), observers_.end(), std::bind2nd(trigger, timestamp));
    }

    void propagate() const { static int timestamp = 0; propagate(++timestamp); }

  private:
    mutable boost::ptr_vector<invoker_base_type> observers_;
  };
}

#endif

// cmake:sourcegroup=Utils
