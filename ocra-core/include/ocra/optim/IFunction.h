/** @file IFunction.h
 *  @brief Declaration file of the IFunction interface.
 *
 *         Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
 *
 *  @author Escande Adrien
 *  @author Evrard Paul
 *  @date 10.05.31
 *
 * Given a (mathematical) function \a f depending on a set of time-dependant variable \a x and possibly explicitly of
 * the time t, the coding mechanism to obtain the value at the point \f$ \left(x(t),t\right) \f$ of this function or
 * one of its (partial) derivative is always the same, despite the fact that the output spaces can be different. For
 * example, if \f$ f: R^n \times R \rightarrow R^m \f, \f$ f(x,t) \f$ is a m-vector and
 * \f$ \frac{\partial^2 f}{\partial t \partial x}(x,t) \f$ is mxn matrix, yet obtaining this value eventually boils
 * down on a programmer viewpoint to calling a method of a class.
 * The IFunction interface aims at taking advantage of this similarity to factorize code.
 *
 * On an programmer viewpoint, the object corresponding to a mathematical function can only compute its value at
 * \f$ \left(x(t),t\right) \f$ or the value of a derivative if the corresponding method has been implemented. We call
 * \a ability the capacity of a (c++ object) function to compute such a value. Each possible ability produces a
 * corresponding IFunction.
 *
 * This file defines several coding mechanims (traits and macros) to make it easy to add and implement new abilities.
 * If you wish to add an ability you need to:
 *  (a) add an identifier in the enumeration eFunctionAbility,
 *  (b) add a DECLARE_FUNCTION_TRAITS call, with the desired return type for your ability and the name of the function
 *    that will compute its value,
 *  (c) add a line in the macro OCRA_FUNCTION_INTERFACE_LIST. It is VERY IMPORTANT to add this line at the end of the
 *    macros. Failing to do so would mess up with the initialization list of the derived classes in a silent way since
 *    default parameters are used.
 * In step (b) you might want to use a type for which a sub_type_traits was not defined yet, in which case you need to
 * add a DECLARE_SUBTYPE_TRAITS call with the proper arguments.
 * Every point where the programmer may want/need to add an entry is flagged by [ADD-IN] in the documentation
 * preceeding the point.
 */


#ifndef _OCRABASE_IFUNCTION_H_
#define _OCRABASE_IFUNCTION_H_

#ifdef WIN32
# pragma once
#endif

#include "ocra/MathTypes.h"
#include "ocra/optim/ocra_assert.h"
#include <iostream>
#include <vector>
#include <stdexcept>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace.
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems.
  */
namespace ocra
{
  /**
  * \brief Enumeration of the computation abilities of a ocra function
  *
  * [ADD-IN] if you wish to add an ability insert an element in this enumeration \a before PROP_NUMBER
  * \sa IFunction
  **/
  enum eFunctionAbility
  {
    FUN_VALUE = 0,        //< \f$ f(x,t) \f$
    PARTIAL_X,            //< \f$ \frac{\partial f}{\partial x}(x,t) \f$
    PARTIAL_T,            //< \f$ \frac{\partial f}{\partial t}(x,t) \f$, i.e. time derivative with x considered as constant
    FUN_DOT,              //< \f$ \dot{f}(x,t) \f$, i.e. time derivative where x is a function of t. \f$ \dot{f} = \frac{\partial f}{\partial x} \dot{x} + \frac{\partial f}{\partial t} \f$
    PARTIAL_X_DOT,        //< \f$ \dot{\left(\frac{\partial f}{\partial x}\right)}(x,t) \f$
    PARTIAL_XX,           //< \f$ \frac{\partial^2 f}{\partial x^2}(x,t) \f$
    FUN_DDOT,             //< \f$ \ddot{f}(x,t) \f$
    PARTIAL_TT,           //< \f$ \frac{\partial^2 f}{\partial t^2}(x,t) \f$
    PARTIAL_TX,           //< \f$ \frac{\partial^2 f}{\partial t \partial x}(x,t) \f$
    PARTIAL_XT,           //< \f$ \frac{\partial^2 f}{\partial x \partial t}(x,t) \f$.  necessary only for non C^2 functions...
    PARTIAL_T_DOT,        //< \f$ \dot{\left(\frac{\partial f}{\partial t}\right)}(x,t) \f$
    PARTIAL_X_DOT_X_DOT,  //< \f$ \dot{\left(\frac{\partial f}{\partial x}\right)}(x,t) \dot{x} \f$
    PROP_NUMBER           //< the number of abilities
  };

  /**
  * \brief type and function definitions for the return types.
  * \internal this is the generic (and rather empty) structure. Specializations include methods to get a sub-part
  * and to resize, see below
  **/
  template<class T>
  struct return_type_traits
  {
    typedef int sub_type_t;
  };


  /** \internal generate code to define specialized return_type_traits structure
  * This specialized structure includes the following methods:
  * - getSub returns a \a subTypeReturn object which is identified as the ith sub-part of a \a type object.
  * How to identify this subpart is specified by the \a access function name. The (optional) \a preAccess
  * is used to cast (for exemple, to make a conversion from/to pointer or reference).
  * - resize gives an implementation of a resize function for objects of class \a type. \a m (resp. \a n)
  * should correspond to the output (resp. input) space dimension, i.e. \f$ f: R^n \times R \rightarrow R^m \f
  *
  * \param[in] type The type for which the struct is specialized
  * \param[in] subTypeReturn The return type of the operation preAccess type::access(index), where index is
  * an integer
  * \param[in] preAccess a command to cast type::access(int) into subTypeReturn
  * \param[in] access the method to access the ith element of \a type
  *
  * For example, DECLARE_SUBTYPE_TRAITS(MatrixXd, MatrixXdRow, , row) create a trait associating to the
  * MatrixXd type the subtype MatrixXdRow, which is obtain by the use of MatrixXd::row(int). Additionaly,
  * it declares a function void resize(MatrixXd& v, int m, int n) whose definition is given below.
  **/
  #define DECLARE_SUBTYPE_TRAITS(type, subTypeReturn, preAccess, access) \
  template<> struct return_type_traits<type > \
  { \
    typedef subTypeReturn sub_type_t; \
    static subTypeReturn getSub(const type& v, int index) {return preAccess(v.access(index));} \
    static void resize(type& v, int m, int n=0);\
  };

  /** \internal use of the DECLARE_SUBTYPE_TRAITS macro for the types used in ocra::Function.
  *
  * [ADD-IN] If you need other types, you need to add a line here, and give a definition of the \a resize function
  **/
  DECLARE_SUBTYPE_TRAITS(VectorXd, double, /*nothing*/, operator[]);
  DECLARE_SUBTYPE_TRAITS(MatrixXd, MatrixXdRow, /*nothing*/, row);
  DECLARE_SUBTYPE_TRAITS(std::vector<MatrixXd*>, const MatrixXd&, *, operator[]);

  #undef DECLARE_SUBTYPE_TRAITS

  /** Definition of return_type_traits<T>::resize for VectorXd*/
  inline void return_type_traits<VectorXd>::resize(VectorXd& v, int m, int n) {v.resize(m);}

  /** Definition of return_type_traits<T>::resize for MatrixXd*/
  inline void return_type_traits<MatrixXd>::resize(MatrixXd& v, int m, int n) {v.resize(m,n);}

  /** Definition of return_type_traits<T>::resize for std::vector<MatrixXd*>*/
  inline void return_type_traits<std::vector<MatrixXd*> >::resize(std::vector<MatrixXd*>& v, int m, int n)
  {
    ocra_assert(0<=m && "invalide size m");
    ocra_assert(0<=n && "invalide size n");
    //if the dimension decrease, we remove the supernumerary matrices.
    //entered only if m < v.size()
    for (int i=(int)v.size()-1; i>=m; --i)
    {
      delete v[i];
      v.pop_back();
    }

    //we resize the matrices
    for (size_t i=0; i<v.size(); ++i)
      v[i]->resize(n, n);

    //if the dimension increases, we add the necessary matrices.
    //entered only if m > v.size()
    for (size_t i=v.size(); i<(size_t)m; ++i)
      v.push_back(new MatrixXd(n, n));
  }

  /** \internal generate code to define general and specialized ocra_function_traits structure
  *
  * The ocra_function_traits structure is meant to associate associate a function ability with its
  * data type, subtype, and a method to compute this data (calling a virtual method that need to be overloaded).
  *
  * \param[in] templateArg The template argument for generic definition of the structure.
  * \param[in] propName The specialization argument. For an argument whose name is \a name, it should
  * be given with the following syntax: <name>
  * \param[in] returnType The type of the data associated to the \a propName ability
  * \param[in] virtualFunctionName Name of the virtual method whose role is to update the data.
  * \param[in] abilityId Id number. For an argument whose name is \a name, it should simply be: name
  *
  * \Warning \a virtualFunctionName need to be unique name for every propName. If not, several methods would
  * share the same overloaded virtual function
  *
  * For example, DECLARE_FUNCTION_TRAITS( , <PARTIAL_TX>, MatrixXd, updateJdot) creates the structure
  * ocra_function_traits<PARTIAL_TX>, specifying that the data associated to PARTIAL_TX (i.e.
  * \f$ \frac{\partial^2 f}{\partial t \partial x}(x,t) \f$) is a MatrixXd, its subtype (through return_type_traits<MatrixXd>)
  * is a MatrixXdRow, and the virtual function to be called by update is updateJdot
  **/
  #define DECLARE_FUNCTION_TRAITS(templateArg, propName, returnType, virtualFunctionName, abilityId) \
  template<templateArg> struct ocra_function_traits propName \
  { \
    typedef returnType type_t; \
    typedef return_type_traits<returnType>::sub_type_t sub_type_t; \
    virtual void virtualFunctionName() const \
    { \
      std::stringstream s; \
      s << "[Function::" << #virtualFunctionName << "] is not implemented. Did you forget to declare its overload as a protected const method ?"; \
      std::cout << s.str()<<std::endl; \
      throw std::runtime_error(s.str()); \
    } \
    /** \internal we need a mechanism to call the right virtual function, i.e. the one associated to the actual \
    derived class, from a method defined in the base class. This is done by templating update with the type of the \
    derived class*/ \
    template<class FunctionType> static void update(const FunctionType& function) { function.virtualFunctionName(); } \
    static bool extractUsage(const std::vector<bool>& usageSet) {return usageSet[abilityId];}\
  };

  /** \internal Generic definition of ocra_function_traits*/
  DECLARE_FUNCTION_TRAITS( eFunctionAbility Property, /* general */, int /*arbitrary type */, updateProperty /* arbitraryName */, -1 /* arbitrary id*/);

  /** \internal use of the DECLARE_FUNCTION_TRAITS macro for each function ability.
  *
  * [ADD-IN] If you need another ability, you need to add a line here, as well as in the end of the macro
  * OCRA_FUNCTION_INTERFACE_LIST
  **/
  DECLARE_FUNCTION_TRAITS(/* special */, <FUN_VALUE>, VectorXd, updateValue, FUN_VALUE);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_X>, MatrixXd, updateJacobian, PARTIAL_X);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_T>, VectorXd, updatePartialT, PARTIAL_T);
  DECLARE_FUNCTION_TRAITS(/* special */, <FUN_DOT>, VectorXd, updateFdot, FUN_DOT);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_X_DOT>, MatrixXd, updateJdot, PARTIAL_X_DOT);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_XX>, std::vector<MatrixXd*>, updateHessian, PARTIAL_XX);
  DECLARE_FUNCTION_TRAITS(/* special */, <FUN_DDOT>, VectorXd, updateFddot, FUN_DDOT);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_TT>, VectorXd, updatePartialTT, PARTIAL_TT);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_TX>, MatrixXd, updatePartialTX, PARTIAL_TX);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_XT>, MatrixXd, updatePartialXT, PARTIAL_XT);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_T_DOT>, VectorXd, updatePartialTdot, PARTIAL_T_DOT);
  DECLARE_FUNCTION_TRAITS(/* special */, <PARTIAL_X_DOT_X_DOT>, VectorXd, updateJdotXdot, PARTIAL_X_DOT_X_DOT);

  #undef DECLARE_FUNCTION_TRAITS



  /**
  * \brief Computation ability of a ocra function
  *
  * This class provides a generic way to implement the function ability to compute some mathematical
  * quantity (typical example are its value or some derivatives at a point).
  * This computation is done by the method \a update which is inherited from ocra_function_traits and its result
  * is accessed by \a get.
  * IFunction provides an update mechanism around the \a update method to avoid computing several time the same
  * quantity. To do so it uses a memory \a _val whose type (\a return_type) is derived from the value of the template
  * parameter \a Property to cache the computation result and a boolean to flag the current value as valid or not.
  * This update mechanism requires from the user to overload the virtual function called in \a update, and to inform
  * of the invalidation of the value. The part where it is decide wether to recompute the value or not and call
  * update if needed is transparent to the user.
  *
  * \a IFunction is only meant to be derived and as such does not offer public constructors.
  *
  * \warning: the buffer \a _val is accessible from the derived classes. It is the responsibilty of those classes to
  * invalidate the gradient computation (with the \c invalidate() method) and if necessary the computations of others
  * abilities, whenever they change the value of \a _val.
  **/
  template<eFunctionAbility Property>
  class IFunction
    : public ocra_function_traits<Property>
  {
    // ------------------------------------------------------------
    // -------------- Constructor / Destructor --------------------
    // ------------------------------------------------------------
  protected:
    /**
    * \brief IFunction Constructor.
    *
    * \param[in] props A vector of bool whose element numbered \a Property indicates wether to use or not this function
    * ability.
    **/
    IFunction(const std::vector<bool>& props)
      :ocra_function_traits<Property>()
      ,_used(ocra_function_traits<Property>::extractUsage(props))
      ,_validated(false)
    {
    };

    ~IFunction()
    {
      resizeData(0, 0);
    }

  private:
    /** \internal lonely private constructors, never to be used*/
    //@{
    IFunction(const IFunction&);
    IFunction& operator= (const IFunction&);
    //@}

  public:
    /** Definition of the type of the quantity computed in this class and its subtype.
      * The type is derived from the template parameter of the class.
      * /sa ocra_function_traits, return_type_traits
      */
    //@{
    typedef typename ocra_function_traits<Property>::type_t return_type;
    typedef typename ocra_function_traits<Property>::sub_type_t return_sub_type;
    //@}

  protected:
    /** Flags the cached value \a _val as invalid, indicating to the \a get methods the need for recomputation. */
    void invalidate() {_validated = false;}

    /** Return the validity of the actual value. */
    bool isValid() const {return _validated;}

    /** \return \a true if this function ability is used, \a false otherwise. */
    bool canBeComputed() const {return _used;}


    /** Get the quantity computed by this function ability.
      * The quantity is recomputed if needed in a transparent way for the user.
      *
      * \param[in] data The actual class instance on which to call the update function. It should be \c *this.
      *
      * \return the value of the quantity.
      *
      * \pre _used==true. If this is not the case, no physical memory is allocated for the buffer and computation will
      * fail or worse result in a buffer overflow.
      *
      * \warning Since there are no exeption throws in case the precondition is violated, a high-level usage should
      * make its own check by calling first \a canBeComputed.
      *
      * \internal Calling directly the virtual method \a virtualFunctionName inherited from ocra_function_traits (the
      * actual name being defined by the macro declaration) in \a get would always mean to use the virtual method
      * defined in ocra_function_traits, whatever the derived class from which the method is called, and wether or not
      * this derived class defined an overload of the virtual method.
      * For example, if a class A derives of IFunction<Value> and overload the virtual method IFunction<Value>::updateValue
      * a call to updateValue in IFunction<Value> would always be resolved by a call to IFunction<Value>::updateValue.
      * The solution used here is to pass the object on which to call the virtual method, without downcasting it to a
      * base class. Then the call of object.virtualFunctionName will call the right virtualFunctionName methods (In our
      * example, we would pass the instance of A as the object and A::updateValue would be called). To avoid the
      * downcasting, we need to template the ocra_function_traits<Property>::update method, and thus the get method, by
      * the type of the derived class.
      */
    template<class FunctionType>
    const return_type& get(FunctionType& data) const
    {
      ocra_assert(_used && "this function property is not used bordel");
      if (!_validated)
      {
        ocra_function_traits<Property>::update(data);
        _validated = true;
      }
      return _val;
    }

    /** Get a subpart of the quantity.
      *
      * \param[in] data The actual class instance on which to call the update function. It should be \c *this.
      * \param[in] index The index of the subpart we're interested in.
      *
      * \return the subvalue of the quantity with index \index.
      *
      * \sa get(FunctionType&)
      **/
    template<class FunctionType>
    return_sub_type get(FunctionType& data, int index) const
    {
      get(data);
      return return_type_traits<return_type>::getSub(_val, index);
    }

    /** Resizes \c _val according to the size of the function
      *
      * \param[in] m dimension of the output space of the function
      * \param[in] n dimension of the input space of the function
      */
    void resizeData(int m, int n)
    {
      if (_used)
      {
        _validated = false;
        return_type_traits<return_type>::resize(_val, m, n);
      }
    }
  protected:
    mutable return_type _val;           //< the memory buffer to keep the result of the computation.

  private:
    mutable bool        _validated;     //< the flag to know wether _val is up-to-date or not
    bool                _used;          //< the flag indicating if this ability is used
  };


/** \internal a macro listing all the existing IFunction and used for further macro definition
  *
  * [ADD-IN] If you need another ability, you need to add a line here, as well as in the end of the macro
  * DECLARE_FUNCTION_TRAITS
  *
  * \param[in] begin Commands to be put before the first element of the list
  * \param[in] pre Commands to be put before each element of the list but the first one
  * \param[in] end Commands to be put after each element of the list including the first one
  **/
#define OCRA_FUNCTION_INTERFACE_LIST(begin, pre, end) \
  begin IFunction<FUN_VALUE> end \
  pre IFunction<PARTIAL_X> end \
  pre IFunction<PARTIAL_T> end \
  pre IFunction<FUN_DOT> end \
  pre IFunction<PARTIAL_X_DOT> end \
  pre IFunction<PARTIAL_XX> end \
  pre IFunction<FUN_DDOT> end \
  pre IFunction<PARTIAL_TT> end \
  pre IFunction<PARTIAL_TX> end \
  pre IFunction<PARTIAL_XT> end \
  pre IFunction<PARTIAL_T_DOT> end \
  pre IFunction<PARTIAL_X_DOT_X_DOT> end \
  /*pre IFunction<[NEW_PROP]> end*/

/** \internal simply defined a coma macro so as to be able to use ',' as an argument for other macros*/
#define OCRA_FUNCTION_INTERFACE_COMA ,

/** \internal defines a macro that can be used as an inheritance list. Adding a line to OCRA_FUNCTION_INTERFACE_LIST
  * automatically add the corresponding IFunction in this inheritance list
  *
  * \param[in] inheritanceAccessRight [virtual ]public/protected/private
  */
#define OCRA_FUNCTION_INTERFACE_INHERITANCE(inheritanceAccessRight)\
  OCRA_FUNCTION_INTERFACE_LIST(inheritanceAccessRight, OCRA_FUNCTION_INTERFACE_COMA inheritanceAccessRight, )

/** \internal defines a macro to call the same method on all the class defined in OCRA_FUNCTION_INTERFACE_LIST
  * automatically add the corresponding IFunction in this inheritance list
  *
  * \param[in] functionAndArgs the name of the method followed by the list of arguments (if any) between parenthesis
  * Example: OCRA_APPLY_FUNCTION_ON_ALL_INTERFACE(myMethod(a, 3.14, obj.foo())
  */
#define OCRA_APPLY_FUNCTION_ON_ALL_INTERFACE(methodAndArgs) \
  OCRA_FUNCTION_INTERFACE_LIST(, ,  ::methodAndArgs;)

/** \internal Defines in a single base-constructor-like type the list of all base constructors for a class which would
  * inherit from all IFunction defined in OCRA_FUNCTION_INTERFACE_LIST.
  */
#define OCRA_FUNCTION_INTERFACE_INITIALIZE(usageSet)\
  OCRA_FUNCTION_INTERFACE_LIST(/**/, OCRA_FUNCTION_INTERFACE_COMA, (usageSet))

}

#endif //_OCRABASE_IFUNCTION_H_

// cmake:sourcegroup=Function

