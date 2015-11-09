/** @file Objective.h
  * @brief Declaration file of the Objective class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 10/06/16
  */

#ifndef _OCRABASE_OBJECTIVE_H_
#define _OCRABASE_OBJECTIVE_H_

#ifdef WIN32
# pragma once
#endif

// includes
#include "ocra/optim/Variable.h"
#include "ocra/optim/Function.h"
#include "ocra/optim/FunctionInterfaceMapping.h"

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include "ocra/optim/SquaredLinearFunction.h"

namespace ocra
{
  /** @class Objective
    *	@brief %Objective class.
    *	@warning None
    * 
    * Given a function \f$ f \f$ over \f$ x \in R^n \f$ and possibly over the time \f$ t \f$, we can write an objective
    * whose most generic expression is \f$ \alpha f(x,t) \f$, where \f$ alpha \f$ could be a square matrix of size the
    * output dimension of \f$ f \f$, but we will only consider that \f$ alpha \f$ can be a real (which generally 
    * amounts to the same since most objectives have a dimension of 1).
    * 
    * The Objective class is built in a very similar way as the Constraint class, with the same mechanism to replicate
    * the Function hierarchy and the derivation from FunctionInterfaceMapping to retrieve part of the interface of 
    * the Function class given as template parameter. Please refer to Constraint documentation for more details.
    *
    * \sa Constraint
    */
  template<class T>
  class Objective
    : public Objective<typename T::functionType_t>
    , public FunctionInterfaceMapping<Objective<T> >
  {
  private:
    typedef Objective<typename T::functionType_t> ObjectiveBase;
    /** copy of Constraint instances is forbidden*/
    //@{
    Objective(const Objective&);
    Objective& operator= (const Objective&);
    //@}

  public:
    Objective(T* function, double weight=1.);

    /** Conversion operator */
    inline operator const T& () {return static_cast<T&>(ObjectiveBase::getFunction());}

    /** getter on the function on which the constraint is built */
    //@{
    inline virtual T& getFunction(void) {return static_cast<T&>(ObjectiveBase::getFunction());}
    inline virtual const T& getFunction(void) const {return static_cast<const T&>(ObjectiveBase::getFunction());}
    //@}

  private:
    typedef boost::is_base_of<Function, T> T_must_derived_from_Function;
    BOOST_STATIC_ASSERT(T_must_derived_from_Function           ::value);
  };



  template<>
  class Objective<Function> 
    : public FunctionInterfaceMapping<Objective<Function> >
  {
    // ------------------------ constructors ------------------------------------
  private:
    /** copy of Constraint instances is forbidden*/
    //@{
    Objective(const Objective&);
    Objective& operator= (const Objective&);
    //@}

  public:
    /** Specializations of the generic Constraint<T> constructors*/
    //@{
    Objective(Function* function, double weight=1.);
    //@}

    // ------------------------ public interface --------------------------------
  public:
    /** Returns the function associated with the constraint*/
    //@{
    inline virtual Function& getFunction() {return _function;}
    inline virtual const Function& getFunction() const {return _function;}
    //@}

    double getWeight() const;
    void   setWeight(double weight);
    // void changeWeight(const Eigen::VectorXd& weight);

  protected:
    Function& _function;  //< the function on which the constaint is built

  private:
    double    _weight;
  };

  
  template<class T>
  inline Objective<T>::Objective(T* function, double weight)
    : Objective<typename T::functionType_t>(function, weight)
  {}

  inline Objective<Function>::Objective(Function* function, double weight)
    : _function(*function), _weight(weight)
  {}


  inline double Objective<Function>::getWeight() const
  {
    return _weight;
  }

  inline void Objective<Function>::setWeight(double weight)
  {
    _weight = weight;
  }

  // inline void Objective<SquaredLinearFunction>::changeWeight(const Eigen::VectorXd& weight)
  // {
  //   _function.changeWeight(weight);
  // }
  
}



#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/QuadraticFunction.h"

namespace ocra
{
  typedef Objective<Function>              GenericObjective;
  typedef Objective<LinearFunction>        LinearObjective;
  typedef Objective<QuadraticFunction>     QuadraticObjective;
}


#endif //_OCRABASE_OBJECTIVE_H_

// cmake:sourcegroup=Constraint
