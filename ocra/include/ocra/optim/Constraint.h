/** @file Constraint.h
  * @brief Declaration file of the Constraint class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 09/04/24
  *
  * File history:
  *   - 10/06/10 Escande Adrien: mapping of Function Interface is deferred to functionInterfaceMapping, adding lower
  * and upper bounds to inequality constraints.
  */

#ifndef _OCRABASE_CONSTRAINT_H_
#define _OCRABASE_CONSTRAINT_H_

#ifdef WIN32
# pragma once
#endif


// includes
#include "ocra/optim/Variable.h"
#include "ocra/optim/Function.h"
#include "ocra/optim/FunctionInterfaceMapping.h"

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>


//#include <sstream>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** Enumerate the different types of equality/inequality constraints
    * 
    * \internal All the equality constraint types must come before the inequality constraint ones and CSTR_LOWER_ZERO
    * must be the first inequality constraint type appearing in this enumeration. isEquality and isInequality rely on
    * this properties to return the correct value. 
    */

  //never change this enum
  enum eConstraintType
  {
    CSTR_EQUAL_ZERO=0,        //> \f$ f(x,t) = 0 \f$
    CSTR_EQUAL_B,             //> \f$ f(x,t) = b \f$
    CSTR_LOWER_ZERO,          //> \f$ f(x,t) \le 0 \f$
    CSTR_LOWER_U,             //> \f$ f(x,t) \le u \f$
    CSTR_GREATER_ZERO,        //> \f$ f(x,t) \ge 0 \f$
    CSTR_GREATER_L,           //> \f$ f(x,t) \ge l \f$
    CSTR_LOWER_AND_GREATER    //> \f$ l \le f(x,t) \le u \f$
  };

  /** @class Constraint
    *	@brief %Constraint class.
    *	@warning None
    *  
    * Given a function \f$ f \f$ over \f$ x \in R^n \f$ and possibly over the time \f$ t \f$, we can write a constraint
    * whose most generic expression is \f$ l \le f(x,t) \le u \f$, with \f$ l \f$ and \f$ u \f$ two vectors of size
    * \f$ m \f$ having their value in \f$ \left(R\cup\left\{-\infty,+\infty\right\}\right)^m \f$ and verifying 
    * \f$ l \le u \f$. This class is a C++ translation of this mathematical writing, but distinguishes some specific
    * cases, for the ease of use, that are enumerated by eConstraintType :
    * - \f$ l = u = b \f$, the constraint is treated as an equality to b (which can be 0): CSTR_EQUAL_ZERO and 
    *   CSTR_EQUAL_B,
    * - \f$ l = -\infty \f$ (resp. \f$ u = +\infty \f$), in which case the lower (resp. upper) bound is ignored. The
    *   remaining bound can be 0: CSTR_LOWER_ZERO, CSTR_LOWER_U, CSTR_GREATER_ZERO and CSTR_GREATER_L,
    * - the general case: CSTR_LOWER_AND_GREATER.
    *
    * The Constraint class is templated by the type of the function on which the constraint is built. Provided the
    * class definition of this function provides the good typedef functionType_t, the hierarchy of the Function classes
    * is mimicked by Constraint: if B is a class deriving of A and B::functionType_t is A, then Constraint<B> will
    * automatically derived from Constraint<A>.
    * As a concrete example, if DiagonalLinearFunction derives from LinearFunction that derives from Function, then
    * Constraint<DiagonalLinearFunction> will derive from Constraint<LinearFunction> that will itself derive from
    * Constraint<Function>. Thus a Constraint<DiagonalLinearFunction> is also a Constraint<LinearFunction> for example.
    * 
    * A Constraint class inherits from FunctionInterfaceMapping of the interface of Function, an instance of Constraint
    * can be treated as a Function, to directly call methods of function on it. Constraint<T> is however not deriving
    * from Function.
    *
    * \sa FunctionInterfaceMapping
    *
    * \internal Constraint<T> and its derived Constraint can inherits from the interface of T, or part of it by
    * specializing FunctionInterfaceMapping for T. Indeed, at each level of the constraint hierarchy, Constraint<T>
    * derives from FunctionInterfaceMapping<Constraint<T> > that is by default an empty structure but has access to 
    * the function f on which the constraint is built. Thus, for a Function class \a AFunction, one can write a
    * specialization FunctionInterfaceMapping<Constraint<AFunction> > whose methods can access f as a AFunction.
    *
    * The mechanism to replicate the Function class hierarchy was proposed by
    * Paul Evrard, AIST/CNRS Joint japanese-french Robotic Lab (JRL, UMI3218/CRT), evrardp@gmail.com
    * based on ideas from Imperfect C++, M. Wilson, Addison Wesley
    */
  template<class T>
  class Constraint
    : public Constraint<typename T::functionType_t>
    , public FunctionInterfaceMapping<Constraint<T> >
  //TODO [mineur] ? Function could be put to T::functionType_t to verify 
  //that functionType_t is defined according to the inheritance tree of Function
  {
  private:
    typedef Constraint<typename T::functionType_t>    BaseConstraint; 

    /** copy of Constraint instances is forbidden*/
    //@{
    Constraint(Constraint&);
    Constraint& operator= (const Constraint&);
    //@}

  public:
    /** Constraint constructor to build simple equality/inequality constraints
      *
      * \param[in] function A pointer to the function on which the constraint is imposed.
      * \param[in] equality True if the constraint is an equality, false if it is an inequality.
      * \param[in] v An optional vector to shift the constraint
      *
      * The following cases arise:
      *  - Constraint(f, true): the constraint describes \f$ f(x,t) = 0 \f$ (type CSTR_EQUAL_ZERO) 
      *  - Constraint(f, false): the constraint describes \f$ f(x,t) \le 0 \f$ (type CSTR_LOWER_ZERO)
      *  - Constraint(f, true, b): the constraint describes \f$ f(x,t) = b \f$ (type CSTR_EQUAL_B)
      *  - Constraint(f, false, u): the constraint describes \f$ f(x,t) \le u \f$ (type CSTR_LOWER_U)
      *
      * \pre If v is given (and non null), v.size() must be equal to function->getDimension()
      */
    Constraint(T* function, bool equality, const VectorXd& v=VectorXd());

    /** Constraint constructor to build inequality constraints with possibly lower and upper bounds
      *
      * \param[in] function A pointer to the function on which the constraint is imposed.
      * \param[in] l Lower bound vector.
      * \param[in] u Lower bound vector.
      *
      * The following cases arise:
      *  - Constraint(f): the constraint describes \f$ f(x,t) \ge 0 \f$ (type CSTR_GREATER_ZERO) 
      *  - Constraint(f, l): the constraint describes \f$ f(x,t) \ge l \f$ (type CSTR_GREATER_L)
      *  - Constraint(f, l, u): the constraint describes \f$ l \le f(x,t) \le u \f$ (type CSTR_LOWER_AND_GREATER)
      *  - Constraint(f, l, u) with l==VectorXd(): the constraint describes \f$ f(x,t) \le u \f$ (type CSTR_LOWER_U)
      *  - Constraint(f, l, u) with l==u: the constraint describes \f$ f(x,t) = u \f$ (type CSTR_EQUAL_B)
      *
      * \pre l<=u
      * \pre If l or u is given (and non null), their size must be equal to function->getDimension()
      */
    Constraint(T* function, const VectorXd& l=VectorXd(), const VectorXd& u=VectorXd());

    /** Conversion operator */
    inline operator const T& () {return static_cast<T&>(BaseConstraint::getFunction());}

    /** getter on the function on which the constraint is built */
    //@{
    inline virtual T& getFunction(void) {return static_cast<T&>(BaseConstraint::getFunction());}
    inline virtual const T& getFunction(void) const {return static_cast<const T&>(BaseConstraint::getFunction());}
    //@}

  private:
    typedef boost::is_base_of<Function, T> T_must_derived_from_Function;
    BOOST_STATIC_ASSERT(T_must_derived_from_Function           ::value);
  };









  template<>
  class Constraint<Function>
    : public FunctionInterfaceMapping<Constraint<Function> >
    , public SubjectBase<EVT_CSTR_CHANGE_BOUNDS_NUMBER>
  {
    // ------------------------ structures --------------------------------------
  public:
    /* The slack variable functionalities are disabled for now
    class UpdatableSlackVariable: public ObserverSubject
    {
    private :
      UpdatableSlackVariable();
      UpdatableSlackVariable(UpdatableSlackVariable&);
    public:
      UpdatableSlackVariable(Function* f)
        :_f(f)
      {
        std::stringstream s;
        s << "_slack_";
        s << cpt++;
        _slack = new BaseVariable(s.str(), f->getDimension());
        f->attach(*this);
      }

      ~UpdatableSlackVariable()
      {
        delete _slack;
        _f->completelyDetach(*this);
      }
  
      virtual void updateSize(void);
      inline Variable* getSlackVariable(void) {return _slack;}

    protected:
      static int cpt;
      BaseVariable* _slack;
      Function* _f;
    };*/

    // ------------------------ constructors ------------------------------------
  private:
    /** copy of Constraint instances is forbidden*/
    //@{
    Constraint(Constraint&);
    Constraint& operator= (const Constraint&);
    //@}

  public:
    /** Specializations of the generic Constraint<T> constructors*/
    //@{
    Constraint(Function* function, bool equality, const VectorXd& v=VectorXd());
    Constraint(Function* function, const VectorXd& l=VectorXd(), const VectorXd& u=VectorXd());
    //@}

    // ------------------------ public interface --------------------------------
  public:
    /** Returns the function associated with the constraint*/
    //@{
    inline virtual Function& getFunction() {return _function;}
    inline virtual const Function& getFunction() const {return _function;}
    //@}

    /** Return true if the ith component of the constraint is valid for the actual value of its function, false 
      * otherwise. If the parameter is non-positive, it will check for the validity of all the components.
      * Validity of a component is check with respect to the violation tolerance. This tolerance is 1.e-7 by default
      * and can be changed with setViolationTolerance() 
      *
      * \param[in] index. Index of the constraint component. If non-positive (as with the default value), all
      * components will be considered
      */
    inline bool isRespected(int index=-1) const;

    /** Equality/Inequality property accessor */
    //@{
    inline bool isEquality() const;
    inline bool isInequality() const;
    //@}

    /** All of the following methods do not change the equality/inequality property of the constraint. In particular,
      * changing \a l and or \a u for an inequality constraint so as to have \a l = \a u will not make it become an
      * equality constraint.
      */
    //@{
    /** Set the right member of an equality constraint.
      * If \a b is null the constraint is considered as an equality to zero (type CSTR_EQUAL_ZERO). If not, it stays or
      * becomes an equality to \a b (type CSTR_EQUAL_B).
      *
      * \pre isEquality()==true
      * \pre If b is not null then b.size==getDimension()
      */
    void setB(const VectorXd& b);

    /** Set the lower bound of an inequality constraint.
      * If \a l is null, the lower bound of the contraint is removed (if applicable). In case the constraint only had a
      * lower bound, this bound is set to zero.
      * If \a l is not null, the lower bound is changed to the value of \a l. If the constraint had no lower bound it is
      * created and set to \a l.
      *
      * \pre isInequality()==true
      * \pre If l is not null then l.size==getDimension()
      * \pre If l is not null, l<_u
      */
    void setL(const VectorXd& l);

    /** Set the upper bound of an inequality constraint.
      * If \a u is null, the upper bound of the contraint is removed (if applicable). In case the constraint only had
      * an upper bound, this bound is set to zero.
      * If \a u is not null, the upper bound is changed to the value of \a u. If the constraint had no upper bound it is
      * created and set to \a u.
      *
      * \pre isInequality()==true
      * \pre If u is not null then u.size==getDimension()
      * \pre If u is not null, _l<u
      */
    void setU(const VectorXd& u);

    /** This method combines setL() and setU().
      * It is provided to avoid breaking the precondition l<u of setL() or setU(), which could happen by calling 
      * sequentially both methods while the new \a l and \a u are perfectly valid.
      * In case \a l or \a u is null (including the case when both are null), the method calls setL() and setU() in the
      * proper order.
      *
      * \pre isInequality()==true.
      * \pre If l and u are not null, l<u.
      * \pre If l or u is null, the precondition of setL() and setU().
      *
      * \sa setL(), setU().
      */
    void setLandU(const VectorXd& l, const VectorXd& u);
    //@}

    /** getters on the datas of Constraint */
    //@{
    eConstraintType getType() const;
    /** Get the right member of an equality constraint. 
      *
      * \pre isEquality()==true.
      */
    const VectorXd& getB() const;
    /** Get the lower bound of an inequality constraint.
      *
      * \pre isEquality()==false.
      */
    const VectorXd& getL() const;
    /** Get the upper bound of an inequality constraint.
      *
      * \pre isEquality()==false.
      */
    const VectorXd& getU() const;
    //@}

    /** getter and setter for the constraint violation tolerance
      *
      * \pre tol>=0
      */
    //@{
    void    setViolationTolerance(double tol);
    double  getViolationTolerance() const;
    //@}
 
    /* methods related to slack variable, 
    void makeSlacked();
    void unmakeSlacked();
    inline bool isSlacked() const;
    inline Variable* getSlackVariable();
    */

    // ------------------------ protected members -------------------------------
  protected:
    Function&               _function;  //< the function on which the constaint is built

    // ------------------------ private members ---------------------------------
  private:
    eConstraintType         _type;      //< true if this is an equality constraint
    VectorXd                _l;         //< lower bound
    VectorXd                _u;         //< upper bound, or equality value (b)
    double                  _violation; //< accepted violation of the constraint
    // UpdatableSlackVariable* _slack;     //< possible slack variables


    //TODO [mineur] : add a scaling factor : need Constraint to be an observer and have its own memory for results
  };


    


  inline bool Constraint<Function>::isRespected(int index) const
  {
    if (index < 0)
    {
      switch (_type)
      {
        case CSTR_EQUAL_ZERO:         return getValue().isZero(_violation);                               break;
        case CSTR_EQUAL_B:            return ((getValue().array()-_u.array()).abs() <= _violation).all(); break;
        case CSTR_LOWER_ZERO:         return (getValue().array() <= _violation).all();                    break;
        case CSTR_LOWER_U:            return ((getValue().array()-_u.array()) <= _violation).all();       break;
        case CSTR_GREATER_ZERO:       return (getValue().array() >= -_violation).all();                   break;
        case CSTR_GREATER_L:          return ((getValue().array()-_l.array()) >= -_violation).all();      break;
        case CSTR_LOWER_AND_GREATER:  return ((getValue().array()-_u.array()) <= _violation).all() 
                                          && ((getValue().array()-_l.array()) >= -_violation).all();      break;
        default: 
          throw std::runtime_error("[Constraint<T>::isValid] invalid constraint type");
      }
    }
    else
    {
      switch (_type)
      {
        case CSTR_EQUAL_ZERO:         return fabs(getValue(index)) <= _violation;           break;
        case CSTR_EQUAL_B:            return fabs(getValue(index)-_u[index]) <= _violation; break;
        case CSTR_LOWER_ZERO:         return getValue(index) <= _violation;                 break;
        case CSTR_LOWER_U:            return (getValue(index)-_u[index]) <= _violation;     break;
        case CSTR_GREATER_ZERO:       return getValue(index) >= -_violation;                break;
        case CSTR_GREATER_L:          return (getValue(index)-_l[index]) >= -_violation;    break;
        case CSTR_LOWER_AND_GREATER:  return (getValue(index)-_u[index]) <= _violation 
                                          && (getValue(index)-_l[index]) >= -_violation;    break;
        default:
          throw std::runtime_error("[Constraint<T>::isValid] invalid constraint type");
      }
    }
  }

  inline bool Constraint<Function>::isEquality() const
  {
    return _type < CSTR_LOWER_ZERO;
  }

  inline bool Constraint<Function>::isInequality() const
  {
    return _type >= CSTR_LOWER_ZERO;
  }

  inline void Constraint<Function>::setB(const VectorXd& b)
  {
    ocra_assert(isEquality() && "You can change b only for an equality constraint");
    if (b.size() > 0)
    {
      ocra_assert(b.size() == getDimension()  && "b has not the size of the function");
      _type = CSTR_EQUAL_B;
    }
    else
      _type = CSTR_EQUAL_ZERO;
    _u = b;
  }

  inline void Constraint<Function>::setL(const VectorXd& l)
  {
    ocra_assert(isInequality() && "You can change l only for an inequality constraint");
    if (l.size() > 0)
    {
      ocra_assert(l.size() == getDimension()  && "l has not the size of the function");
      ocra_assert(
        (
        _type == CSTR_GREATER_ZERO
        || _type == CSTR_GREATER_L
        || (_type == CSTR_LOWER_ZERO && (l.array()<0).all() )
        || (_u.size() != 0 && (l.array()-_violation < _u.array()).all())
        )
        && "when there is a upper bound, it must be greater than l");
      switch(_type)
      {
        case CSTR_LOWER_ZERO:         _u = VectorXd::Zero(getDimension());                      //no break here on purpose
        case CSTR_LOWER_U:            _type = CSTR_LOWER_AND_GREATER; 
                                      SubjectBase<EVT_CSTR_CHANGE_BOUNDS_NUMBER>::propagate();  break;
        case CSTR_GREATER_ZERO:       _type = CSTR_GREATER_L;                                   break;
        case CSTR_GREATER_L:          /*nothing to do*/                                         break;        
        case CSTR_LOWER_AND_GREATER:  /*nothing to do*/                                         break;
      }
    }
    else
    {
      switch(_type)
      {
        case CSTR_LOWER_ZERO:         /*nothing to do*/                                         break;
        case CSTR_LOWER_U:            /*nothing to do*/                                         break;
        case CSTR_GREATER_ZERO:       /*nothing to do*/                                         break;
        case CSTR_GREATER_L:          _type = CSTR_GREATER_ZERO;                                break;
        case CSTR_LOWER_AND_GREATER:  _type = CSTR_LOWER_U;
                                      SubjectBase<EVT_CSTR_CHANGE_BOUNDS_NUMBER>::propagate();  break;
      }
    }
    _l = l;
  }

  inline void Constraint<Function>::setU(const VectorXd& u)
  {
    ocra_assert(isInequality() && "You can change u only for an inequality constraint");
    if (u.size() > 0)
    {
      ocra_assert(u.size() == getDimension()  && "u has not the size of the function");
      ocra_assert(
        (
        _type == CSTR_LOWER_ZERO 
        || _type == CSTR_LOWER_U       
        || ( _type == CSTR_GREATER_ZERO && (u.array()>0).all() )      
        || (_l.size() != 0 && (_l.array()-_violation < u.array()).all())
        )
        && "when there is a lower bound, it must be lower than u");
      switch(_type)
      {
        case CSTR_LOWER_ZERO:         _type =  CSTR_LOWER_U;                                    break;
        case CSTR_LOWER_U:            /*nothing to do*/                                         break;
        case CSTR_GREATER_ZERO:       _l = VectorXd::Zero(getDimension());                      //no break here on purpose
        case CSTR_GREATER_L:          _type = CSTR_LOWER_AND_GREATER;
                                      SubjectBase<EVT_CSTR_CHANGE_BOUNDS_NUMBER>::propagate();  break;
        case CSTR_LOWER_AND_GREATER:  /*nothing to do*/                                         break;
      }
    }
    else
    {
      switch(_type)
      {
        case CSTR_LOWER_ZERO:         /*nothing to do*/                                         break;
        case CSTR_LOWER_U:            _type = CSTR_LOWER_ZERO;                                  break;
        case CSTR_GREATER_ZERO:       /*nothing to do*/                                         break;
        case CSTR_GREATER_L:          /*nothing to do*/                                         break;        
        case CSTR_LOWER_AND_GREATER:  _type = CSTR_GREATER_L;
                                      SubjectBase<EVT_CSTR_CHANGE_BOUNDS_NUMBER>::propagate();  break;
      }
    }
    _u = u;
  }

  inline void Constraint<Function>::setLandU(const VectorXd& l, const VectorXd& u)
  {
    ocra_assert(isInequality() && "You can change l and u only for an inequality constraint");
    if (l.size()>0)
    {
      ocra_assert(l.size() == getDimension()  && "l has not the size of the function");
      if (u.size() > 0)
      {
        ocra_assert(u.size() == getDimension()  && "u has not the size of the function");
        ocra_assert((l.array()-_violation < u.array()).all() && "l must be lower than u");
        switch(_type)
        {
          case CSTR_LOWER_ZERO:         
          case CSTR_LOWER_U:            
          case CSTR_GREATER_ZERO:       
          case CSTR_GREATER_L:          _type = CSTR_LOWER_AND_GREATER;
                                        SubjectBase<EVT_CSTR_CHANGE_BOUNDS_NUMBER>::propagate();  break;
          case CSTR_LOWER_AND_GREATER:  /*nothing to do*/                                         break;
        }
        _l = l; 
        _u = u;
      }
      else
      {
        // l is initialized
        // u is not initialized (u.size() == 0)
        setL(l);
        setU(u);
      }
    }
    else
    {
      ocra_assert( u.size()>0 && "l and u are not initialized, constraint is undefined");

      // l is not initialized (l.size() == 0)
      // u is initialized
      setU(u);
      setL(l);
    }
  }

  inline eConstraintType Constraint<Function>::getType() const
  {
    return _type;
  }

  inline const VectorXd& Constraint<Function>::getB() const
  {
    ocra_assert(isEquality() && "getB() can be called only on an equality constraint");
    return _u;
  }

  inline const VectorXd& Constraint<Function>::getL() const
  {
    ocra_assert(isInequality() && "getL() can be called only on an inequality constraint");
    return _l;
  }

  inline const VectorXd& Constraint<Function>::getU() const
  {
    ocra_assert(isInequality() && "getU() can be called only on an inequality constraint");
    return _u;
  }

  inline void Constraint<Function>::setViolationTolerance(double tol)
  {
    ocra_assert(tol>=0 && "tolerance must be a positive number");
    _violation = tol;
  }

  inline double Constraint<Function>::getViolationTolerance() const
  {
    return _violation;
  }



  //constructors
  template<class T>
  inline Constraint<T>::Constraint(T *function, bool equality, const VectorXd& v)
    :Constraint<typename T::functionType_t>(function, equality, v)
  {
      //force T to inherit of Function
                                        //TODO [mineur] ? could be put to T::functionType_t to verify 
                                        //that functionType_t is defined according to the inheritance tree of Function
  }

  template<class T>
  inline Constraint<T>::Constraint(T* function, const VectorXd& l, const VectorXd& u)
    :Constraint<typename T::functionType_t>(function, l, u)
  {
      //force T to inherit of Function
                                        //TODO [mineur] ? could be put to T::functionType_t to verify 
                                        //that functionType_t is defined according to the inheritance tree of Function
  }

  inline Constraint<Function>::Constraint(Function *function, bool equality, const VectorXd& v)
    :_function(*function)/*, _slack(NULL)*/, _u(v), _violation(1.e-7)
  {
    if (equality)
    {
      if (v.size() > 0)
      {
        ocra_assert(v.size() == getDimension() && "v has not the size of the function");
        _type = CSTR_EQUAL_B;
      }
      else
        _type = CSTR_EQUAL_ZERO;
    }
    else
    {
      if (v.size() > 0)
      {
        ocra_assert(v.size() == getDimension() && "v has not the size of the function");
        _type = CSTR_LOWER_U;
      }
      else
        _type = CSTR_LOWER_ZERO;
    }
  }

  inline Constraint<Function>::Constraint(Function* function, const VectorXd& l, const VectorXd& u)
    :_function(*function)/*, _slack(NULL)*/, _l(l), _u(u), _violation(1.e-7)
  {
    if (l.size() > 0)
    {
      ocra_assert(l.size() == getDimension() && "l has not the size of the function");
      if (u.size() > 0)
      {
        ocra_assert(_l.size() == _u.size() && "l and u don't have the same size");
        ocra_assert((l.array()-_violation < u.array()).all() && "l must be lower than u");
        if (l.isApprox(u))
          _type = CSTR_EQUAL_B;
        else
          _type = CSTR_LOWER_AND_GREATER;
      }
      else
        _type = CSTR_GREATER_L;
    }
    else
    {
      if (u.size() > 0)
      {
        ocra_assert(u.size() == getDimension() && "u has not the size of the function");
        _type = CSTR_LOWER_U;
      }
      else
        _type = CSTR_GREATER_ZERO;
    }
  }
/*
  inline bool Constraint<Function>::isSlacked(void) const
  {
    return _slack != NULL;
  }

  inline Variable* Constraint<Function>::getSlackVariable(void)
  {
    if (_slack != NULL)
      return _slack->getSlackVariable();
    else
      return NULL;
  }*/
}

#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/BoundFunction.h"
#include "ocra/optim/IdentityFunction.h"
#include "ocra/optim/DiagonalLinearFunction.h"
//#include "ocra/optim/LinearTask.h"
//#include "ocra/optim/SecondOrderLinearTask.h"
//#include "ocra/optim/QuadraticFunction.h"

namespace ocra
{
  typedef Constraint<Function>                 GenericConstraint;
  typedef Constraint<LinearFunction>           LinearConstraint;
  typedef Constraint<BoundFunction>            BoundConstraint;
  typedef Constraint<IdentityFunction>         IdentityConstraint;
  typedef Constraint<DiagonalLinearFunction>   DiagonalLinearConstraint;
//  typedef Constraint<LinearTask>            TaskConstraint;
//  typedef Constraint<SecondOrderLinearTask> SecondOrderTaskConstraint;
//  typedef Constraint<QuadraticFunction>     QuadraticConstraint;
}

#endif	//_OCRABASE_CONSTRAINT_H_

// cmake:sourcegroup=Constraint

