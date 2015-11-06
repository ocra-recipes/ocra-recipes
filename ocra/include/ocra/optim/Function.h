/** @file Function.h
 *  @brief Declaration file of the Function class.
 *
 *         Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
 *         Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
 *
 *  @author Escande Adrien
 *  @author Brisset Julien
 *  @author Evrard Paul
 *	@date 10.05.31
 */



#ifndef _OCRABASE_FUNCTION_H_
#define _OCRABASE_FUNCTION_H_



//ocra includes

#include "ocra/optim/IFunction.h"
#include "ocra/optim/IFunctionProperties.h"
#include "ocra/optim/AbilitySet.h"
#include "ocra/optim/CoupledInputOutputSize.h"
#include "ocra/optim/Variable.h"
#include "ocra/optim/ObserverSubject.h"
#include "ocra/optim/NamedInstance.h"



/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class Function
    *	@brief %Function class.
    *	@author Escande Adrien
    *	@warning None
    *  
    * Base class for a function \f$ f_A \f$ defined as follows:
    * \f$x\f$ is time-dependant n-dimensional variable living in \f$R^n\f$, A is a set of (possibly changing) 
    * parameters. \f$ f_A \f$ is a function of \f$x\f$ with a possible explicit time dependency. Thus we have
    * \f$ f_A : R^n \times R \rightarrow R^m \f$, with \f$ f_A(x,t) = (f_{1_A}(x,t), \cdots, f_{m_A}(x,t))^T \f$
    * The set of parameters is considered to be constant with respect to the time on a mathematical point of view, yet
    * the user can change it anytime. Although \f$ f \f$ might be explicitly time-dependant, no interfaces are given
    * at this level to specify the value of variable \f$ t \f$. This responsibility is transfered to the derived classes.
    *
    * Function inherits from several IFunction, each of which comes with a virtual method that need to be overloaded if
    * the corresponding ability is \a used. Below is the list of abilities with the corresponding methods and return
    * types. In case of doubt or out-of-date documentation, please see the use of the macro DECLARE_FUNCTION_TRAITS in
    * IFunction.h. For the most usual abilities, some shortcuts are implemented to ease both programming and reading.
    *
    *  Ability           return type             virtual update method       buffer alias      getter alias
    * FUN_VALUE           VectorXd                updateValue                 _value            getValue 
    * PARTIAL_X           MatrixXd                updateJacobian              _jacobian         getJacobian 
    * PARTIAL_T           VectorXd                updatePartialT 
    * FUN_DOT             VectorXd                updateFdot 
    * PARTIAL_X_DOT       MatrixXd                updateJdot 
    * PARTIAL_XX          std::vector<MatrixXd*>  updateHessian 
    * FUN_DDOT            VectorXd                updateFddot 
    * PARTIAL_TT          VectorXd                updatePartialTT 
    * PARTIAL_TX          MatrixXd                updatePartialTX 
    * PARTIAL_XT          MatrixXd                updatePartialXT 
    * PARTIAL_T_DOT       VectorXd                updatePartialTdot
    * PARTIAL_X_DOT_X_DOT VectorXd                updateJdotXdot
    *
    * [Please UPDATE if you add abilities]
    *
    * Function also inherits from IFunctionProperties and as such is given an interface of trivial getters and setters
    * which don't appear here.
    * \sa IFunctionProperty
    */
  class Function 
    : public ObserverSubject
    , public IFunctionProperties
    , OCRA_FUNCTION_INTERFACE_INHERITANCE(public)
    , virtual public NamedInstance
    , virtual public AbilitySet
    , virtual public CoupledInputOutputSize
  {
    // ------------------------ constructors --------------------------------------
  protected:
    /**
    * \brief Function Constructor.
    *
    * \param[in] x  The time-dependant variable on which f is defined. 
    * \param[in] dimension  The dimension of the output space, \a m in the description of the class. 
    * \param[in] linearity  Linearity property of the function. 
    * \param[in] convexity  Convexity property of the function. 
    * \param[in] continuity  Continuity property of the function.
    * \param[in] explicitlyTimeDependant
    * \param[in] separableTimeDependancy
    * \param[in] use[SomeAbility] Indicates wether the function must perform memory allocation and computation for the
    *             corresponding ability.
    *
    * \pre (continuity==CONTINUITY_UNKNOWN) || (continuity>0 && continuity<=CONTINUITY_CINF)
    **/
    Function(Variable& x, int dimension,  eFunctionLinearity linearity = LINEARITY_UNDEFINED, 
             eFunctionConvexity convexity = CONVEXITY_UNDEFINED, int continuity = CONTINUITY_UNKNOWN, 
             bool explicitlyTimeDependant=false, bool separableTimeDependancy = true);

  public:
    virtual ~Function();

  protected:
    /**
     * Call this method before deleting, if needed, the variable &x in any child classes. Else it's automatically called in ~Function.
     * Error prone design inside(tm)
     */
    void disconnectVariable();

  private:
    /** copy of Function instances is forbidden*/
    //@{
    Function(const Function&);
    Function& operator= (const Function&);
    //@}


    // ------------------------ public interface ----------------------------------
  public:
    /** get the dimension m of the output space*/
    int   getDimension() const;

    /** get the variable \a x on which the function is defined */
    //@{
    const Variable&     getVariable() const;
    Variable&           getVariable();

    /** invalidate the value of the corresponding ability*/
    template<eFunctionAbility Ability>
    void invalidate();

    /** invalidate the value of each ability*/
    void invalidateAll(int timestamp);
    void invalidateAll();

    /** check the validity of the value of the corresponding ability*/
    template<eFunctionAbility Ability>
    bool isValid() const;

    /** return true if the corresponding ability is enabled*/
    template<eFunctionAbility Ability>
    bool canCompute() const;

    /** Get the value of the quantity corresponding to the function ability specified by \a Ability. 
      * This value is recomputed if needed in a transparent way for the user.
      *
      * \tparam Ability The label of the ability
      *
      * \pre IFunction<Ability>::canBeComputed()==true. If this is not the case, no physical memory is allocated for 
      * the corresponding memory buffer and computation will fail or worse result in a buffer overflow. 
      *
      * Throws std::runtime_error if the virtual update method has not been overloaded for this ability.
      *
      * \Warning For performance reasons, no exceptions are thrown when this method is called for an ability which is not 
      * used. A high-level usage should make its own check by calling first \c IFunction<Ability>::canBeComputed.
      *
      * For example, get<PARTIAL_TX>() returns \f$ \frac{\partial^2 f}{\partial t \partial x}\f$ computed at the
      * current (x,t).
      *
      * For some abilities, there exist aliases.
      * \sa getValue(), getJacobian()
      */
    template<eFunctionAbility Ability>
    const typename IFunction<Ability>::return_type& get() const;


    /** Get a subpart of the value of the quantity corresponding to the function ability specified by \a Ability. 
      * This value is recomputed if needed in a transparent way for the user.
      *
      * \tparam Ability The label of the ability.
      * \param[in] index The index of the subpart of the value (corresponding to f_index).
      *
      * For example, get<PARTIAL_X>(i) returns \f$ \frac{\partial f_i}{\partial x}\f$ computed at the current (x,t).
      *
      * \sa get()
      */
    template<eFunctionAbility Ability>
    typename IFunction<Ability>::return_sub_type get(int index) const;

    /**aliases */
    //@{
    /** return the value of the function at the current (x,t). Synonym for get<FUN_VALUE>() */
    const VectorXd& getValue() const;

    /** return the ith component of the value of the function at the current (x,t). 
      * Synonym for get<FUN_VALUE>(index) 
      */
    double          getValue(int index) const;

    /** return the jacobian of the function at the current (x,t). Synonym for get<PARTIAL_X>() */
    const MatrixXd& getJacobian() const;

    /** return the ith component of the jacobian of the function at the current (x,t). 
      * Synonym for get<PARTIAL_X>(index) 
      */
    MatrixXdRow     getJacobian(int index) const;
    //@}


    // ------------------------ public methods ------------------------------------
  public:
    /** Generic implementation of updateFdot.
      *
      * \pre The first time derivative of the variable exists.
      * \pre Value partial_X needs to be computable.
      * \pre Value partial_T needs to be computable if the function depends explicitly on time.
      *
      * \Warning This is not an optimal implementation in case it is known wether the function depends explicitly on
      *          the time or not. A test is indeed performed at runtime to decide which computations need to be done.
      */
    virtual void updateFdot() const;

    /** Generic implentation of updateFddot.
      *
      * \pre The first and second time-derivative of the variable exist.
      * \pre Values partial_X and partial_X_DOT needs to be computable.
      * \pre Values partial_XT and partialTT needs to be computable if the function depends explicitly on time.
      *
      * \Warning This is not an optimal implementation in case it is known wether the function depends explicitly on
      *          time or not. A test is indeed performed at runtime to decide which computations need to be done.
      */
    virtual void updateFddot() const;

    /** Generic implementation of updateJdotXdot.
      *
      * \pre The first time derivative of the variable exists.
      * \pre Value partial_X_DOT needs to be computable.
      */
    virtual void updateJdotXdot() const;

    // ------------------------ protected methods ---------------------------------
  protected:
    /** Changes the dimension of the function to \a newDimension.
      * This changes the value of _dimension by performing the following actions:
      * - calls changeFunctionDimensionImpl which 
      *     (i)   invokes doUpdateDimensionBegin(),
      *     (ii)  set _dimension to its new value
      *     (iii) calls resize(),
      *     (iv)  and calls doUpdateDimensionEnd()
      *   in that order
      * - fires a EVT_RESIZE.
      *
      * This method calls \c doUpdateDimensionBegin before executing its core and doUpdateDimensionEnd after.
      * These virtual methods might throw exceptions depending on their overloaded implementation. By default,
      * doUpdateSizeBegin throws a runtime_error: the programmer of a new function needs to overloaded it if his/her
      * function supports function resizing.
      *
      * \post Each IFunction<Ability>::_validated must be false.
      */ 
    void changeFunctionDimension(int newDimension);

    /** Update the size of the memory buffers according to the size of the input variable and the current dimension
      * \a m of the function. Only the buffers corresponding to used abilities are resized. This implies that their 
      * values are invalidated. This method is automaticaly invoked when the function variable \c x is resized.
      *
      * This method calls \c doUpdateSizeBegin before executing its core and doUpdateSizeEnd after.
      * These virtual methods might throw exceptions depending on their overloaded implementation. By default,
      * doUpdateSizeBegin throws a runtime_error: the programmer of a new function needs to overloaded it if his/her
      * function supports variable resizing.
      *
      * \post Each IFunction<Ability>::_validated must be false.
      */
    void updateInputSize(int timestamp);

    /** Core of \c updateInputSize() and 
      * Update the size of the memory buffers according to the size of the input variable and the dimension \a m of
      * the function. Only the buffers corresponding to used ability are resized. This implies that their values are
      * invalidated.
      */
    void resize();

    /** Methods for pre and post update in case the variable size has changed.
      * Default implementation of doUpdateInputSizeEnd does nothing, doUpdateInputSizeBegin throws an exception.
      * These methods might throw exceptions when overloaded.
      * Furthermore, \c doUpdateInputSizeEnd() must comply with the post condition of \c updateInputSize(), i.e. let 
      * the values of each ability invalidated.
      */
    //@{
    virtual void doUpdateInputSizeBegin();
    virtual void doUpdateInputSizeEnd();
    //@}

    /** Methods for pre and post update in case the dimension of the function has changed.
      * Default implementation of doUpdateDimensionEnd does nothing, doUpdateDimensionBegin throws an exception.
      * These methods might throw exceptions  when overloaded.
      * Furthermore, \c doUpdateDimensionEnd() must comply with the post condition of \c updateDimension(), i.e. let 
      * the values of each ability invalidated.
      */
    //@{
    virtual void doUpdateDimensionBegin(int newDimension);
    virtual void doUpdateDimensionEnd(int oldDimension);
    //@}

    /** Does what it tells...*/
    virtual int computeDimensionFromInputSize() const;

    // ------------------------ private methods -----------------------------------
  private:
    void changeFunctionDimensionImpl(int newDimension);

    // ------------------------ protected members ---------------------------------
  protected:
    Variable&     x;                            //< the variable x

    //Shortcuts
    VectorXd&     _value;                       //< alias on IFunction<FUN_VALUE>::_val
    MatrixXd&     _jacobian;                    //< alias on IFunction<PARTIAL_X>::_val

  private:
    int           _dimension;                   //< the dimension of the image space (m in the class description)
    bool          _hasDisconnected;             //< see error prone design(tm) method disconnectVariable 

  protected:
    const int&    _dim;                         //< alias on the dimension for easy (read-only) acces by derived class

  };

  template<eFunctionAbility Ability>
  inline void Function::invalidate()
  {
    IFunction<Ability>::invalidate();
  }

  inline void Function::invalidateAll()
  {
    OCRA_APPLY_FUNCTION_ON_ALL_INTERFACE(invalidate());
  }

  inline void Function::invalidateAll(int)
  {
    invalidateAll();
  }

  template<eFunctionAbility Ability>
  inline bool Function::isValid() const
  {
    return IFunction<Ability>::isValid();
  }

  template<eFunctionAbility Ability>
  inline bool Function::canCompute() const
  {
    return IFunction<Ability>::canBeComputed();
  }

  template< eFunctionAbility Ability>
  inline const typename IFunction<Ability>::return_type& Function::get() const
  {
    return IFunction<Ability>::get(*this);
  }

  template< eFunctionAbility Ability>
  inline typename IFunction<Ability>::return_sub_type Function::get(int index) const
  {
    return IFunction<Ability>::get(*this, index);
  }


  inline const VectorXd& Function::getValue() const
  {
    return IFunction<FUN_VALUE>::get(*this);
  }

  inline double Function::getValue(int index) const
  {
    return IFunction<FUN_VALUE>::get(*this, index);
  }

  inline const MatrixXd& Function::getJacobian() const
  {
    return IFunction<PARTIAL_X>::get(*this);
  }

  inline MatrixXdRow Function::getJacobian(int index) const
  {
    return IFunction<PARTIAL_X>::get(*this, index);
  }
}

void testFunction();

#endif //_OCRABASE_FUNCTION_H_

// cmake:sourcegroup=Function
