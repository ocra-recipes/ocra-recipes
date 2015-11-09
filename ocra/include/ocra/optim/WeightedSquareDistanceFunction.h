/** @file WeightedSquareDistanceFunction.h
  * @brief Declaration file of the WeightedSquareDistanceFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/06/16
  *
  * File history:
  *  - 10/07/06: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRA_WEIGHTED_SQUARE_DISTANCE_FUNCTION_H_
#define _OCRA_WEIGHTED_SQUARE_DISTANCE_FUNCTION_H_

// includes
#include "ocra/optim/QuadraticFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class WeightedSquareDistanceFunction
    *	@brief %WeightedSquareDistanceFunction class.
    *	@warning None
    *  
    * Quadratic function of the form \f& 1/2 \left\|x - x_{ref} \right\|^2_W \f& where W is a diagonal matrix.
    */
  class WeightedSquareDistanceFunction : public QuadraticFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef QuadraticFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    WeightedSquareDistanceFunction(const WeightedSquareDistanceFunction&);
    WeightedSquareDistanceFunction& operator= (const WeightedSquareDistanceFunction&);

  public:
    template <class VectorBase>
    WeightedSquareDistanceFunction(Variable& x, double weight, const VectorBase& reference);

    template <class VectorBase1, class VectorBase2>
    WeightedSquareDistanceFunction(Variable& x, const VectorBase1& weight, const VectorBase2& reference);

    // ------------------------ public interface --------------------------------
  public:
    void changeWeight(double weight);

    template <class VectorBase>
    void changeWeight(const VectorBase& weight);

    const VectorXd& getWeight() const { return _weight; }

    template <class VectorBase>
    void changeReference(const VectorBase& reference);

    // ------------------------ protected methods -------------------------------
  protected:
    void updateValue() const;
    void updateJacobian() const;

    void doUpdateInputSizeBegin();
    void doUpdateInputSizeEnd();

    void doChangePi(const MatrixXd& Pi, int index = 0);
    void doChangeqi(const VectorXd& qi, int index = 0);
    void doChangeri(double ri, int index = 0);

    // ------------------------ private methods ---------------------------------
  private:
    void computeP();
    void computeq();
    void computer();

    // ------------------------ protected members -------------------------------
  protected:
    VectorXd  _weight;        //< diagonal of the weight matrix
    VectorXd  _reference;     //< \f&x_{ref} \f&
    double    _defaultWeight; //< default value to be used when the variable size increase. Default for _reference is 0
  };


  template<class VectorBase>
  inline WeightedSquareDistanceFunction::WeightedSquareDistanceFunction(Variable& x, double weight, const VectorBase& reference)
    :NamedInstance("weighted square distance function")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false) 
    ,QuadraticFunction(x), _defaultWeight(weight), _reference(reference)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase);
    ocra_assert(reference.rows()==1 || reference.cols()==1);

    if (reference.size() != x.getSize())
      throw std::runtime_error("[WeightedSquareDistanceFunction::WeightedSquareDistanceFunction] size of reference does not match the size of the variable");

    _weight.resize(x.getSize());
    _weight.fill(_defaultWeight);
    computeP();
    computeq();
    computer();
    if (weight>0)
      changeConvexityProperty(CONVEXITY_STRICTLY_CONVEX);
    else if (weight<0)
      changeConvexityProperty(CONVEXITY_STRICTLY_CONCAVE);
    else
      changeConvexityProperty(CONVEXITY_CONVEX_AND_CONCAVE);
  }


  template <class VectorBase1, class VectorBase2>
  inline WeightedSquareDistanceFunction::WeightedSquareDistanceFunction(Variable& x, const VectorBase1& weight, const VectorBase2& reference)
    :NamedInstance("weighted square distance function")
    ,AbilitySet(PARTIAL_X, PARTIAL_XX)
    ,CoupledInputOutputSize(false) 
    ,QuadraticFunction(x), _weight(weight), _reference(reference)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase1);
    ocra_assert(weight.rows()==1 || weight.cols()==1);
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase2);
    ocra_assert(reference.rows()==1 || reference.cols()==1);

    if (reference.size() != x.getSize())
      throw std::runtime_error("[WeightedSquareDistanceFunction::WeightedSquareDistanceFunction] size of reference does not match the size of the variable");
    if (weight.size() != x.getSize())
      throw std::runtime_error("[WeightedSquareDistanceFunction::WeightedSquareDistanceFunction] size of weight does not match the size of the variable");

    _defaultWeight = weight[weight.size()-1];
    computeP();
    computeq();
    computer();

    if ((weight.array()>0).all())
      changeConvexityProperty(CONVEXITY_STRICTLY_CONVEX);
    else if ((weight.array()<0).all())
      changeConvexityProperty(CONVEXITY_STRICTLY_CONCAVE);
    else
      changeConvexityProperty(CONVEXITY_UNDEFINED);
  }


  template <class VectorBase>
  inline void WeightedSquareDistanceFunction::changeWeight(const VectorBase& weight)
  {
    if (weight.size() != x.getSize())
      throw std::runtime_error("[WeightedSquareDistanceFunction::changeWeight] size of weight does not match the size of the variable");

    _weight = weight;
    _defaultWeight = weight[weight.size()-1];
    computeP();
    computeq();
    computer();
    invalidateAll();
    propagate<EVT_CHANGE_VALUE>();
  }


  template <class VectorBase>
  inline void WeightedSquareDistanceFunction::changeReference(const VectorBase& reference)
  {
    if (reference.size() != x.getSize())
      throw std::runtime_error("[WeightedSquareDistanceFunction::changeReference] size of reference does not match the size of the variable");

    _reference = reference;
    computeq();
    computer();
    invalidateAll();
    propagate<EVT_CHANGE_VALUE>();
  }




  void testWeightedSquareDistanceFunction();
}

#endif	//_OCRA_WEIGHTED_SQUARE_DISTANCE_FUNCTION_H_

// cmake:sourcegroup=Function

