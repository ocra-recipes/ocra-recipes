/** @file FSQPConstraintManager.h
  * @brief Declaration file of the FSQPConstraintManager class.
  *
  *   Copyright (C) 2011 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 11/04/11
  *
  */

#ifndef _OCRABASE_FSQP_CONSTRAINT_MANAGER_H_
#define _OCRABASE_FSQP_CONSTRAINT_MANAGER_H_

// includes
#include "Constraint.h"
#include <vector>

namespace ocra
{
  /** @class FSQPConstraintManager
    *	@brief %FSQPConstraintManager class.
    *	@warning None
    *  
    * The aim of this class is to maintain a mapping between indices and couple (constraint, line of this constraint)
    * that is coherent with fsqp requirement: an ocra constraint is considered as a set of monodimensional constraints
    * and all this 'atomic' constraints are sorted according to their type. Non-linear inequalities come first, then
    * linear inequalities, non-linear equalities, and finally linear equalities.
    * This mapping is available with the [] operator. manager[i] returns a couple (c,j) such that the ith constraint
    * for fsqp is the jth line of the constraint c. If the constraint is of the type CSTR_LOWER_AND_GREATER, the index
    * j will range from 0 to 2d-1 included, where d is the dimension of the function the constraint is based upon. The
    * indices from 0 to d-1 correspond to the "greater than" part of the constraint, those from d to 2d-1 correspond to
    * to the lower part.
    */
  class FSQPConstraintManager
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef CwiseUnaryOp<internal::scalar_multiple_op<double>, MatrixXdRow> ScalarMultMatrixXdRow;

    struct Mapping
    {
      int length;
      int index;

      Mapping() : length(0), index(0) {}
      Mapping(int i) : length(0), index(i) {}
      Mapping(int l, int i) : length(l), index(i) {}
    };

    // ------------------------ constructors ------------------------------------
  private:
    FSQPConstraintManager(const FSQPConstraintManager&);
    FSQPConstraintManager& operator=(const FSQPConstraintManager&);
  protected:
  public:
    FSQPConstraintManager();

    // ------------------------ public interface --------------------------------
  public:
    void addConstraint(LinearConstraint& constraint);
    void addConstraint(GenericConstraint& constraint);
    void removeConstraint(LinearConstraint& constraint);
    void removeConstraint(GenericConstraint& constraint);

    double                      getValue(int i) const;
    const ScalarMultMatrixXdRow getGradient(int i) const;
    const ScalarMultMatrixXdRow getGradient(const std::pair<GenericConstraint*, int>& p) const;

    int  nineqn() const {updateMapping(); return _nineqn;}
    int  nineq()  const {updateMapping(); return _nineq;}
    int  neqn()   const {updateMapping(); return _neqn;}
    int  neq()    const {updateMapping(); return _neq;}

    void updateMapping() const; //force the update of the mapping which is otherwise done lazily when calling operator[]
    void invalidateMapping();   

    std::pair<GenericConstraint*, int> operator[] (int i) const;


    // ------------------------ private members ---------------------------------
  private:
    mutable int _nineqn;
    mutable int _nineq;
    mutable int _neqn;
    mutable int _neq;
    std::vector<GenericConstraint*> _nonLinearIneq;
    std::vector<LinearConstraint*>  _linearIneq;
    std::vector<GenericConstraint*> _nonLinearEq;
    std::vector<LinearConstraint*>  _linearEq;

    mutable std::vector<Mapping>  _mappings;
    mutable int                   _invalidatedIndex;                  
  };

  void testFsqpMapping();
}

#endif	//_OCRABASE_FSQP_CONSTRAINT_MANAGER_H_
