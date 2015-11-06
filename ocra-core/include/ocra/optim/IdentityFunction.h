/** @file IdentityFunction.h
  * @brief Declaration file of the IdentityFunction class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/06/23
  */

#ifndef _OCRABASE_IDENTITY_FUNCTION_H_
#define _OCRABASE_IDENTITY_FUNCTION_H_

#ifdef WIN32
# pragma once
#endif

#include "ocra/optim/DiagonalLinearFunction.h"

namespace ocra
{
  /** @class IdentityFunction
    *	@brief %IdentityFunction class.
    *	@warning None
    *  
    * Identity function 
    */
  class IdentityFunction : public DiagonalLinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef DiagonalLinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    /** copy of IdentityFunction instances is forbidden*/
    //@{ 
    IdentityFunction(const IdentityFunction&);
    IdentityFunction& operator= (const IdentityFunction&);
    //@}

  public:
    /** Trivial constructor. If resizable is false, a runtime_error will be thrown upon resizing. */
    IdentityFunction(Variable& x, bool resizable = true);

    // ------------------------ public interface --------------------------------

    /** Trivial getter */
    bool isResizable() const;

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void updateValue() const;

    /** Throw a runtime_error if this function is not resizable*/
    virtual void doUpdateInputSizeBegin();

    /** Rebuild the jacobian and b*/
    virtual void doUpdateInputSizeEnd();

    /** This methods throw a runtime_error because diagonal and default value can't be changed for a IndentityFunction
      */
    //@{
    virtual void doChangeDiagonal(const VectorXd& d);
    virtual void doChangeDiagonal(const double diagonalElementValue, const bool changeDefault = true);
    virtual void doChangeDefaultDiagonalValue(const double v);
    virtual void doChangeDefaultbValue(const double v);
    //@}

    /** Overload checking if \a A is a diagonal matrix. It is here just in case, changing A on a Identity is not very 
      * helpfull...
      */
    virtual void doChangeA(const MatrixXd& A);

    // ------------------------ private methods -------------------------------
  private:
    void buildIdentity();

    // ------------------------ private members -------------------------------
  private:
    bool _resizable;
  };
}



#endif //_OCRABASE_IDENTITY_FUNCTION_H_

// cmake:sourcegroup=Function
