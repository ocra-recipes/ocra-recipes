/** @file LinearizedCoulombFunction.h
  * @brief Declaration file of the LinearizedCoulombFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/04/24
  *
  * File history:
  *  - 10/07/05: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_LINEARIZED_COULOMB_FUNCTION_H
#define _OCRABASE_LINEARIZED_COULOMB_FUNCTION_H

// ocra includes
#include "ocra/optim/LinearFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class LinearizedCoulombFunction
    *	@brief %LinearizedCoulombFunction class.
    *	@warning None
    *  
    * Implementation of \f$ \left\|f_t\right\| - \mu f_n \f$ discretized and linearized. The function is thus a linear
    * function Af+b with b normally equal to 0 and A built so that f is in the cone if \f$ Af+b \le 0 \f$. b can 
    * however be given a constant non-zero value to account for a (possibly negative) margin.
    */
  class LinearizedCoulombFunction : public LinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.
  
    // ------------------------ constructors ------------------------------------
  private:
    /** Non copyable class */
    //@{
    LinearizedCoulombFunction(const LinearizedCoulombFunction&);
    LinearizedCoulombFunction& operator= (const LinearizedCoulombFunction&);
    //@}

  public:
    /** Constructor
      * Build a linear function Af+b such as Af+b<=0 is a discretized cone with \a numberOfFaces conservatively 
      * representing a Coulomb friction cone with friction coefficient \a frictionCoeff. 
      * f is supposed to represent a 3-dimensionnal force in a local frame whose z-axis is the normal vector at the 
      * contact point (so that it is the axis of the cone). b should therefore be equal to zero (which is its default 
      * value, but could have its components set to another value to account for  a (possibly negative) margin.
      *
      * By default, the first edge of the cone will be in the xz plane. This can however be changed by setting the
      * static parameter ANGLE_OFFSET to a non-zero value.
      *
      * \pre f.getSize() == 3
      * \pre numberOfFaces >= 3
      * \pre frictionCoeff > 0
      */
    LinearizedCoulombFunction(Variable& f, double frictionCoeff, int numberOfFaces=4, double margin=0);

    // ------------------------ public interface --------------------------------
  public:
    /** getters/setters */
    //@{
    double  getFrictionCoeff() const;
    double  getMargin() const;
    void    setFrictionCoeff(double coeff);
    void    setMargin(double margin);
    void              setConeOrientation(const Matrix3d& R);
    const Matrix3d&   getConeOrientation() const;
    //@}

    // ------------------------ private methods ---------------------------------
  private:
    void buildA();
    void buildb();

    // ------------------------ private static methods --------------------------
  private:
    static void checkCoeff(double mu);

    // ------------------------ protected members -------------------------------
  protected:
    double _mu;       //< the friction coefficient
    double _margin;   //< margin on the cone. Positive means tighter
    Matrix3d _R_cone;     //< orientation of the cone
    MatrixXd _J_cache;

    // ------------------------ protected static members ------------------------
  protected:
    static const double ANGLE_OFFSET;    //< offset for the orientation of the plane
  };

  void testLinearCoulomb(void);
}

#endif	//_OCRABASE_LINEARIZED_COULOMB_FUNCTION_H

// cmake:sourcegroup=Function
