/** @file AbsoluteCoordinatesFunction.h
  * @brief Declaration file of the AbsoluteCoordinatesFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/05/26
  */

#ifndef _OCRACONTROL_ABSOLUTE_COORDINATES_FUNCTION_H_
#define _OCRACONTROL_ABSOLUTE_COORDINATES_FUNCTION_H_

// includes
#include "ocra/optim/Function.h"
#include "ocra/control/Robot.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class AbsoluteCoordinatesFunction
    *	@brief %AbsoluteCoordinatesFunction class.
    *	@warning None
    *  
    * Description
    */
  class AbsoluteCoordinatesFunction: public Function
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef Function  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.
  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
    AbsoluteCoordinatesFunction();
    AbsoluteCoordinatesFunction(AbsoluteCoordinatesFunction&);
  protected:
  public:
    AbsoluteCoordinatesFunction(Robot* robot, cfl_size_t segment, const Vector3& v, bool isAPoint=true); //p_point_segment


    // ------------------------ public interface --------------------------------
  public:
    void setPoint(const Vector3& v);
    const Vector3& getPoint(void) const;

    // ------------------------ public methods ----------------------------------
  public:

    void checkGradient(real delta, real threshold) const;

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void computeValue(void)  const;
    virtual void computeGradient(void)  const;
    virtual void computeHessian(void)  const;
    virtual void computeJdot(void) const;
    virtual void computeJdotXdot(void) const;

    virtual void doUpdateSize(void);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:
    Robot*      _robot;
    Segment*    _segment;
    Vector3     _point;   //< a point or a vector
    bool        _isAPoint;
    mutable Matrix      J_body_scene_p_body_r_body;
    
    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRACONTROL_ABSOLUTE_COORDINATES_FUNCTION_H_

// cmake:sourcegroup=Functions
