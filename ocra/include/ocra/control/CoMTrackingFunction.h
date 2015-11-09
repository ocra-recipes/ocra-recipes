/** @file CoMTrackingFunction.h
  * @brief Declaration file of the CoMTrackingFunction class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/03/07
  */

#ifndef _OCRACONTROL_COM_TRACKING_FUNCTION_H_
#define _OCRACONTROL_COM_TRACKING_FUNCTION_H_

// includes
#include "ocra/optim/Function.h"
#include "ocra/control/Robot.h"
#include "ocra/control/Target.h"
#include "ocra/control/ControlEnum.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class CoMTrackingFunction
    *	@brief %CoMTrackingFunction class.
    *	@warning None
    *  
    * Description
    */
  class CoMTrackingFunction : public Function
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
    CoMTrackingFunction();
    CoMTrackingFunction(CoMTrackingFunction&);
    CoMTrackingFunction& operator= (const CoMTrackingFunction&);
  protected:
  public:
    CoMTrackingFunction(Robot* robot, const Target& target, ECartesianDof fixedPosition = XYZ);

    // ------------------------ public interface --------------------------------
  public:
    void changeTarget(const Target& target);

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void computeValue(void) const;
    virtual void computeGradient(void) const;
    virtual void computeHessian(void) const;
    virtual void computeJXdot(void) const;
    virtual void computeJdot(void) const;
    virtual void computeJdotXdot(void) const;

    virtual void doUpdateSize(void);

    // ------------------------ protected static methods ------------------------
  protected:
    inline static int computeDimensionFor(ECartesianDof fixedPosition);

    // ------------------------ private methods ---------------------------------
  private:

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:
    int       _fixedPosition;
    Robot*    _robot;
    Target    _target;

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };

  inline int CoMTrackingFunction::computeDimensionFor(ECartesianDof fixedPosition)
  {
    return (fixedPosition&0x1) + ((fixedPosition>>1)&0x1) + ((fixedPosition>>2)&0x1);
  }
}

#endif	//_OCRACONTROL_TRACKING_FUNCTION_H_

// cmake:sourcegroup=Functions
