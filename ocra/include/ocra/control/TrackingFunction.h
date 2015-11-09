/** @file TrackingFunction.h
  * @brief Declaration file of the TrackingFunction class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/02/18
  *
  * File history:
  *  - 10/08/05: Escande Adrien - Switch to Eigen.
  *  - 11/02/02: Escande Adrien - New implementation based on Feature.
  */

#ifndef _OCRACONTROL_TRACKING_FUNCTION_H_
#define _OCRACONTROL_TRACKING_FUNCTION_H_

// includes
#include "ocra/optim/Function.h"
#include "ocra/control/ControlEnum.h"
#include "ocra/control/ControlFrame.h"
#include <Eigen/Lgsm>


namespace ocra
{
  class Feature;
}

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class TrackingFunction
    *	@brief %TrackingFunction class.
    *	@warning None
    *  
    * Description
    */
  class TrackingFunction : public Function
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
    TrackingFunction(TrackingFunction&);
    TrackingFunction& operator= (const TrackingFunction&);
  protected:
  public:
    TrackingFunction(const SegmentFrame& segmentFrame, ECartesianDof fixedPosition = XYZ, ECartesianDof fixedOrientation = XYZ);
    TrackingFunction(const CoMFrame& comFrame, ECartesianDof fixedPosition = XYZ);
    ~TrackingFunction();

    // ------------------------ public interface --------------------------------
  public:
    void setDesiredDisplacement(const Eigen::Displacementd& Hdes);
    void setDesiredVelocity(const Eigen::Twistd& Tdes);

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    void updateValue() const;
    void updateJacobian() const;
    void updateFdot() const;

    //void doUpdateSize(void);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:
    ECartesianDof       _fixedPosition;
    ECartesianDof       _fixedOrientation;

    TargetFrame         _target;
    Feature*            _robotFeature;
    Feature*            _targetFeature;

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };

}

#endif	//_OCRACONTROL_TRACKING_FUNCTION_H_

// cmake:sourcegroup=Functions
