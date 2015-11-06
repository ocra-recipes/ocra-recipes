/** @file ControlEnum.h
  * @brief Some enumerations for the control.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/03/01
  */

#ifndef _OCRACONTROL_CONTROL_ENUM_H_
#define _OCRACONTROL_CONTROL_ENUM_H_

namespace ocra
{
  enum ECartesianDof
  {
    NONE=0,
    X,
    Y,
    XY,
    Z,
    XZ,
    YZ,
    XYZ
  };

  namespace utils
  {
    //! Computes the number of directions specified by two ECartesianDof enums.
    /*!
    Examples:
    computeDimensionFor(NONE, NONE) == 0
    computeDimensionFor(XY, XYZ) == 5
    computeDimensionFor(Z, NONE) == 1
    */
    inline int computeDimensionFor(ECartesianDof fixedPosition, ECartesianDof fixedOrientation)
    {
      return (fixedPosition&0x1) + ((fixedPosition>>1)&0x1) + ((fixedPosition>>2)&0x1)
              +(fixedOrientation&0x1) + ((fixedOrientation>>1)&0x1) + ((fixedOrientation>>2)&0x1);
    }
  }
}

#endif //_OCRACONTROL_CONTROL_ENUM_H_

// cmake:sourcegroup=Api
