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

#include <ocra/util/StringUtilities.h>
#include <iostream>

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

    inline int computeDimensionFor(ECartesianDof cDoF)
    {
        if  (cDoF == ECartesianDof::X || cDoF == ECartesianDof::Y || cDoF == ECartesianDof::Z) {
            return 1;
        } else if (cDoF == ECartesianDof::XY || cDoF == ECartesianDof::YZ || cDoF == ECartesianDof::XZ) {
            return 2;
        } else if (cDoF == ECartesianDof::XYZ) {
            return 3;
        } else {
            return 0;
        }
    }

    inline ECartesianDof cartesianDofFromString(const std::string& dofString)
    {
        std::string tmpString = util::convertToUpperCase(dofString);
        if      (tmpString == "NONE")   {return ECartesianDof::NONE;}
        else if (tmpString == "X")      {return ECartesianDof::X;}
        else if (tmpString == "Y")      {return ECartesianDof::Y;}
        else if (tmpString == "XY")     {return ECartesianDof::XY;}
        else if (tmpString == "Z")      {return ECartesianDof::Z;}
        else if (tmpString == "XZ")     {return ECartesianDof::XZ;}
        else if (tmpString == "YZ")     {return ECartesianDof::YZ;}
        else if (tmpString == "XYZ")    {return ECartesianDof::XYZ;}
        else {
            std::cout << "[WARNING] (cartesianDofFromString): Couldn't find a match for: " << tmpString << std::endl;
            return ECartesianDof::NONE;
        }
    }

    inline std::string cartesianDofToString(const ECartesianDof dofEnum)
    {
        std::string tmpString;
        switch (dofEnum) {
            case ECartesianDof::NONE:   {tmpString = "NONE";}   break;
            case ECartesianDof::X:      {tmpString = "X";}      break;
            case ECartesianDof::Y:      {tmpString = "Y";}      break;
            case ECartesianDof::XY:     {tmpString = "XY";}     break;
            case ECartesianDof::Z:      {tmpString = "Z";}      break;
            case ECartesianDof::XZ:     {tmpString = "XZ";}     break;
            case ECartesianDof::YZ:     {tmpString = "YZ";}     break;
            case ECartesianDof::XYZ:    {tmpString = "XYZ";}    break;
            default:                    {tmpString = "NONE";}   break;
        }
        return tmpString;
    }
  }
}

#endif //_OCRACONTROL_CONTROL_ENUM_H_

// cmake:sourcegroup=Api
