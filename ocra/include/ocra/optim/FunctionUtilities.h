/** @file FunctionUtilities.h
  * @brief Declaration file of the FunctionProperties class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 09/05/04
  *
  * File history:
  *   - 10/06/28 Escande Adrien: rename to FunctionUtilities and move to ocra::utils. Update according to changes in 
  * Function
  */

#ifndef _OCRABASE_FUNCTION_UTILITIES_H_
#define _OCRABASE_FUNCTION_UTILITIES_H_

#ifdef WIN32
# pragma once
#endif

// includes
#include "ocra/optim/Function.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *   a library of classes to write and solve optimization problems dedicated to
  *   the control of multi-body systems. 
  */
namespace ocra
{
  namespace utils
  {
    int  getAddType(const Function& f1, const Function& f2);
    int  getAddContinuityProperty(const Function& f1, const Function& f2);
    bool computeGradient(const Function& f1, const Function& f2);
    bool computeHessian(const Function& f1, const Function& f2);
    //Variable* getConcatenatedVariable(Function& f1, Function& f2);

    int  getOppositeConvexityProperty(const Function& f);

    Variable* createOutputVariable(Function& f);
  };
}

#endif	//_OCRABASE_FUNCTION_UTILITIES_H_

// cmake:sourcegroup=Function

