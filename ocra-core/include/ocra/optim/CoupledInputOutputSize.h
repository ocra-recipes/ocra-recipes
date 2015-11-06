#ifndef _OCRA_COUPLED_INPUT_OUTPUT_SIZE_H_
#define _OCRA_COUPLED_INPUT_OUTPUT_SIZE_H_

// XXX | WTF is it ?
//     V 
#ifdef WIN32
# pragma once
#endif

namespace ocra
{
  /** A trivial class with a single bool member.
    * 
    * Within ocra, this class is meant to be used as a \a virtual base class to force classes deriving from Function to
    * declare if their dimension is depending of the size of the input variable.
    */
  class CoupledInputOutputSize
  {
  protected:
    CoupledInputOutputSize(bool coupledInputOutputSize)
      : _coupled(coupledInputOutputSize)
    {
    }

    bool inputAndOutputSizesAreCoupled() const
    {
      return _coupled;
    }

  private:
    bool _coupled;
  };
}


#endif //_OCRA_COUPLED_INPUT_OUTPUT_SIZE_H_

// cmake:sourcegroup=Function
