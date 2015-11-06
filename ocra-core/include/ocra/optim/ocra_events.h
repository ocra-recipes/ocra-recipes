#ifndef _OCRABASE_OCRA_EVENTS_H_
#define _OCRABASE_OCRA_EVENTS_H_

#ifdef WIN32
# pragma once
#endif

namespace ocra
{
  /** a list of event used in ocra*/
  enum
  {
    EVT_RESIZE = 0xffff,            //> used when a variable changes its size or a function its dimension.
    EVT_CHANGE_DEPENDENCIES,        //> a variable (or a function) changed the variables it was relying on.
    EVT_CHANGE_VALUE,               //> the value of a variable or a function has been changed
    EVT_INVALIDATE,
    EVT_CSTR_CHANGE_BOUNDS_NUMBER   //> a (inequality) constraint changes from single to double bounds or from double to single.
  };
}

#endif //_OCRABASE_OCRA_EVENTS_H_

// cmake:sourcegroup=Utils
