#ifndef _OCRABASE_OCRA_DEBUG_H_
#define _OCRABASE_OCRA_DEBUG_H_

#ifdef WIN32
# pragma once
#endif


#include <iosfwd>


#ifdef OCRA_DBG_1
# undef OCRA_DBG_1
#endif
#ifdef OCRA_DBG_2
# undef OCRA_DBG_2
#endif
#ifdef OCRA_DBG_3
# undef OCRA_DBG_3
#endif
#ifdef OCRA_DBG_4
# undef OCRA_DBG_4
#endif
#ifdef OCRA_DBG_5
# undef OCRA_DBG_5
#endif


#ifdef OCRA_DEBUG_LEVEL

# if OCRA_DEBUG_LEVEL > 5

#  error "Debug Level too high"

# else // OCRA_DEBUG_LEVEL > 5

#  if OCRA_DEBUG_LEVEL < 0
#   error "Debug Level must be > 0"
#  endif

#  if OCRA_DEBUG_LEVEL >= 1
namespace ocra_debug { extern std::ofstream ocra_dbg_1; }
#   define OCRA_DBG_1(MSG) ocra_debug::ocra_dbg_1 << MSG << std::endl
#  else
#   define OCRA_DBG_1(MSG)
#  endif

#  if OCRA_DEBUG_LEVEL >= 2
namespace ocra_debug { extern std::ofstream ocra_dbg_2; }
#   define OCRA_DBG_2(MSG) ocra_debug::ocra_dbg_2 << MSG << std::endl
#  else
#   define OCRA_DBG_2(MSG)
#  endif

#  if OCRA_DEBUG_LEVEL >= 3
namespace ocra_debug { extern std::ofstream ocra_dbg_3; }
#   define OCRA_DBG_3(MSG) ocra_debug::ocra_dbg_3 << MSG << std::endl
#  else
#   define OCRA_DBG_3(MSG)
#  endif

#  if OCRA_DEBUG_LEVEL >= 4
namespace ocra_debug { extern std::ofstream ocra_dbg_4; }
#   define OCRA_DBG_4(MSG) ocra_debug::ocra_dbg_4 << MSG << std::endl
#  else
#   define OCRA_DBG_4(MSG)
#  endif

#  if OCRA_DEBUG_LEVEL == 5
namespace ocra_debug { extern std::ofstream ocra_dbg_5; }
#   define OCRA_DBG_5(MSG) ocra_debug::ocra_dbg_5 << MSG << std::endl
#  else
#   define OCRA_DBG_5(MSG)
#  endif

# endif // OCRA_DEBUG_LEVEL > 5

#else // OCRA_DEBUG_LEVEL is defined

#define OCRA_DBG_1(MSG)
#define OCRA_DBG_2(MSG)
#define OCRA_DBG_3(MSG)
#define OCRA_DBG_4(MSG)
#define OCRA_DBG_5(MSG)

#endif // OCRA_DEBUG_LEVEL is defined


#endif //_OCRABASE_OCRA_DEBUG_H_

// cmake:sourcegroup=Utils
