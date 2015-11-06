#ifndef _OCRA_ASSERT_H_
#define _OCRA_ASSERT_H_

namespace ocra
{
  template<bool condition> struct ocra_static_assert {};

  template<>
  struct ocra_static_assert<true>
  {
    enum
    {
      TYPE_MUST_BE_A_VECTOR_OR_A_DYNAMIC_MATRIX
    };
  };

}

#ifdef _MSC_VER

  #define OCRA_STATIC_ASSERT(OCRA_ASSERT_CONDITION,MSG) \
    {ocra::ocra_static_assert<OCRA_ASSERT_CONDITION ? true : false>::MSG;}

#else

  #define OCRA_STATIC_ASSERT(CONDITION,MSG) \
    if (ocra::ocra_static_assert<CONDITION ? true : false>::MSG) {}

#endif


struct ocra_assertion_fired {};

#undef ocra_assert

#ifndef NDEBUG
# ifndef OCRA_ASSERT_ACTIVE
#  define OCRA_ASSERT_ACTIVE
# endif
#endif

#ifdef OCRA_ASSERT_ACTIVE

# include <cstdio>
# define ocra_assert(ocra_expression) if(!(ocra_expression)) { fprintf(stderr, #ocra_expression"\n"); fflush(stderr); throw ocra_assertion_fired(); }

#else // OCRA_ASSERT_ACTIVE not defined

# define ocra_assert(ocra_expression)     ((void)0)

#endif // OCRA_ASSERT_ACTIVE


#endif // _OCRA_ASSERT_H_


// cmake:sourcegroup=Utils
