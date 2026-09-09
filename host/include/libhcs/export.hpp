#pragma once

#if defined(_MSC_VER)
# ifdef BUILDING_libhcs
#  define libhcs_API __declspec(dllexport)
# elif defined(STATIC_LINKING_libhcs)
#  define libhcs_API
# else
#  define libhcs_API __declspec(dllimport)
# endif
#elif defined(__GNUC__) || defined(__clang__)
# ifdef BUILDING_libhcs
#  define libhcs_API __attribute__((visibility("default")))
# else
#  define libhcs_API
# endif
#else
# define libhcs_API
#endif
