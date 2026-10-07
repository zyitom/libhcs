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

// Marks a function that never blocks: no lock, no allocation, no system call.
// Where the compiler can check that (clang's function effect analysis, enabled
// with -Wfunction-effects) it does, here and in every caller that makes the
// same promise -- which is how a real-time loop is allowed to call into this
// library at all. Elsewhere it expands to nothing and the promise is only
// documentation.
//
// Probed with __has_cpp_attribute rather than a compiler version: gcc warns
// about attributes it does not know.
#if defined(__has_cpp_attribute)
# if __has_cpp_attribute(clang::nonblocking)
#  define libhcs_NONBLOCKING [[clang::nonblocking]]
# endif
#endif
#ifndef libhcs_NONBLOCKING
# define libhcs_NONBLOCKING
#endif
