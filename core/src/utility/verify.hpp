#pragma once

#define libhcs_VERIFY(_cond, _ret) \
    do {                           \
        if (_cond) {               \
            (void)0;               \
        } else {                   \
            return _ret;           \
        }                          \
    } while (false)

#define libhcs_VERIFY_LIKELY(_cond, _ret) \
    do {                                  \
        if (_cond) [[likely]] {           \
            (void)0;                      \
        } else {                          \
            return _ret;                  \
        }                                 \
    } while (false)
