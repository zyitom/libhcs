#pragma once

#include "core/src/utility/immovable.hpp"
#include "firmware/c_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware {

class App : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<App>;

    App();

    [[noreturn]] void run();
};

inline constinit App::Lazy app;

} // namespace libhcs::firmware
