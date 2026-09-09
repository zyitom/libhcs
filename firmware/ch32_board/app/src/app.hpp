#pragma once

#include "core/src/utility/immovable.hpp"
#include "firmware/ch32_board/app/src/utility/lazy.hpp"

namespace libhcs::firmware {

// Top-level application object for ch32_board, mirroring the mc02 forwarding
// bridge. App::App() brings up the clock tree and every peripheral; App::run()
// is the single-threaded forwarding loop that interleaves the USB SS uplink
// with each CAN/UART transmit. Constructed via the Lazy singleton below so its
// members (which live in the WCH driver objects) are placed deterministically.
class App : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<App>;

    App();

    [[noreturn]] void run();
};

inline constinit App::Lazy app;

} // namespace libhcs::firmware
