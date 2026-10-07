// Links the SDK built from the released source: the board class pulls in the protocol handler
// and the USB transport (libusb), the logging call a plain exported function. Opening a board
// needs hardware, so that path is only linked, never run.
#include <cstdio>

#include <libhcs/board/hpm5321.hpp>
#include <libhcs/logging.hpp>

namespace {

class Callback final : public libhcs::board::Hpm5321::Callback {};

[[maybe_unused]] void open_a_board() {
    Callback callback;
    const libhcs::board::Hpm5321 board{callback};
}

} // namespace

int main(int argc, char**) {
    if (argc > 1)
        open_a_board();
    std::printf(
        "stderr lines dropped: %llu\n",
        static_cast<unsigned long long>(libhcs::host::logging::stderr_lines_dropped()));
    return 0;
}
