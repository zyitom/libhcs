// Links the SDK built from the released source: the Board class pulls in the link
// engine and the USB transport (libusb), the logging call a plain exported
// function. Opening a board needs hardware, so that path is only linked, never
// run.
#include <cstdio>

#include <libhcs/board.hpp>
#include <libhcs/board/hcs_config.hpp>
#include <libhcs/logging.hpp>

namespace {

[[maybe_unused]] void open_a_board() {
    libhcs::Board board{{.board_type = "hpm5321", .serial = "A1B2"}};
    auto& can1 = board.add_can("CAN1", libhcs::board::hcs::kClassic1M);
    (void)can1;
    board.start();
    board.run();
    board.receive();
    board.send();
    board.stop();
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
