// Throwaway topology probe: with all four CAN ports on one bus, which
// controllers actually see a frame transmitted by A.CAN1?
#include <atomic>
#include <chrono>
#include <cstdio>
#include <dirent.h>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <libhcs/board/hpm5321.hpp>


// CAN ports are named as the silkscreen on THAT board. 5321 prints CAN1/CAN2.
// See libhcs/board/hcs_can_port.hpp.
using libhcs::board::hcs::CanPort;

namespace {
struct Counter : libhcs::board::Hpm5321::Callback {
    const char* name;
    std::atomic<int> c0{0}, c1{0};
    std::atomic<uint32_t> ts0{0}, ts1{0};
    explicit Counter(const char* n) : name(n) {}
    void can_receive(CanPort port, const libhcs::data::CanDataView& d) override {
        switch (port) {
        case CanPort::kCan1: {
            if (d.can_id != 0x321) return;
            c0.fetch_add(1);
            if (d.timestamp_us) ts0.store(*d.timestamp_us);
            break;
        }
        case CanPort::kCan2: {
            if (d.can_id != 0x321) return;
            c1.fetch_add(1);
            if (d.timestamp_us) ts1.store(*d.timestamp_us);
            break;
        }
        default: break;
        }
    }
};
std::vector<std::string> boards() {
    std::vector<std::string> f;
    DIR* dir = opendir("/sys/bus/usb/devices");
    while (const dirent* e = readdir(dir)) {
        std::string b = std::string{"/sys/bus/usb/devices/"} + e->d_name;
        auto rd = [&](const char* l) { std::string p = b + "/" + l; FILE* fp = fopen(p.c_str(), "re");
            if (!fp) return std::string{}; char buf[128]{}; if (!fgets(buf, sizeof buf, fp)) { fclose(fp); return std::string{}; }
            fclose(fp); std::string v{buf}; while (!v.empty() && (v.back()=='\n'||v.back()=='\r')) v.pop_back(); return v; };
        if (rd("idVendor") == "a511" && rd("idProduct") == "5322") { auto s = rd("serial"); if (!s.empty()) f.push_back(s); }
    }
    closedir(dir); std::sort(f.begin(), f.end()); return f;
}
}

int main() {
    auto s = boards();
    if (s.size() < 2) { printf("need 2 boards\n"); return 1; }
    Counter ca{"A"}, cb{"B"};
    libhcs::board::Hpm5321 a{ca, s[0]}, b{cb, s[1]};
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    const std::array<std::byte, 8> payload{};
    for (int i = 0; i < 20; i++) {
        { auto tx = a.start_transmit(); tx.can_transmit(
            CanPort::kCan1, {.can_id = 0x321, .can_data = payload, .is_fdcan = true}); }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{300});
    printf("sent 20 frames from A.CAN1 (id 0x321)\n");
    printf("  A.CAN1 rx=%d  A.CAN2 rx=%d   <- transmitting board self-reception\n",
           ca.c0.load(), ca.c1.load());
    printf("  B.CAN1 rx=%d  B.CAN2 rx=%d\n", cb.c0.load(), cb.c1.load());
    printf("  last hw timestamps us: A.CAN2=%u  B.CAN1=%u  B.CAN2=%u\n",
           ca.ts1.load(), cb.ts0.load(), cb.ts1.load());
    return 0;
}
