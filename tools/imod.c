// Read/rewrite the xHCI Interrupt Moderation register (IMOD, interrupter 0)
// of a PCI host controller, in 256 ns units. Requires root.
//
//   sudo ./imod                      # print current IMOD (ns)
//   sudo ./imod <BDF>                # explicit device, e.g. 0000:00:14.0
//   sudo ./imod <BDF> 0              # disable moderation (interrupt per event)
//   sudo ./imod <BDF> 160            # restore the 40.96 us default
//
// The driver does not rewrite IMOD unless the controller is reinitialised, so
// an A/B latency test can flip it between runs without unbinding anything.
// Register layout: PCI bar 0 -> Capability registers: CAPLENGTH @0x00 (u8),
// HCSParams2 @0x08, RTSoFF @0x18 (u32, mask 0xFFFFE0). Runtime registers:
// MFINDEX @RTS+0x00, then per-interrupter IR0: IMAN @RTS+0x20, IMOD @RTS+0x24.

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

int main(int argc, char** argv) {
    const char* bdf = (argc > 1 && argv[1][0] != '\0' && strchr(argv[1], ':') != NULL)
                          ? argv[1]
                          : "0000:00:14.0";
    uint32_t value_ns = 40000;
    int write_it = 0;
    if (argc > 2) {
        value_ns = (uint32_t) strtoul(argv[2], NULL, 0);
        write_it = 1;
    }

    char path[128];
    snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource0", bdf);
    int fd = open(path, O_RDWR | O_SYNC);
    if (fd < 0) {
        perror(path);
        return 1;
    }
    // Map enough for cap regs + runtime regs (well inside one page).
    volatile uint8_t* base =
        (volatile uint8_t*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    const uint8_t caplength = base[0x00] & 0xFC;
    uint32_t rtsoff;
    memcpy(&rtsoff, (const void*) (base + 0x18), 4);
    const uint32_t rts = rtsoff & 0xFFFFFE0u;
    volatile uint32_t* imod = (volatile uint32_t*) (base + rts + 0x24);
    volatile uint32_t* iman = (volatile uint32_t*) (base + rts + 0x20);

    printf("%s CAPLENGTH=0x%02x RTS=0x%04x IMAN=0x%08x IMOD=%u (%u ns)\n", bdf, caplength,
           rts, *iman, *imod, *imod * 256);

    if (write_it) {
        const uint32_t units = value_ns / 256;
        *imod = units;
        printf("wrote IMOD=%u (%u ns), readback %u (%u ns)\n", units, units * 256, *imod,
               *imod * 256);
    }
    munmap((void*)base, 0x1000);
    close(fd);
    return 0;
}
