// Read or toggle C1E (MSR_POWER_CTL 0x1FC, bit 1) on all online CPUs.
//
//   sudo ./c1e            print per-CPU C1E state
//   sudo ./c1e off        clear bit 1 on every CPU
//   sudo ./c1e on         set bit 1 back
//
// C1E drops voltage/frequency on every C1 entry; waking from it costs a measured
// 15-20 us despite C1_ACPI's nominal 1 us latency (HOST_TUNING 10.9.3). Clearing
// bit 1 keeps plain C1 -- a cheap halt wake -- so this is the runtime equivalent
// of the BIOS "Enhanced C-states" switch, and unlike disabling C1_ACPI per core
// (HOST_TUNING 10.9.4) it leaves cores actually idle in C1 instead of spinning
// in POLL. The MSR is per-core; the setting persists until something rewrites
// it (BIOS handoff, tgl/nvme style drivers do not touch it).

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MSR_POWER_CTL 0x1FC
#define C1E_BIT (1ULL << 1)

static int msr_fd(int cpu, int flags) {
    char path[64];
    snprintf(path, sizeof(path), "/dev/cpu/%d/msr", cpu);
    return open(path, flags);
}

int main(int argc, char** argv) {
    int set_bit = -1;
    if (argc > 1) {
        if (strcmp(argv[1], "off") == 0)
            set_bit = 0;
        else if (strcmp(argv[1], "on") == 0)
            set_bit = 1;
        else {
            fprintf(stderr, "usage: %s [off|on]\n", argv[0]);
            return 2;
        }
    }

    FILE* online = fopen("/sys/devices/system/cpu/online", "re");
    if (online == NULL) {
        perror("/sys/devices/system/cpu/online");
        return 1;
    }
    char mask[256] = {0};
    if (fgets(mask, sizeof(mask), online) == NULL) {
        fprintf(stderr, "empty cpu online mask\n");
        return 1;
    }
    fclose(online);
    mask[strcspn(mask, "\n")] = 0;

    int failures = 0;
    char* cursor = mask;
    while (cursor != NULL && *cursor != 0) {
        int first = (int) strtol(cursor, &cursor, 10);
        int last = first;
        if (*cursor == '-') {
            last = (int) strtol(cursor + 1, &cursor, 10);
        }
        for (int cpu = first; cpu <= last; ++cpu) {
            int fd = msr_fd(cpu, O_RDWR);
            if (fd < 0) {
                fprintf(stderr, "cpu%d: %s\n", cpu, strerror(errno));
                ++failures;
                continue;
            }
            uint64_t value = 0;
            if (pread(fd, &value, sizeof(value), MSR_POWER_CTL) != sizeof(value)) {
                fprintf(stderr, "cpu%d: pread: %s\n", cpu, strerror(errno));
                close(fd);
                ++failures;
                continue;
            }
            if (set_bit >= 0) {
                uint64_t updated =
                    set_bit ? (value | C1E_BIT) : (value & ~C1E_BIT);
                if (pwrite(fd, &updated, sizeof(updated), MSR_POWER_CTL) != sizeof(updated)) {
                    fprintf(stderr, "cpu%d: pwrite: %s\n", cpu, strerror(errno));
                    close(fd);
                    ++failures;
                    continue;
                }
                value = updated;
            }
            printf("cpu%d: MSR_POWER_CTL=0x%04llx C1E=%d\n", cpu, (unsigned long long) value,
                   (int) ((value & C1E_BIT) != 0));
            close(fd);
        }
        if (cursor != NULL && *cursor != 0)
            ++cursor; // skip ','
    }
    return failures != 0 ? 1 : 0;
}
