/*
 * Minimal newlib syscall stubs for the CH32H417 bare-metal target.
 *
 * The WCH Debug library (bsp/wch/Debug/debug.c) already provides _write (USART
 * retarget) and _sbrk (heap). newlib's printf/malloc machinery additionally
 * references the syscalls below; we stub them here instead of pulling in
 * nosys.specs, which would collide with debug.c's non-weak _write/_sbrk.
 */

#include <errno.h>
#include <sys/stat.h>

int _close(int fd) {
    (void)fd;
    return -1;
}

int _fstat(int fd, struct stat *st) {
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd) {
    (void)fd;
    return 1;
}

int _lseek(int fd, int ptr, int dir) {
    (void)fd;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int fd, char *buf, int size) {
    (void)fd;
    (void)buf;
    (void)size;
    return 0;
}

int _getpid(void) {
    return 1;
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status) {
    (void)status;
    while (1) {
    }
}
