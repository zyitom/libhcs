#define UNICODE
#define _UNICODE
#define _WIN32_WINNT 0x0601
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

// Windows SDK headers require windows.h before SetupAPI.
// clang-format off
#include <windows.h>
#include <setupapi.h>
#include <winusb.h>
// clang-format on

// Independent PCAN-Basic ABI adapter for the HCS WinUSB firmware. This library
// never opens a PEAK device or calls the proprietary PCAN_USB kernel driver.
// The initial supported API is CAN FD, as used by EYou RP Series.
typedef uint32_t Status;
enum {
    kOk = 0,
    kOverrun = 2,
    kBusLight = 4,
    kBusHeavy = 8,
    kBusOff = 16,
    kQueueEmpty = 0x20,
    kResource = 0x2000,
    kBadParameter = 0x8000,
    kBadHandle = 0x1400,
    kNotInitialized = 0x4000000,
    kUnsupported = 0x8000000,
    kQueueSize = 4096,
};

typedef struct {
    uint32_t id;
    uint8_t type;
    uint8_t dlc;
    uint8_t data[64];
} MessageFd;

typedef struct {
    uint32_t id;
    uint8_t type;
    uint8_t length;
    uint8_t data[8];
} Message;

typedef struct {
    uint32_t millis;
    uint16_t millis_overflow;
    uint16_t micros;
} Timestamp;

typedef struct {
    uint16_t handle;
    uint8_t device_type;
    uint8_t controller;
    uint32_t features;
    char name[33];
    uint32_t device_id;
    uint32_t condition;
} ChannelInformation;

_Static_assert(sizeof(MessageFd) == 72, "TPCANMsgFD ABI");
_Static_assert(offsetof(MessageFd, data) == 6, "TPCANMsgFD data ABI");
_Static_assert(sizeof(Message) == 16, "TPCANMsg ABI");
_Static_assert(sizeof(ChannelInformation) == 52, "TPCANChannelInformation ABI");

typedef struct {
    MessageFd message;
    uint64_t timestamp;
} Received;

typedef struct {
    Received frames[kQueueSize];
    unsigned head;
    unsigned count;
    Status status;
    BOOL initialized;
    char bitrate[256];
} Channel;

static const GUID kInterfaceGuid = {
    0x74ce6bd6, 0xffae, 0x49c6, {0xac, 0x5a, 0xbc, 0x43, 0xf7, 0xae, 0x2a, 0x19}
};
static SRWLOCK g_api_lock = SRWLOCK_INIT;
static SRWLOCK g_queue_lock = SRWLOCK_INIT;
static Channel g_channels[2];
static HANDLE g_file = INVALID_HANDLE_VALUE;
static WINUSB_INTERFACE_HANDLE g_usb;
static HANDLE g_reader;
static volatile LONG g_stop;
static volatile LONG g_connected;
static unsigned g_channel_count;
static HMODULE g_module;

static uint16_t load16(const uint8_t* p) { return (uint16_t)(p[0] | (uint16_t)p[1] << 8); }

static uint32_t load32(const uint8_t* p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void store16(uint8_t* p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void store32(uint8_t* p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static unsigned payload_length(uint8_t dlc) {
    static const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return dlc < 16 ? lengths[dlc] : 0;
}

static int channel_index(uint16_t handle) {
    return handle >= 0x51 && handle <= 0x52 ? handle - 0x51 : -1;
}

static BOOL find_device(wchar_t* path, size_t capacity, unsigned* channel_count) {
    HDEVINFO devices =
        SetupDiGetClassDevsW(&kInterfaceGuid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (devices == INVALID_HANDLE_VALUE)
        return FALSE;
    BOOL found = FALSE;
    for (DWORD index = 0; !found; ++index) {
        SP_DEVICE_INTERFACE_DATA item = {0};
        item.cbSize = sizeof item;
        if (!SetupDiEnumDeviceInterfaces(devices, NULL, &kInterfaceGuid, index, &item))
            break;
        DWORD size = 0;
        SetupDiGetDeviceInterfaceDetailW(devices, &item, NULL, 0, &size, NULL);
        if (size < sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W))
            continue;
        PSP_DEVICE_INTERFACE_DETAIL_DATA_W detail = malloc(size);
        if (!detail)
            break;
        detail->cbSize = sizeof *detail;
        if (SetupDiGetDeviceInterfaceDetailW(devices, &item, detail, size, NULL, NULL)) {
            wchar_t upper[1024];
            size_t length = wcslen(detail->DevicePath);
            if (length < capacity && length < sizeof upper / sizeof upper[0]) {
                wcscpy(upper, detail->DevicePath);
                CharUpperBuffW(upper, (DWORD)length);
                if (wcsstr(upper, L"VID_A511&PID_F531") || wcsstr(upper, L"VID_A511&PID_F532")) {
                    wcscpy(path, detail->DevicePath);
                    *channel_count = wcsstr(upper, L"PID_F532") ? 2 : 1;
                    found = TRUE;
                }
            }
        }
        free(detail);
    }
    SetupDiDestroyDeviceInfoList(devices);
    return found;
}

static BOOL write_pipe(uint8_t pipe, uint8_t* bytes, ULONG length) {
    ULONG transferred = 0;
    return WinUsb_WritePipe(g_usb, pipe, bytes, length, &transferred, NULL)
        && transferred == length;
}

static BOOL set_driver_loaded(BOOL loaded) {
    uint8_t bytes[16] = {0};
    bytes[1] = loaded ? 1 : 0;
    WINUSB_SETUP_PACKET setup = {0x43, 2, 5, 0, sizeof bytes};
    ULONG transferred = 0;
    return WinUsb_ControlTransfer(g_usb, setup, bytes, sizeof bytes, &transferred, NULL)
        && transferred == sizeof bytes;
}

static BOOL send_command(unsigned channel, uint16_t opcode, const uint8_t* args) {
    uint8_t bytes[8] = {0};
    store16(bytes, (uint16_t)(opcode | channel << 12));
    if (args)
        memcpy(bytes + 2, args, 6);
    return write_pipe(1, bytes, sizeof bytes);
}

static void clear_queue(unsigned index) {
    AcquireSRWLockExclusive(&g_queue_lock);
    g_channels[index].head = 0;
    g_channels[index].count = 0;
    g_channels[index].status = 0;
    ReleaseSRWLockExclusive(&g_queue_lock);
}

static void receive_packet(const uint8_t* bytes, size_t length) {
    for (size_t offset = 0; offset + 4 <= length;) {
        const uint8_t* p = bytes + offset;
        unsigned size = load16(p);
        unsigned type = load16(p + 2);
        if (!size)
            break;
        if (size < 4 || size > length - offset)
            break;
        offset += size;
        if (type == 1 && size >= 28) {
            unsigned index = p[20] & 15;
            unsigned dlc = p[20] >> 4;
            unsigned flags = load16(p + 22);
            unsigned payload = flags & 1 ? 0 : payload_length((uint8_t)dlc);
            if (index >= g_channel_count || payload > size - 28)
                continue;
            // Unrequested hardware echoes are transport confirmations only.
            if ((flags & 4) && !(flags & 0x80))
                continue;
            Received frame = {0};
            frame.message.id = load32(p + 24);
            frame.message.dlc = (uint8_t)dlc;
            frame.message.type = (uint8_t)((flags & 3) | ((flags & 0x70) >> 2));
            if (flags & 4)
                frame.message.type |= 0x20;
            memcpy(frame.message.data, p + 28, payload);
            frame.timestamp = load32(p + 4) | (uint64_t)load32(p + 8) << 32;
            AcquireSRWLockExclusive(&g_queue_lock);
            Channel* channel = &g_channels[index];
            if (channel->count == kQueueSize) {
                channel->status |= kOverrun;
            } else {
                channel->frames[(channel->head + channel->count) % kQueueSize] = frame;
                ++channel->count;
            }
            ReleaseSRWLockExclusive(&g_queue_lock);
        } else if (type == 3 && size >= 16 && (p[12] & 15) < g_channel_count) {
            Status status = p[12] & 0x80 ? kBusOff
                          : p[12] & 0x20 ? kBusHeavy
                          : p[12] & 0x40 ? kBusLight
                                         : kOk;
            AcquireSRWLockExclusive(&g_queue_lock);
            g_channels[p[12] & 15].status = status;
            ReleaseSRWLockExclusive(&g_queue_lock);
        }
    }
}

static DWORD WINAPI receive_thread(void* unused) {
    (void)unused;
    uint8_t packet[512];
    HMODULE reference = NULL;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCWSTR)g_module, &reference);
    while (!InterlockedCompareExchange(&g_stop, 0, 0)) {
        ULONG length = 0;
        if (WinUsb_ReadPipe(g_usb, 0x82, packet, sizeof packet, &length, NULL)) {
            receive_packet(packet, length);
        } else {
            DWORD error = GetLastError();
            if (error == ERROR_SEM_TIMEOUT || error == ERROR_OPERATION_ABORTED)
                continue;
            InterlockedExchange(&g_connected, 0);
            break;
        }
    }
    if (reference)
        FreeLibraryAndExitThread(reference, 0);
    return 0;
}

static void close_device(void) {
    if (g_usb && InterlockedCompareExchange(&g_connected, 0, 0))
        set_driver_loaded(FALSE);
    if (g_reader) {
        InterlockedExchange(&g_stop, 1);
        WinUsb_AbortPipe(g_usb, 0x82);
        WaitForSingleObject(g_reader, INFINITE);
        CloseHandle(g_reader);
        g_reader = NULL;
    }
    if (g_usb) {
        WinUsb_Free(g_usb);
        g_usb = NULL;
    }
    if (g_file != INVALID_HANDLE_VALUE) {
        CloseHandle(g_file);
        g_file = INVALID_HANDLE_VALUE;
    }
    InterlockedExchange(&g_connected, 0);
}

static BOOL open_device(void) {
    if (g_usb)
        return InterlockedCompareExchange(&g_connected, 0, 0) != 0;
    wchar_t path[1024];
    if (!find_device(path, sizeof path / sizeof path[0], &g_channel_count))
        return FALSE;
    g_file = CreateFileW(
        path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
    if (g_file == INVALID_HANDLE_VALUE)
        return FALSE;
    if (!WinUsb_Initialize(g_file, &g_usb)) {
        close_device();
        return FALSE;
    }
    if (!set_driver_loaded(TRUE)) {
        close_device();
        return FALSE;
    }
    ULONG timeout = 50;
    WinUsb_SetPipePolicy(g_usb, 0x82, PIPE_TRANSFER_TIMEOUT, sizeof timeout, &timeout);
    timeout = 1000;
    for (unsigned i = 1; i <= 3; ++i)
        WinUsb_SetPipePolicy(g_usb, (UCHAR)i, PIPE_TRANSFER_TIMEOUT, sizeof timeout, &timeout);
    InterlockedExchange(&g_stop, 0);
    InterlockedExchange(&g_connected, 1);
    g_reader = CreateThread(NULL, 0, receive_thread, NULL, 0, NULL);
    if (!g_reader) {
        close_device();
        return FALSE;
    }
    return TRUE;
}

static BOOL timing_value(const char* text, const char* key, unsigned* value) {
    size_t key_length = strlen(key);
    const char* p = text;
    while ((p = strstr(p, key)) != NULL) {
        if ((p == text || p[-1] == ',' || p[-1] == ' ') && p[key_length] == '=') {
            char* end = NULL;
            unsigned long number = strtoul(p + key_length + 1, &end, 10);
            if (end == p + key_length + 1 || number > 80000000UL)
                return FALSE;
            *value = (unsigned)number;
            return TRUE;
        }
        ++p;
    }
    return FALSE;
}

static BOOL phase_arguments(const char* text, const char* prefix, uint8_t* args, BOOL data) {
    static const char* suffixes[] = {"brp", "tseg1", "tseg2", "sjw"};
    unsigned values[4];
    for (unsigned i = 0; i < 4; ++i) {
        char key[32];
        snprintf(key, sizeof key, "%s_%s", prefix, suffixes[i]);
        if (!timing_value(text, key, &values[i]) || !values[i])
            return FALSE;
    }
    if (values[0] > 512 || values[1] > (data ? 32U : 256U) || values[2] > (data ? 16U : 128U)
        || values[3] > values[2] || values[3] > (data ? 16U : 128U))
        return FALSE;
    args[0] = data ? 0 : 96;
    args[1] = (uint8_t)(values[3] - 1);
    args[2] = (uint8_t)(values[2] - 1);
    args[3] = (uint8_t)(values[1] - 1);
    store16(args + 4, (uint16_t)(values[0] - 1));
    return TRUE;
}

Status WINAPI CAN_InitializeFD(uint16_t handle, const char* bitrate) {
    int index = channel_index(handle);
    uint8_t slow[6], fast[6];
    unsigned clock = 0;
    if (index < 0)
        return kBadHandle;
    if (!bitrate || strlen(bitrate) >= sizeof g_channels[0].bitrate
        || !phase_arguments(bitrate, "nom", slow, FALSE)
        || !phase_arguments(bitrate, "data", fast, TRUE)
        || !(
            (timing_value(bitrate, "f_clock_mhz", &clock) && clock == 80)
            || (timing_value(bitrate, "f_clock", &clock) && clock == 80000000)))
        return kBadParameter;
    AcquireSRWLockExclusive(&g_api_lock);
    Status result = kResource;
    if (open_device() && (unsigned)index < g_channel_count) {
        uint8_t options[6] = {5, 0, 0, 0, 0, 0};
        clear_queue((unsigned)index);
        if (send_command((unsigned)index, 1, NULL) && send_command((unsigned)index, 4, slow)
            && send_command((unsigned)index, 5, fast) && send_command((unsigned)index, 11, options)
            && send_command((unsigned)index, 2, NULL)) {
            g_channels[index].initialized = TRUE;
            strcpy(g_channels[index].bitrate, bitrate);
            result = kOk;
        }
    }
    ReleaseSRWLockExclusive(&g_api_lock);
    return result;
}

Status WINAPI CAN_Uninitialize(uint16_t handle) {
    int index = channel_index(handle);
    if (handle && index < 0)
        return kBadHandle;
    AcquireSRWLockExclusive(&g_api_lock);
    for (unsigned i = 0; i < 2; ++i) {
        if (handle && i != (unsigned)index)
            continue;
        if (g_usb && g_channels[i].initialized)
            send_command(i, 1, NULL);
        g_channels[i].initialized = FALSE;
        clear_queue(i);
    }
    if (!g_channels[0].initialized && !g_channels[1].initialized)
        close_device();
    ReleaseSRWLockExclusive(&g_api_lock);
    return kOk;
}

Status WINAPI CAN_Reset(uint16_t handle) {
    int index = channel_index(handle);
    if (index < 0)
        return kBadHandle;
    AcquireSRWLockExclusive(&g_api_lock);
    Status status = kNotInitialized;
    if (g_channels[index].initialized && g_usb) {
        uint8_t args[6] = {0};
        status = send_command((unsigned)index, 9, args) ? kOk : kResource;
        clear_queue((unsigned)index);
    }
    ReleaseSRWLockExclusive(&g_api_lock);
    return status;
}

Status WINAPI CAN_GetStatus(uint16_t handle) {
    int index = channel_index(handle);
    if (index < 0)
        return kBadHandle;
    AcquireSRWLockExclusive(&g_api_lock);
    Status status = kNotInitialized;
    if (g_channels[index].initialized) {
        AcquireSRWLockShared(&g_queue_lock);
        status =
            InterlockedCompareExchange(&g_connected, 0, 0) ? g_channels[index].status : kBusOff;
        ReleaseSRWLockShared(&g_queue_lock);
    }
    ReleaseSRWLockExclusive(&g_api_lock);
    return status;
}

Status WINAPI CAN_ReadFD(uint16_t handle, MessageFd* message, uint64_t* timestamp) {
    int index = channel_index(handle);
    if (index < 0)
        return kBadHandle;
    if (!message)
        return kBadParameter;
    AcquireSRWLockExclusive(&g_api_lock);
    Status status = kNotInitialized;
    if (g_channels[index].initialized) {
        AcquireSRWLockExclusive(&g_queue_lock);
        Channel* channel = &g_channels[index];
        status = kQueueEmpty;
        if (channel->count) {
            const Received* frame = &channel->frames[channel->head];
            *message = frame->message;
            if (timestamp)
                *timestamp = frame->timestamp;
            channel->head = (channel->head + 1) % kQueueSize;
            --channel->count;
            status = kOk;
        }
        ReleaseSRWLockExclusive(&g_queue_lock);
    }
    ReleaseSRWLockExclusive(&g_api_lock);
    return status;
}

Status WINAPI CAN_WriteFD(uint16_t handle, MessageFd* message) {
    int index = channel_index(handle);
    if (index < 0)
        return kBadHandle;
    if (!message || message->dlc > 15 || (message->type & 0xc0)
        || ((message->type & 2) ? message->id > 0x1fffffff : message->id > 0x7ff)
        || (!(message->type & 4) && message->dlc > 8)
        || ((message->type & 1) && (message->type & 4)))
        return kBadParameter;
    unsigned payload = message->type & 1 ? 0 : payload_length(message->dlc);
    unsigned size = (20 + payload + 3) & ~3U;
    uint8_t packet[84] = {0};
    store16(packet, (uint16_t)size);
    store16(packet + 2, 0x1000);
    packet[12] = (uint8_t)(message->dlc << 4 | index);
    unsigned flags = (message->type & 3) | ((message->type & 0x1c) << 2);
    if (message->type & 0x20)
        flags |= 0x80;
    store16(packet + 14, (uint16_t)flags);
    store32(packet + 16, message->id);
    memcpy(packet + 20, message->data, payload);
    AcquireSRWLockExclusive(&g_api_lock);
    Status status = kNotInitialized;
    if (g_channels[index].initialized && g_usb)
        status = write_pipe((uint8_t)(index + 2), packet, size) ? kOk : kResource;
    ReleaseSRWLockExclusive(&g_api_lock);
    return status;
}

static Status copy_value(void* out, uint32_t size, const void* value, size_t length) {
    if (!out || length > size)
        return kBadParameter;
    memcpy(out, value, length);
    return kOk;
}

Status WINAPI CAN_GetValue(uint16_t handle, uint8_t parameter, void* out, uint32_t size) {
    static const char version[] = "libhcs PCAN-Basic compatibility 0.1";
    static const char name[] = "HCS CAN FD Bridge";
    static const char driver[] = "libhcs WinUSB transport 0.1";
    if (parameter == 5)
        return copy_value(out, size, version, sizeof version);
    if (parameter == 6)
        return copy_value(out, size, driver, sizeof driver);
    if (parameter == 14)
        return copy_value(out, size, name, sizeof name);
    wchar_t path[1024];
    unsigned count = 0;
    find_device(path, sizeof path / sizeof path[0], &count);
    if (parameter == 0x2a) {
        uint32_t value = count;
        return copy_value(out, size, &value, sizeof value);
    }
    if (parameter == 0x2b) {
        ChannelInformation channels[2] = {0};
        for (unsigned i = 0; i < count; ++i) {
            channels[i].handle = (uint16_t)(0x51 + i);
            channels[i].device_type = 5;
            channels[i].controller = (uint8_t)i;
            channels[i].features = 1;
            strcpy(channels[i].name, name);
            channels[i].device_id = UINT32_MAX;
            channels[i].condition = 1;
        }
        return copy_value(out, size, channels, count * sizeof channels[0]);
    }
    int index = channel_index(handle);
    if (index < 0 || (unsigned)index >= count)
        return kBadHandle;
    uint32_t value = 0;
    switch (parameter) {
    case 1: value = UINT32_MAX; break;
    case 0x0d: value = 1; break;
    case 0x16: value = 1; break;
    case 0x17: {
        static const char firmware[] = "HCS CAN FD Bridge 1.0";
        return copy_value(out, size, firmware, sizeof firmware);
    }
    case 0x22: {
        AcquireSRWLockShared(&g_api_lock);
        Status status =
            copy_value(out, size, g_channels[index].bitrate, strlen(g_channels[index].bitrate) + 1);
        ReleaseSRWLockShared(&g_api_lock);
        return status;
    }
    default: return kUnsupported;
    }
    return copy_value(out, size, &value, sizeof value);
}

Status WINAPI CAN_GetErrorText(Status error, uint16_t language, char* out) {
    (void)language;
    if (!out)
        return kBadParameter;
    const char* text = error == kOk             ? "No error"
                     : error == kQueueEmpty     ? "Receive queue is empty"
                     : error == kBadHandle      ? "Invalid HCS bridge channel"
                     : error == kBadParameter   ? "Unsupported parameter value"
                     : error == kNotInitialized ? "CAN FD channel is not initialized"
                     : error == kUnsupported
                         ? "This API operation is not implemented by the HCS bridge"
                         : "HCS WinUSB transport error";
    snprintf(out, 256, "%s", text);
    return kOk;
}

Status WINAPI CAN_Initialize(uint16_t h, uint16_t rate, uint8_t type, uint32_t port, uint16_t irq) {
    (void)h;
    (void)rate;
    (void)type;
    (void)port;
    (void)irq;
    return kUnsupported;
}

Status WINAPI CAN_Read(uint16_t h, Message* message, Timestamp* timestamp) {
    (void)h;
    (void)message;
    (void)timestamp;
    return kUnsupported;
}

Status WINAPI CAN_Write(uint16_t h, Message* message) {
    (void)h;
    (void)message;
    return kUnsupported;
}

Status WINAPI CAN_FilterMessages(uint16_t h, uint32_t from, uint32_t to, uint8_t mode) {
    (void)h;
    (void)from;
    (void)to;
    (void)mode;
    return kUnsupported;
}

Status WINAPI CAN_SetValue(uint16_t h, uint8_t parameter, void* value, uint32_t size) {
    (void)h;
    (void)parameter;
    (void)value;
    (void)size;
    return kUnsupported;
}

Status WINAPI CAN_LookUpChannel(const char* parameters, uint16_t* handle) {
    (void)parameters;
    (void)handle;
    return kUnsupported;
}

BOOL WINAPI DllMain(HINSTANCE instance, DWORD reason, void* reserved) {
    (void)reserved;
    if (reason == DLL_PROCESS_ATTACH) {
        g_module = instance;
        DisableThreadLibraryCalls(instance);
    }
    return TRUE;
}
