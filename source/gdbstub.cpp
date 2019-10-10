#include <algorithm>
#include <numeric>
#include <vector>
#include <iostream>
#include <string.h>
#include <malloc.h>
#include <map>
#include <fmt/format.h>
#include <pthread.h>
#include <switch.h>

#include "USBSerial.h"

#define LOG_ERROR(...) GDBStub::FormatMsg(0, __VA_ARGS__)
#define LOG_INFO(...) GDBStub::FormatMsg(1, __VA_ARGS__)
#define LOG_DEBUG(...) GDBStub::FormatMsg(2, __VA_ARGS__)

extern "C" Handle g_procHandle;
extern "C" u64 g_nroAddr; 
extern "C" void debug_message(int level, const char *message);

/// Thread exception dump structure.
struct ThreadExceptionDump_t{
    u32 error_desc;             ///< See \ref ThreadExceptionDesc.
    u32 pad[3];

    CpuRegister cpu_gprs[29];   ///< GPRs 0..28. Note: also contains AArch32 registers.
    CpuRegister fp;             ///< Frame pointer.
    CpuRegister lr;             ///< Link register.
    CpuRegister sp;             ///< Stack pointer.
    CpuRegister pc;             ///< Program counter (elr_el1).

    u32 fpsr;
    u32 fpcr;

    FpuRegister fpu_gprs[32];   ///< 32 general-purpose NEON registers.

    u32 pstate;                 ///< pstate & 0xFF0FFE20
    u32 afsr0;
    u32 afsr1;
    u32 esr;

    CpuRegister far;            ///< Fault Address Register.
} PACKED;

namespace GDBStub {
namespace {
constexpr int GDB_BUFFER_SIZE = 10000;

constexpr char GDB_STUB_START = '$';
constexpr char GDB_STUB_END = '#';
constexpr char GDB_STUB_ACK = '+';
constexpr char GDB_STUB_NACK = '-';

#ifndef SIGTRAP
constexpr u32 SIGTRAP = 5;
#endif

#ifndef SIGTERM
constexpr u32 SIGTERM = 15;
#endif

constexpr u32 FP_REGISTER = 29;
constexpr u32 LR_REGISTER = 30;
constexpr u32 SP_REGISTER = 31;
constexpr u32 PC_REGISTER = 32;
constexpr u32 PSTATE_REGISTER = 33;
constexpr u32 UC_ARM64_REG_Q0 = 34;
constexpr u32 FPSR_REGISTER = 66;
constexpr u32 FPCR_REGISTER = 67;

// For sample XML files see the GDB source /gdb/features
// GDB also wants the l character at the start
// This XML defines what the registers are for this specific ARM device
constexpr char target_xml[] =
    R"(l<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target version="1.0">
	<feature name="org.gnu.gdb.aarch64.core">
		<flags id="pstate_flags" size="4">
			<field name="SP" start="0" end="0"/>
			<field name="" start="1" end="1"/>
			<field name="EL" start="2" end="3"/>
			<field name="nRW" start="4" end="4"/>
			<field name="" start="5" end="5"/>
			<field name="F" start="6" end="6"/>
			<field name="I" start="7" end="7"/>
			<field name="A" start="8" end="8"/>
			<field name="D" start="9" end="9"/>
			<field name="IL" start="20" end="20"/>
			<field name="SS" start="21" end="21"/>
			<field name="V" start="28" end="28"/>
			<field name="C" start="29" end="29"/>
			<field name="Z" start="30" end="30"/>
			<field name="N" start="31" end="31"/>
		</flags>
		<reg name="x0" bitsize="64"/>
		<reg name="x1" bitsize="64"/>
		<reg name="x2" bitsize="64"/>
		<reg name="x3" bitsize="64"/>
		<reg name="x4" bitsize="64"/>
		<reg name="x5" bitsize="64"/>
		<reg name="x6" bitsize="64"/>
		<reg name="x7" bitsize="64"/>
		<reg name="x8" bitsize="64"/>
		<reg name="x9" bitsize="64"/>
		<reg name="x10" bitsize="64"/>
		<reg name="x11" bitsize="64"/>
		<reg name="x12" bitsize="64"/>
		<reg name="x13" bitsize="64"/>
		<reg name="x14" bitsize="64"/>
		<reg name="x15" bitsize="64"/>
		<reg name="x16" bitsize="64"/>
		<reg name="x17" bitsize="64"/>
		<reg name="x18" bitsize="64"/>
		<reg name="x19" bitsize="64"/>
		<reg name="x20" bitsize="64"/>
		<reg name="x21" bitsize="64"/>
		<reg name="x22" bitsize="64"/>
		<reg name="x23" bitsize="64"/>
		<reg name="x24" bitsize="64"/>
		<reg name="x25" bitsize="64"/>
		<reg name="x26" bitsize="64"/>
		<reg name="x27" bitsize="64"/>
		<reg name="x28" bitsize="64"/>
		<reg name="x29" bitsize="64"/>
		<reg name="x30" bitsize="64"/>
		<reg name="sp" bitsize="64" type="data_ptr"/>
		<reg name="pc" bitsize="64" type="code_ptr"/>
		<reg name="pstate" bitsize="32" type="pstate_flags"/>
	</feature>
	<feature name="org.gnu.gdb.aarch64.fpu">
  </feature>
</target>
)";

u8 command_buffer[GDB_BUFFER_SIZE];
u32 command_length = 0;

u32 latest_signal = 0;
bool first_exception = true;
bool single_stepping = false;
struct USBSerial_t* serial;

//Kernel::Thread* current_thread = nullptr;
//u32 current_core = 0;

/// Breakpoint Method
enum class BreakpointType {
    None,    ///< None
    Execute, ///< Execution Breakpoint
    Read,    ///< Read Breakpoint
    Write,   ///< Write Breakpoint
    Access   ///< Access (R/W) Breakpoint
};

struct BreakpointAddress_t {
    u64 address;
    BreakpointType type;
};

struct Breakpoint_t {
    bool active;
    u64 addr;
    u64 len;
    std::array<u8, 4> inst;
};

using BreakpointMap = std::map<u64, Breakpoint_t>;
BreakpointMap breakpoints_execute;
BreakpointMap breakpoints_read;
BreakpointMap breakpoints_write;

union NZCV {
    struct {
        unsigned res0 : 28;
        unsigned V : 1;
        unsigned C : 1;
        unsigned Z : 1;
        unsigned N : 1;
    };
    u32 reg;
};
} // Anonymous namespace

template<typename... Args>
void FormatMsg(int level, const char* fmt, Args... args)
{
    std::string msg = fmt::format(fmt, args...);
    debug_message(level, msg.c_str());
}

void Init(struct USBSerial_t* s) {
    breakpoints_execute.clear();
    breakpoints_read.clear();
    breakpoints_write.clear();

    memset(command_buffer, 0, sizeof(command_buffer));
    command_length = 0;

    latest_signal = 0;
    first_exception = true;
    single_stepping = false;

    serial = s;
}

void Shutdown() {
}

/*static Kernel::Thread* FindThreadById(s64 id) {
    for (u32 core = 0; core < Core::NUM_CPU_CORES; core++) {
        const auto& threads = Core::System::GetInstance().Scheduler(core).GetThreadList();
        for (auto& thread : threads) {
            if (thread->GetThreadID() == static_cast<u64>(id)) {
                current_core = core;
                return thread.get();
            }
        }
    }
    return nullptr;
}*/

static u64 RegRead(std::size_t id, struct ThreadExceptionDump_t *ctx) {
    if (ctx == nullptr) {
        return 0;
    }

    if (id < FP_REGISTER) {
        return ctx->cpu_gprs[id].x;
    } else if (id == FP_REGISTER) {
        return ctx->fp.x;
    } else if (id == LR_REGISTER) {
        return ctx->lr.x;
    } else if (id == SP_REGISTER) {
        return ctx->sp.x;
    } else if (id == PC_REGISTER) {
        return ctx->pc.x;
    } else if (id == PSTATE_REGISTER) {
        return ctx->pstate;
    } else if (id > PSTATE_REGISTER && id < FPSR_REGISTER) {
        return ctx->fpu_gprs[id - UC_ARM64_REG_Q0].d;
    } else if (id == FPSR_REGISTER) {
        return ctx->fpsr;
    } else if (id == FPCR_REGISTER) {
        return ctx->fpcr;
    } else {
        return 0;
    }
}

static void RegWrite(std::size_t id, u64 val, struct ThreadExceptionDump_t *ctx) {
    if (ctx == nullptr) {
        return;
    }

    if (id < FP_REGISTER) {
        ctx->cpu_gprs[id].x = val;
    } else if (id == FP_REGISTER) {
        ctx->fp.x = val;
    } else if (id == LR_REGISTER) {
        ctx->lr.x = val;
    } else if (id == SP_REGISTER) {
        ctx->sp.x = val;
    } else if (id == PC_REGISTER) {
        ctx->pc.x = val;
    } else if (id == PSTATE_REGISTER) {
        ctx->pstate = static_cast<u32>(val);
    } else if (id > PSTATE_REGISTER && id < FPSR_REGISTER) {
        ctx->fpu_gprs[id - (PSTATE_REGISTER + 1)].d = val;
    } else if (id == FPSR_REGISTER) {
        ctx->fpsr = static_cast<u32>(val);
    } else if (id == FPCR_REGISTER) {
       ctx->fpcr = static_cast<u32>(val);
    }
}

static u128 FpuRead(std::size_t id, struct ThreadExceptionDump_t *ctx) {
    if (ctx == nullptr) {
        return 0;
    }

    if (id >= UC_ARM64_REG_Q0 && id < FPSR_REGISTER) {
        return ctx->fpu_gprs[id - UC_ARM64_REG_Q0].v;
    } else {
        return 0;
    }
}

static void FpuWrite(std::size_t id, u128 val, struct ThreadExceptionDump_t *ctx) {
    if (ctx == nullptr) {
        return;
    }

    if (id >= UC_ARM64_REG_Q0 && id < FPSR_REGISTER) {
        ctx->fpu_gprs[id - UC_ARM64_REG_Q0].v = val;
    }
}

static void CacheFlush(char* start, char* end)
{
    // Don't rely on GCC's __clear_cache implementation, as it caches
    // icache/dcache cache line sizes, that can vary between cores on
    // big.LITTLE architectures.
    uint64_t addr, ctr_el0;
    static size_t icache_line_size = 0xffff, dcache_line_size = 0xffff;
    size_t isize, dsize;

    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr_el0));
    isize = 4 << ((ctr_el0 >> 0) & 0xf);
    dsize = 4 << ((ctr_el0 >> 16) & 0xf);

    // use the global minimum cache line size
    icache_line_size = isize = icache_line_size < isize ? icache_line_size : isize;
    dcache_line_size = dsize = dcache_line_size < dsize ? dcache_line_size : dsize;

    addr = (uint64_t)start & ~(uint64_t)(dsize - 1);
    for (; addr < (uint64_t)end; addr += dsize)
        // use "civac" instead of "cvau", as this is the suggested workaround for
        // Cortex-A53 errata 819472, 826319, 827319 and 824069.
            __asm__ volatile("dc civac, %0" : : "r"(addr) : "memory");
    __asm__ volatile("dsb ish" : : : "memory");

    addr = (uint64_t)start & ~(uint64_t)(isize - 1);
    for (; addr < (uint64_t)end; addr += isize)
            __asm__ volatile("ic ivau, %0" : : "r"(addr) : "memory");

    __asm__ volatile("dsb ish" : : : "memory");
    __asm__ volatile("isb" : : : "memory");
}

static void ReadBlock(const u64 src_addr, void* dest_buffer, const std::size_t size) {
    MemoryInfo meminfo;
    u32 pageinfo;

    Result rc = svcQueryMemory(&meminfo, &pageinfo, src_addr);
    if(R_FAILED(rc))
    {
        LOG_ERROR("gdb: failed to query memory");
        return;
    }

    // page aligned
    u64 page_addr = src_addr & ~0xFFF;
    std::size_t _size = (size + 0xFFF) & ~0xFFF;
    u64 source = src_addr;
    u64 offset = 0;

    if(!(meminfo.perm & Perm_R))
    {
        source = (u64)virtmemReserve(_size);
        offset = src_addr - page_addr;
        rc = svcMapProcessMemory((void*)source, g_procHandle, page_addr, _size);
        if(R_FAILED(rc))
            LOG_ERROR("gdb: failed to map process memory");
    }

    if(R_SUCCEEDED(rc)) memcpy(dest_buffer, (void*)(source+offset), size);

    if(!(meminfo.perm & Perm_R))
    {
        if(R_SUCCEEDED(rc))
        {
            rc = svcUnmapProcessMemory((void*)source, g_procHandle, page_addr, _size);
            if(R_FAILED(rc))
                LOG_ERROR("gdb: failed to unmap process memory");
        }
        virtmemFree((void*)source, _size);
    }
}

static void WriteBlock(const u64 dest_addr, const void* src_buffer, const std::size_t size) {
    MemoryInfo meminfo;
    u32 pageinfo;

    Result rc = svcQueryMemory(&meminfo, &pageinfo, dest_addr);
    if(R_FAILED(rc))
    {
        LOG_ERROR("gdb: failed to query memory");
        return;
    }

    // page aligned
    u64 page_addr = dest_addr & ~0xFFF;
    std::size_t _size = (size + 0xFFF) & ~0xFFF;
    u64 destination = dest_addr;
    u64 offset = 0;

    if(!(meminfo.perm & Perm_W))
    {
        destination = (u64)virtmemReserve(_size);
        offset = dest_addr - page_addr;
        rc = svcMapProcessMemory((void*)destination, g_procHandle, page_addr, _size);
        if(R_FAILED(rc))
            LOG_ERROR("gdb: failed to map process memory");
    }

    if(R_SUCCEEDED(rc)) memcpy((void*)(destination+offset), src_buffer, size);
    if(R_SUCCEEDED(rc)) CacheFlush((char*)(destination+offset), (char*)(destination+offset+size));

    if(!(meminfo.perm & Perm_W))
    {
        if(R_SUCCEEDED(rc))
        {
            rc = svcUnmapProcessMemory((void*)destination, g_procHandle, page_addr, _size);
            if(R_FAILED(rc))
                LOG_ERROR("gdb: failed to unmap process memory");
        }
        virtmemFree((void*)destination, _size);
    }
}

/**
 * Turns hex string character into the equivalent byte.
 *
 * @param hex Input hex character to be turned into byte.
 */
static u8 HexCharToValue(u8 hex) {
    if (hex >= '0' && hex <= '9') {
        return hex - '0';
    } else if (hex >= 'a' && hex <= 'f') {
        return hex - 'a' + 0xA;
    } else if (hex >= 'A' && hex <= 'F') {
        return hex - 'A' + 0xA;
    }

    LOG_ERROR("Invalid nibble: {} ({:02X})", hex, hex);
    return 0;
}

/**
 * Turn nibble of byte into hex string character.
 *
 * @param n Nibble to be turned into hex character.
 */
static u8 NibbleToHex(u8 n) {
    n &= 0xF;
    if (n < 0xA) {
        return '0' + n;
    } else {
        return 'a' + n - 0xA;
    }
}

/**
 * Converts input hex string characters into an array of equivalent of u8 bytes.
 *
 * @param src Pointer to array of output hex string characters.
 * @param len Length of src array.
 */
static u32 HexToInt(const u8* src, std::size_t len) {
    u32 output = 0;
    while (len-- > 0) {
        output = (output << 4) | HexCharToValue(src[0]);
        src++;
    }
    return output;
}

/**
 * Converts input hex string characters into an array of equivalent of u8 bytes.
 *
 * @param src Pointer to array of output hex string characters.
 * @param len Length of src array.
 */
static u64 HexToLong(const u8* src, std::size_t len) {
    u64 output = 0;
    while (len-- > 0) {
        output = (output << 4) | HexCharToValue(src[0]);
        src++;
    }
    return output;
}

/**
 * Converts input array of u8 bytes into their equivalent hex string characters.
 *
 * @param dest Pointer to buffer to store output hex string characters.
 * @param src Pointer to array of u8 bytes.
 * @param len Length of src array.
 */
static void MemToGdbHex(u8* dest, const u8* src, std::size_t len) {
    while (len-- > 0) {
        u8 tmp = *src++;
        *dest++ = NibbleToHex(tmp >> 4);
        *dest++ = NibbleToHex(tmp);
    }
}

/**
 * Converts input gdb-formatted hex string characters into an array of equivalent of u8 bytes.
 *
 * @param dest Pointer to buffer to store u8 bytes.
 * @param src Pointer to array of output hex string characters.
 * @param len Length of src array.
 */
static void GdbHexToMem(u8* dest, const u8* src, std::size_t len) {
    while (len-- > 0) {
        *dest++ = (HexCharToValue(src[0]) << 4) | HexCharToValue(src[1]);
        src += 2;
    }
}

/**
 * Convert a u32 into a gdb-formatted hex string.
 *
 * @param dest Pointer to buffer to store output hex string characters.
 * @param v    Value to convert.
 */
static void IntToGdbHex(u8* dest, u32 v) {
    for (int i = 0; i < 8; i += 2) {
        dest[i + 1] = NibbleToHex(static_cast<u8>(v >> (4 * i)));
        dest[i] = NibbleToHex(static_cast<u8>(v >> (4 * (i + 1))));
    }
}

/**
 * Convert a u64 into a gdb-formatted hex string.
 *
 * @param dest Pointer to buffer to store output hex string characters.
 * @param v    Value to convert.
 */
static void LongToGdbHex(u8* dest, u64 v) {
    for (int i = 0; i < 16; i += 2) {
        dest[i + 1] = NibbleToHex(static_cast<u8>(v >> (4 * i)));
        dest[i] = NibbleToHex(static_cast<u8>(v >> (4 * (i + 1))));
    }
}

/**
 * Convert a u128 into a gdb-formatted hex string.
 *
 * @param dest Pointer to buffer to store output hex string characters.
 * @param v    Value to convert.
 */
static void U128ToGdbHex(u8* dest, u128 v) {
    for (int i = 0; i < 32; i += 2) {
        dest[i + 1] = NibbleToHex(static_cast<u8>(v >> (4 * i)));
        dest[i] = NibbleToHex(static_cast<u8>(v >> (4 * (i + 1))));
    }
}

/**
 * Convert a gdb-formatted hex string into a u32.
 *
 * @param src Pointer to hex string.
 */
static u32 GdbHexToInt(const u8* src) {
    u32 output = 0;

    for (int i = 0; i < 8; i += 2) {
        output = (output << 4) | HexCharToValue(src[7 - i - 1]);
        output = (output << 4) | HexCharToValue(src[7 - i]);
    }

    return output;
}

/**
 * Convert a gdb-formatted hex string into a u64.
 *
 * @param src Pointer to hex string.
 */
static u64 GdbHexToLong(const u8* src) {
    u64 output = 0;

    for (int i = 0; i < 16; i += 2) {
        output = (output << 4) | HexCharToValue(src[15 - i - 1]);
        output = (output << 4) | HexCharToValue(src[15 - i]);
    }

    return output;
}

/**
 * Convert a gdb-formatted hex string into a u128.
 *
 * @param src Pointer to hex string.
 */
static u128 GdbHexToU128(const u8* src) {
    u128 output = 0;

    for (int i = 0; i < 32; i += 2) {
        output = (output << 4) | HexCharToValue(src[31 - i - 1]);
        output = (output << 4) | HexCharToValue(src[31 - i]);
    }

    return output;
}

/// Calculate the checksum of the current command buffer.
static u8 CalculateChecksum(const u8* buffer, std::size_t length) {
    return static_cast<u8>(std::accumulate(buffer, buffer + length, 0, std::plus<u8>()));
}

/**
 * Get the map of breakpoints for a given breakpoint type.
 *
 * @param type Type of breakpoint map.
 */
static BreakpointMap& GetBreakpointMap(BreakpointType type) {
    switch (type) {
    case BreakpointType::Execute:
        return breakpoints_execute;
    case BreakpointType::Read:
        return breakpoints_read;
    case BreakpointType::Write:
        return breakpoints_write;
    default:
        return breakpoints_read;
    }
}

/**
 * Remove the breakpoint from the given address of the specified type.
 *
 * @param type Type of breakpoint.
 * @param addr Address of breakpoint.
 */
static void RemoveBreakpoint(BreakpointType type, u64 addr) {
    BreakpointMap& p = GetBreakpointMap(type);

    const auto bp = p.find(addr);
    if (bp == p.end()) {
        return;
    }

    LOG_DEBUG("gdb: removed a breakpoint: {:016X} bytes at {:016X} of type {}",
              bp->second.len, bp->second.addr, static_cast<int>(type));

    if (type == BreakpointType::Execute) {
        WriteBlock(bp->second.addr, bp->second.inst.data(), bp->second.inst.size());
    }
    p.erase(addr);
}

/*BreakpointAddress_t GetNextBreakpointFromAddress(u64 addr, BreakpointType type) {
    const BreakpointMap& p = GetBreakpointMap(type);
    const auto next_breakpoint = p.lower_bound(addr);
    BreakpointAddress_t breakpoint;

    if (next_breakpoint != p.end()) {
        breakpoint.address = next_breakpoint->first;
        breakpoint.type = type;
    } else {
        breakpoint.address = 0;
        breakpoint.type = BreakpointType::None;
    }

    return breakpoint;
}*/

/*bool CheckBreakpoint(u64 addr, BreakpointType type) {
    if (!IsConnected()) {
        return false;
    }

    const BreakpointMap& p = GetBreakpointMap(type);
    const auto bp = p.find(addr);

    if (bp == p.end()) {
        return false;
    }

    u64 len = bp->second.len;

    // IDA Pro defaults to 4-byte breakpoints for all non-hardware breakpoints
    // no matter if it's a 4-byte or 2-byte instruction. When you execute a
    // Thumb instruction with a 4-byte breakpoint set, it will set a breakpoint on
    // two instructions instead of the single instruction you placed the breakpoint
    // on. So, as a way to make sure that execution breakpoints are only breaking
    // on the instruction that was specified, set the length of an execution
    // breakpoint to 1. This should be fine since the CPU should never begin executing
    // an instruction anywhere except the beginning of the instruction.
    if (type == BreakpointType::Execute) {
        len = 1;
    }

    if (bp->second.active && (addr >= bp->second.addr && addr < bp->second.addr + len)) {
        LOG_DEBUG("Found breakpoint type {} @ {:016X}, range: {:016X}"
                  " - {:016X} ({:X} bytes)",
                  static_cast<int>(type), addr, bp->second.addr, bp->second.addr + len, len);
        return true;
    }

    return false;
}*/

/**
 * Send packet to gdb client.
 *
 * @param packet Packet to be sent to client.
 */
static void SendPacket(const char packet) {
    std::size_t sent_size = serial->write(serial, &packet, 1);
    if (sent_size != 1) {
        LOG_ERROR("send failed");
    }
}

/**
 * Send reply to gdb client.
 *
 * @param reply Reply to be sent to client.
 */
static void SendReply(const char* reply) {
    LOG_DEBUG("Reply: {}", reply);

    memset(command_buffer, 0, sizeof(command_buffer));

    command_length = static_cast<u32>(strlen(reply));
    if (command_length + 4 > sizeof(command_buffer)) {
        LOG_ERROR("command_buffer overflow in SendReply");
        return;
    }

    memcpy(command_buffer + 1, reply, command_length);

    u8 checksum = CalculateChecksum(command_buffer, command_length + 1);
    command_buffer[0] = GDB_STUB_START;
    command_buffer[command_length + 1] = GDB_STUB_END;
    command_buffer[command_length + 2] = NibbleToHex(checksum >> 4);
    command_buffer[command_length + 3] = NibbleToHex(checksum);

    u8* ptr = command_buffer;
    u32 left = command_length + 4;
    while (left > 0) {
        int sent_size = serial->write(serial, reinterpret_cast<char*>(ptr), left);
        if (sent_size < 0) {
            LOG_ERROR("gdb: send failed");
            return Shutdown();
        }

        left -= sent_size;
        ptr += sent_size;
    }
}

/// Handle query command from gdb client.
static void HandleQuery(struct ThreadExceptionDump_t *ctx) {
    LOG_DEBUG("gdb: query '{}'", command_buffer + 1);

    const char* query = reinterpret_cast<const char*>(command_buffer + 1);

    if (strcmp(query, "TStatus") == 0) {
        SendReply("T0");
    } else if (strncmp(query, "Supported", strlen("Supported")) == 0) {
        // PacketSize needs to be large enough for target xml
        std::string buffer = "PacketSize=2000;qXfer:features:read+";
        /*std::string buffer = "PacketSize=2000;qXfer:features:read+;qXfer:threads:read+";*/
        SendReply(buffer.c_str());
    } else if (strncmp(query, "Xfer:features:read:target.xml:",
                       strlen("Xfer:features:read:target.xml:")) == 0) {
        SendReply(target_xml);
    } else if (strncmp(query, "Offsets", strlen("Offsets")) == 0) {
		u64 base_address = g_nroAddr;
        std::string buffer = fmt::format("TextSeg={:0x}", base_address);
        SendReply(buffer.c_str());
    } else if (strncmp(query, "fThreadInfo", strlen("fThreadInfo")) == 0) {
        std::string val = "m";
        /*for (u32 core = 0; core < Core::NUM_CPU_CORES; core++) {
            const auto& threads = Core::System::GetInstance().Scheduler(core).GetThreadList();
            for (const auto& thread : threads) {
                val += fmt::format("{:x},", thread->GetThreadID());
            }
        }*/
        val.pop_back();
        SendReply(val.c_str());
    } else if (strncmp(query, "sThreadInfo", strlen("sThreadInfo")) == 0) {
        SendReply("l");
    } else if (strncmp(query, "Xfer:threads:read", strlen("Xfer:threads:read")) == 0) {
        std::string buffer;
        buffer += "l<?xml version=\"1.0\"?>";
        buffer += "<threads>";
        /*for (u32 core = 0; core < Core::NUM_CPU_CORES; core++) {
            const auto& threads = Core::System::GetInstance().Scheduler(core).GetThreadList();
            for (const auto& thread : threads) {
                buffer +=
                    fmt::format(R"*(<thread id="{:x}" core="{:d}" name="Thread {:x}"></thread>)*",
                                thread->GetThreadID(), core, thread->GetThreadID());
            }
        }*/
        buffer += "</threads>";
        SendReply(buffer.c_str());
    } else {
        SendReply("");
    }
}

/// Handle set thread command from gdb client.
static void HandleSetThread() {
    /*int thread_id = -1;
    if (command_buffer[2] != '-') {
        thread_id = static_cast<int>(HexToInt(command_buffer + 2, command_length - 2));
    }
    if (thread_id >= 1) {
        current_thread = FindThreadById(thread_id);
    }
    if (!current_thread) {
        thread_id = 1;
        current_thread = FindThreadById(thread_id);
    }
    if (current_thread) {
        SendReply("OK");
        return;
    }
    SendReply("E01");*/
    SendReply("E01");
}

/// Handle thread alive command from gdb client.
static void HandleThreadAlive() {
    /*int thread_id = static_cast<int>(HexToInt(command_buffer + 1, command_length - 1));
    if (thread_id == 0) {
        thread_id = 1;
    }
    if (FindThreadById(thread_id)) {
        SendReply("OK");
        return;
    }
    SendReply("E01");*/
    SendReply("E01");
}

/**
 * Send signal packet to client.
 *
 * @param signal Signal to be sent to client.
 */
static void SendSignal(struct ThreadExceptionDump_t *ctx, u32 signal, bool full = true) {
    latest_signal = signal;

    if (ctx == nullptr) {
        full = false;
    }

    std::string buffer;
    if (full) {
        buffer = fmt::format("T{:02x}{:02x}:{:016x};{:02x}:{:016x};{:02x}:{:016x};", latest_signal,
                             PC_REGISTER, __builtin_bswap64(RegRead(PC_REGISTER, ctx)), SP_REGISTER,
                             __builtin_bswap64(RegRead(SP_REGISTER, ctx)), LR_REGISTER,
                             __builtin_bswap64(RegRead(LR_REGISTER, ctx)));
    } else {
        buffer = fmt::format("T{:02x};", latest_signal);
    }

    /*if (thread) {
        buffer += fmt::format("thread:{:x};", thread->GetThreadID());
    }*/

    SendReply(buffer.c_str());
}

/// Read command from gdb client.
static void ReadCommand(struct ThreadExceptionDump_t *ctx) {
    u8 checksum_received;
    u8 checksum_calculated;
    
    char *buffer = (char*)memalign(0x1000, 16384);
    u8 *c = (u8*)buffer;
    int size = serial->read(serial, buffer, 16384);
    command_length = 0;
    memset(command_buffer, 0, sizeof(command_buffer));

    if(size < 0)
        goto failed;

    if (*c == '+') {
        // ignore ack
        goto failed;
    } else if (*c == 0x03) {
        LOG_INFO("gdb: found break command");
        SendSignal(ctx, SIGTRAP);
        goto failed;
    } else if (*c != GDB_STUB_START) {
        LOG_DEBUG("gdb: read invalid byte {:02X}", c);
        goto failed;
    }

    while (*(++c) != GDB_STUB_END) {
        if (command_length >= sizeof(command_buffer)) {
            LOG_ERROR("gdb: command_buffer overflow");
            SendPacket(GDB_STUB_NACK);
            goto failed;
        }
        command_buffer[command_length++] = *c;
    }

    checksum_received = HexCharToValue(*(++c)) << 4;
    checksum_received |= HexCharToValue(*(++c));
    checksum_calculated = CalculateChecksum(command_buffer, command_length);

    if (checksum_received != checksum_calculated) {
        LOG_ERROR("gdb: invalid checksum: calculated {:02X} and read {:02X} for ${}# (length: {})",
                  checksum_calculated, checksum_received, command_buffer, command_length);

        command_length = 0;

        SendPacket(GDB_STUB_NACK);
        goto failed;
    }

    SendPacket(GDB_STUB_ACK);

failed:
    free(buffer);
}

/// Send requested register to gdb client.
static void ReadRegister(struct ThreadExceptionDump_t *ctx) {
    static u8 reply[64];
    memset(reply, 0, sizeof(reply));

    u32 id = HexCharToValue(command_buffer[1]);
    if (command_buffer[2] != '\0') {
        id <<= 4;
        id |= HexCharToValue(command_buffer[2]);
    }

    if (id <= SP_REGISTER) {
        LongToGdbHex(reply, RegRead(id, ctx));
    } else if (id == PC_REGISTER) {
        LongToGdbHex(reply, RegRead(id, ctx));
    } else if (id == PSTATE_REGISTER) {
        IntToGdbHex(reply, static_cast<u32>(RegRead(id, ctx)));
    } else if (id >= UC_ARM64_REG_Q0 && id < FPSR_REGISTER) {
        u128 r = FpuRead(id, ctx);
        U128ToGdbHex(reply, r);
    } else if (id == FPSR_REGISTER) {
        IntToGdbHex(reply, static_cast<u32>(RegRead(id, ctx)));
    } else if (id == FPCR_REGISTER) {
        IntToGdbHex(reply, static_cast<u32>(RegRead(id, ctx)));
    }

    SendReply(reinterpret_cast<char*>(reply));
}

/// Send all registers to the gdb client.
static void ReadRegisters(struct ThreadExceptionDump_t *ctx) {
    static u8 buffer[GDB_BUFFER_SIZE - 4];
    memset(buffer, 0, sizeof(buffer));

    u8* bufptr = buffer;

    for (u32 reg = 0; reg <= SP_REGISTER; reg++) {
        LongToGdbHex(bufptr + reg * 16, RegRead(reg, ctx));
    }

    bufptr += 32 * 16;

    LongToGdbHex(bufptr, RegRead(PC_REGISTER, ctx));
    bufptr += 16;

    IntToGdbHex(bufptr, static_cast<u32>(RegRead(PSTATE_REGISTER, ctx)));
    bufptr += 8;

    u128 r;

    for (u32 reg = UC_ARM64_REG_Q0; reg < FPSR_REGISTER; reg++) {
        r = FpuRead(reg, ctx);
        U128ToGdbHex(bufptr + reg * 32, r);
    }

    bufptr += 32 * 32;

    IntToGdbHex(bufptr, static_cast<u32>(RegRead(FPSR_REGISTER, ctx)));
    bufptr += 8;

    IntToGdbHex(bufptr, static_cast<u32>(RegRead(FPCR_REGISTER, ctx)));
    bufptr += 8;

    SendReply(reinterpret_cast<char*>(buffer));
}

/// Modify data of register specified by gdb client.
static void WriteRegister(struct ThreadExceptionDump_t *ctx) {
    const u8* buffer_ptr = command_buffer + 3;

    u32 id = HexCharToValue(command_buffer[1]);
    if (command_buffer[2] != '=') {
        ++buffer_ptr;
        id <<= 4;
        id |= HexCharToValue(command_buffer[2]);
    }

    if (id <= SP_REGISTER) {
        RegWrite(id, GdbHexToLong(buffer_ptr), ctx);
    } else if (id == PC_REGISTER) {
        RegWrite(id, GdbHexToLong(buffer_ptr), ctx);
    } else if (id == PSTATE_REGISTER) {
        RegWrite(id, GdbHexToInt(buffer_ptr), ctx);
    } else if (id >= UC_ARM64_REG_Q0 && id < FPSR_REGISTER) {
        FpuWrite(id, GdbHexToU128(buffer_ptr), ctx);
    } else if (id == FPSR_REGISTER) {
        //TODO
    } else if (id == FPCR_REGISTER) {
        //TODO
    }

    SendReply("OK");
}

/// Modify all registers with data received from the client.
static void WriteRegisters(struct ThreadExceptionDump_t *ctx) {
    const u8* buffer_ptr = command_buffer + 1;

    if (command_buffer[0] != 'G')
        return SendReply("E01");

    for (u32 i = 0, reg = 0; reg <= FPCR_REGISTER; i++, reg++) {
        if (reg <= SP_REGISTER) {
            RegWrite(reg, GdbHexToLong(buffer_ptr + i * 16), ctx);
        } else if (reg == PC_REGISTER) {
            RegWrite(PC_REGISTER, GdbHexToLong(buffer_ptr + i * 16), ctx);
        } else if (reg == PSTATE_REGISTER) {
            RegWrite(PSTATE_REGISTER, GdbHexToInt(buffer_ptr + i * 16), ctx);
        } else if (reg >= UC_ARM64_REG_Q0 && reg < FPSR_REGISTER) {
            RegWrite(reg, GdbHexToLong(buffer_ptr + i * 16), ctx);
        } else if (reg == FPSR_REGISTER) {
            RegWrite(FPSR_REGISTER, GdbHexToLong(buffer_ptr + i * 16), ctx);
        } else if (reg == FPCR_REGISTER) {
            RegWrite(FPCR_REGISTER, GdbHexToLong(buffer_ptr + i * 16), ctx);
        }
    }

    SendReply("OK");
}

/// Read location in memory specified by gdb client.
static void ReadMemory() {
    static u8 reply[GDB_BUFFER_SIZE - 4];

    auto start_offset = command_buffer + 1;
    const auto addr_pos = std::find(start_offset, command_buffer + command_length, ',');
    const u64 addr = HexToLong(start_offset, static_cast<u64>(addr_pos - start_offset));

    start_offset = addr_pos + 1;
    const u64 len =
        HexToLong(start_offset, static_cast<u64>((command_buffer + command_length) - start_offset));

    LOG_DEBUG("gdb: addr: {:016X} len: {:016X}", addr, len);

    if (len * 2 > sizeof(reply)) {
        SendReply("E01");
    }

    //TODO: check address space
    /*if (!Memory::IsValidVirtualAddress(addr)) {
        return SendReply("E00");
    }*/

    std::vector<u8> data(len);
    ReadBlock(addr, data.data(), len);

    MemToGdbHex(reply, data.data(), len);
    reply[len * 2] = '\0';
    SendReply(reinterpret_cast<char*>(reply));
}

/// Modify location in memory with data received from the gdb client.
static void WriteMemory() {
    auto start_offset = command_buffer + 1;
    auto addr_pos = std::find(start_offset, command_buffer + command_length, ',');
    u64 addr = HexToLong(start_offset, static_cast<u64>(addr_pos - start_offset));

    start_offset = addr_pos + 1;
    auto len_pos = std::find(start_offset, command_buffer + command_length, ':');
    u64 len = HexToLong(start_offset, static_cast<u64>(len_pos - start_offset));

    //TODO: check address space
    /*if (!Memory::IsValidVirtualAddress(addr)) {
        return SendReply("E00");
    }*/

    std::vector<u8> data(len);

    GdbHexToMem(data.data(), len_pos + 1, len);
    WriteBlock(addr, data.data(), len);
    SendReply("OK");
}

/// Tell the CPU to continue executing.
static void Continue(struct ThreadExceptionDump_t *ctx) {
    (void)ctx;
}

/**
 * Commit breakpoint to list of breakpoints.
 *
 * @param type Type of breakpoint.
 * @param addr Address of breakpoint.
 * @param len Length of breakpoint.
 */
static bool CommitBreakpoint(BreakpointType type, u64 addr, u64 len) {
    BreakpointMap& p = GetBreakpointMap(type);

    Breakpoint_t breakpoint;
    breakpoint.active = true;
    breakpoint.addr = addr;
    breakpoint.len = len;
    ReadBlock(addr, breakpoint.inst.data(), breakpoint.inst.size());

    static constexpr std::array<u8, 4> btrap{0x00, 0x00, 0x20, 0xd4};
    if (type == BreakpointType::Execute) {
        WriteBlock(addr, btrap.data(), btrap.size());
    }
    p.insert({addr, breakpoint});

    LOG_DEBUG("gdb: added {} breakpoint: {:016X} bytes at {:016X}",
              static_cast<int>(type), breakpoint.len, breakpoint.addr);

    return true;
}

/// Tell the CPU that it should perform a single step.
static void Step(struct ThreadExceptionDump_t *ctx) {
    if (command_length > 1) {
        RegWrite(PC_REGISTER, GdbHexToLong(command_buffer + 1), ctx);
    }

    u64 pc = RegRead(PC_REGISTER, ctx);
    u32 opcode = 0;
    ReadBlock(pc, &opcode, 4);

    if (((opcode >> 26) == 5) || ((opcode >> 26) == 37)) { // B/BL
        pc += (((int)opcode << 6) >> 6) << 2;
    } else if ((opcode >> 24) == 84) { // B.cond
        union NZCV pstate;
        u32 cond = opcode & 0xf;
        pstate.reg = (u32)RegRead(PSTATE_REGISTER, ctx);
        int imm = ((int)(opcode << 8) >> 13) << 2;
        bool result;

        switch (cond >> 1) {
        case 0: // EQ or NE
            result = (pstate.Z == 1);
            break;
        case 1: // CS or CC
            result = (pstate.C == 1);
            break;
        case 2: // MI or PL
            result = (pstate.N == 1);
            break;
        case 3: // VS or VC
            result = (pstate.V == 1);
            break;
        case 4: // HI or LS
            result = ((pstate.C == 1) && (pstate.Z == 0));
            break;
        case 5: // GE or LT
            result = (pstate.N == pstate.V);
            break;
        case 6: // GT or LE
            result = ((pstate.N == pstate.V) && (pstate.Z == 0));
            break;
        case 7: // AL
            result = true;
            break;
        default:
            break;
        }
        if ((cond & 1) && (cond != 0xf))
            result = !result;

        pc = result ? pc + imm : pc + 4; // taken / not taken
    } else if (((opcode >> 10) == 3510208) || ((opcode >> 10) == 3508160) ||
               ((opcode >> 10) == 3512256)) { // BLR/BR/RET
        pc = RegRead(((opcode >> 5) & 31), ctx);
    } else if (((opcode >> 25) & 0x3f) == 26) { // CBNZ / CBZ
        u64 cond = RegRead((opcode & 31), ctx);
        cond = ((opcode >> 31) & 1) ? cond : (u32)cond; // 64-bit / 32-bit
        int imm = ((int)(opcode << 8) >> 13) << 2;
        bool op = (((opcode >> 24) & 1) == 0);
        bool isZero = (cond == 0);
        pc = (isZero == op) ? pc + imm : pc + 4; // taken / not taken
    } else if (((opcode >> 25) & 0x3f) == 27) {  // TBNZ / TBZ
        u64 cond = RegRead((opcode & 31), ctx);
        u32 bit = (((opcode >> 31) & 1) << 5) | ((opcode >> 19) & 31);
        int imm = ((int)(opcode << 13) >> 18) << 2;
        bool op = (((opcode >> 24) & 1) == 0);
        bool isZero = ((cond & (1LL << bit)) == 0);
        pc = (isZero == op) ? pc + imm : pc + 4; // taken / not taken
    } else {                                     // Not a branch
        pc += 4;
    }

    CommitBreakpoint(BreakpointType::Execute, pc, 4);
    single_stepping = true;
}

/// Handle add breakpoint command from gdb client.
static void AddBreakpoint() {
    BreakpointType type;

    u8 type_id = HexCharToValue(command_buffer[1]);
    switch (type_id) {
    case 0:
    case 1:
        type = BreakpointType::Execute;
        break;
    case 2:
        type = BreakpointType::Write;
        break;
    case 3:
        type = BreakpointType::Read;
        break;
    case 4:
        type = BreakpointType::Access;
        break;
    default:
        return SendReply("E01");
    }

    auto start_offset = command_buffer + 3;
    auto addr_pos = std::find(start_offset, command_buffer + command_length, ',');
    u64 addr = HexToLong(start_offset, static_cast<u64>(addr_pos - start_offset));

    start_offset = addr_pos + 1;
    u64 len =
        HexToLong(start_offset, static_cast<u64>((command_buffer + command_length) - start_offset));

    if (type == BreakpointType::Access) {
        // Access is made up of Read and Write types, so add both breakpoints
        type = BreakpointType::Read;

        if (!CommitBreakpoint(type, addr, len)) {
            return SendReply("E02");
        }

        type = BreakpointType::Write;
    }

    if (!CommitBreakpoint(type, addr, len)) {
        return SendReply("E02");
    }

    SendReply("OK");
}

/// Handle remove breakpoint command from gdb client.
static void RemoveBreakpoint() {
    BreakpointType type;

    u8 type_id = HexCharToValue(command_buffer[1]);
    switch (type_id) {
    case 0:
    case 1:
        type = BreakpointType::Execute;
        break;
    case 2:
        type = BreakpointType::Write;
        break;
    case 3:
        type = BreakpointType::Read;
        break;
    case 4:
        type = BreakpointType::Access;
        break;
    default:
        return SendReply("E01");
    }

    auto start_offset = command_buffer + 3;
    auto addr_pos = std::find(start_offset, command_buffer + command_length, ',');
    u64 addr = HexToLong(start_offset, static_cast<u64>(addr_pos - start_offset));

    if (type == BreakpointType::Access) {
        // Access is made up of Read and Write types, so add both breakpoints
        type = BreakpointType::Read;
        RemoveBreakpoint(type, addr);

        type = BreakpointType::Write;
    }

    RemoveBreakpoint(type, addr);
    SendReply("OK");
}

void HandlePacket(struct ThreadExceptionDump_t *ctx) {
    if(!first_exception)
        SendSignal(ctx, SIGTRAP);

    if(single_stepping || first_exception)
        RemoveBreakpoint(BreakpointType::Execute, RegRead(PC_REGISTER, ctx));

    first_exception = false;
    single_stepping = false;

    while(1)
    {
        ReadCommand(ctx);
        if (command_length == 0) {
            continue;
        }

        LOG_DEBUG("Packet: {}", command_buffer);

        switch (command_buffer[0]) {
        case 'q':
            HandleQuery(ctx);
            break;
        case 'H':
            HandleSetThread();
            break;
        case '?':
            SendSignal(ctx, latest_signal);
            break;
        case 'k':
            Shutdown(); 
            LOG_INFO("killed by gdb");
            return;
        case 'g':
            ReadRegisters(ctx);
            break;
        case 'G':
            WriteRegisters(ctx);
            break;
        case 'p':
            ReadRegister(ctx);
            break;
        case 'P':
            WriteRegister(ctx);
            break;
        case 'm':
            ReadMemory();
            break;
        case 'M':
            WriteMemory();
            break;
        case 's':
            Step(ctx);
        case 'C':
        case 'c':
            Continue(ctx);
            return;
        case 'z':
            RemoveBreakpoint();
            break;
        case 'Z':
            AddBreakpoint();
            break;
        case 'T':
            HandleThreadAlive();
            break;
        default:
            SendReply("");
            break;
        }
    }
}

void AddExecuteBreakpoint(u64 addr) {
    (void)GDBStub::CommitBreakpoint(BreakpointType::Execute, addr, 4);
}
}; // namespace GDBStub

extern "C"{
struct ThreadExceptionDump_t GDBStub_exceptiondump;
alignas(16) u8 GDBStub_exception_stack[0x10000];
u64 GDBStub_exception_stack_size = sizeof(GDBStub_exception_stack);

static int quit;
static pthread_t setup_thread, gdb_thread;
static struct USBSerial_t gdb_serial, debug_serial;
static struct setup_thread_args args;
static Mutex mutex;
static CondVar condvar;

void debug_message(int level, const char *message)
{
    debug_serial.write(&debug_serial, message, strlen(message));
}

static void* handle_gdb_packet(void* args)
{
    int* quit = (int*)args;
    while(!*quit)
    {
        // Wait for exception
        mutexLock(&mutex);
        Result rc = condvarWaitTimeout(&condvar, &mutex, 1000000000LL);
        mutexUnlock(&mutex);

        if(R_SUCCEEDED(rc))
        {
            GDBStub::HandlePacket(&GDBStub_exceptiondump);
            mutexLock(&mutex);
            condvarWake(&condvar, -1);
            mutexUnlock(&mutex);
        }
    }
    return NULL;
}

int GDBStub_init()
{
    Result rc;
    struct usb_device_descriptor device_descriptor = {
        .bLength = USB_DT_DEVICE_SIZE,
        .bDescriptorType = USB_DT_DEVICE,
        .bcdUSB = 0x0110,
        .bDeviceClass = 0xef,
        .bDeviceSubClass = 0x02,
        .bDeviceProtocol = 0x01,
        .bMaxPacketSize0 = 0x40,
        .idVendor = 0x057e,
        .idProduct = 0x5000,
        .bcdDevice = 0x0100,
        .bNumConfigurations = 0x01
    };

    // Init global vars
    quit = 0;
    memset(&USBSerial_interfaces, 0, sizeof(USBSerial_interfaces));
    USBSerial_interface_count = 0;
    args.quit = &quit;
    args.interface = USBSerial_interfaces;
    args.interface_count = &USBSerial_interface_count;
    USBSerial(&gdb_serial);
    USBSerial(&debug_serial);
    mutexInit(&mutex);
    condvarInit(&condvar);

    // Setup thread
    if (pthread_create(&setup_thread, NULL, handle_setup_packet, &args))
        return 0;
    
    // Init usb
    rc = usbInitialize(&device_descriptor, "Nintendo", "Nintendo Switch", "SerialNumber");
    if (R_FAILED(rc))
        return 0;

    rc = gdb_serial.initialize(&gdb_serial, "GDB");
    if (R_FAILED(rc))
        return 0;

    rc = debug_serial.initialize(&debug_serial, "DEBUG");
    if (R_FAILED(rc))
        return 0;

    rc = usbEnable();
    if (R_FAILED(rc))
        return 0;

    // GDB Init
    if (pthread_create(&gdb_thread, NULL, handle_gdb_packet, &quit))
        return 0;

    GDBStub::Init(&gdb_serial);
    return 1;
}

void GDBStub_shutdown() {
    quit = 1;
    pthread_join(setup_thread, NULL);
    pthread_join(gdb_thread, NULL);
    usbExit();
    GDBStub::Shutdown();
}

void GDBStub_add_execute_breakpoint(u64 addr) {
    GDBStub::AddExecuteBreakpoint(addr);
}

void GDBStub_exception_handler()
{
    mutexLock(&mutex);
    condvarWake(&condvar, -1);
    mutexUnlock(&mutex);

    mutexLock(&mutex);
    condvarWait(&condvar, &mutex);
    mutexUnlock(&mutex);
}
}
