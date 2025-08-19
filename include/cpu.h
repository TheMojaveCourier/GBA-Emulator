#ifndef CPU_H
#define CPU_H

#include <cstdint>
#include "memory.h"

class DebugTracer;
// Register indices (for public API and instruction decoding)
constexpr int R0 = 0, R1 = 1, R2 = 2, R3 = 3, R4 = 4, R5 = 5, R6 = 6, R7 = 7;
constexpr int R8 = 8, R9 = 9, R10 = 10, R11 = 11, R12 = 12;
constexpr int SP = 13, LR = 14, PC = 15;

// CPSR Flags
constexpr uint32_t N_FLAG = (1u << 31);  // Should be 0x80000000
constexpr uint32_t Z_FLAG = (1u << 30);  // Should be 0x40000000  
constexpr uint32_t C_FLAG = (1u << 29);  // Should be 0x20000000
constexpr uint32_t V_FLAG = (1u << 28);  // Should be 0x10000000
constexpr uint32_t I_FLAG = (1u << 7); // IRQ disable
constexpr uint32_t F_FLAG = (1u << 6); // FIQ disable
constexpr uint32_t T_BIT  = (1u << 5); // Thumb state

// CPU Modes
enum CpuMode {
    USR_MODE = 0b10000,
    FIQ_MODE = 0b10001,
    IRQ_MODE = 0b10010,
    SVC_MODE = 0b10011,
    ABT_MODE = 0b10111,
    UND_MODE = 0b11011,
    SYS_MODE = 0b11111,
    MODE_MASK = 0b11111
};


class CPU {
public:
    CPU(Memory& mem);
    void reset();
    int step();
    
    // Public accessors for debugging or external tools
    uint32_t getRegister(int reg) const;
    void setRegister(int reg, uint32_t value);
    uint32_t getCPSR() const { return cpsr; }
    uint32_t thumb_bl_offset = 0;
    bool thumb_bl_pending = false;
    void setDebugger(DebugTracer* tracer);
    void enableLegacyDebug(bool enable) { legacyDebugEnabled = enable; }
    bool isLegacyDebugEnabled() const { return legacyDebugEnabled; }

    // Inlined for efficiency
    bool isThumbMode() const { return (cpsr & T_BIT) != 0; }

private:
    // --- Debugger methods ---
    bool debuggerTraceInstruction(uint32_t addr, uint32_t opcode, bool isThumb);
    bool debuggerCheckBreakpoint(uint32_t addr);
    void debuggerDumpRegisters();
    void debuggerTraceCall(uint32_t from, uint32_t to);
    void debuggerTraceReturn(uint32_t from, uint32_t to);
    void debuggerTraceMemoryAccess(uint32_t addr, uint32_t value, bool isWrite, int size);
    void debuggerDumpMemoryRange(uint32_t start, uint32_t end);
    
    // --- Banked Core State ---
    uint32_t r[13];      // R0-R12, common to all modes
    uint32_t pc;         // R15, program counter

    // Banked Registers R13 (SP) & R14 (LR)
    uint32_t sp_usr, lr_usr;
    uint32_t sp_svc, lr_svc;
    uint32_t sp_irq, lr_irq;

    // Program Status Registers
    uint32_t cpsr;        // Current PSR
    uint32_t spsr_svc;    // Saved PSR for Supervisor
    uint32_t spsr_irq;    // Saved PSR for IRQ

    bool ime;             // Interrupt Master Enable flag
    bool halted;
    Memory& memory;
    
    // --- Private Core Helpers ---
    uint32_t getSpsr() const;
    void setSpsr(uint32_t value);
    void switchMode(CpuMode newMode);
    void branch(uint32_t targetAddress, bool link);
    bool checkCondition(uint32_t condition) const;
    
    // --- Memory Access ---
    uint8_t  readMemory8(uint32_t address);
    uint16_t readMemory16(uint32_t address);
    uint32_t readMemory32(uint32_t address);
    void writeMemory8(uint32_t address, uint8_t value);
    void writeMemory16(uint32_t address, uint16_t value);
    void writeMemory32(uint32_t address, uint32_t value);
    bool isValidMemoryAddress(uint32_t address) const;

    // --- Fetching ---
    uint32_t fetchArmOpcode();
    uint16_t fetchThumbOpcode();

    // --- Decoding & Execution (All return void now) ---
    void decodeArm(uint32_t opcode);
    void decodeThumb(uint16_t opcode);

    // --- Instruction Handlers (All return void now) ---
    void handleDataProcessing(uint32_t opcode);
    void handleLoadStore(uint32_t opcode);
    void handleBranch(uint32_t opcode);
    void handleBranchExchange(uint32_t opcode);
    void handleMsr(uint32_t opcode);
    void dumpROMArea(uint32_t address, int num_bytes);
    void handleSWI(uint8_t swiNumber);
    void handleBlockDataTransfer(uint32_t opcode);
    void handleMultiply(uint32_t opcode);
	void handleMultiplyLong(uint32_t opcode);
	void handleThumbAddSubtract(uint16_t opcode);
	void handleThumbBranchExchange(uint16_t opcode);
    
    // --- Interrupt Handling ---
    void checkAndHandleInterrupts();
    void handleUndefinedInstruction();
       

    // --- Flag Updates ---
    void updateNZFlags(uint32_t result);
    void updateCVFlagsForAdd(uint32_t op1, uint32_t op2, uint32_t result);
    void updateCVFlagsForSub(uint32_t op1, uint32_t op2, uint32_t result);
    
    // --- Debug staff  ---
    DebugTracer* debugger = nullptr;
    bool legacyDebugEnabled = false;
	friend class DebugTracer;
};

#endif // CPU_H
