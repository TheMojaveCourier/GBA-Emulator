#include "cpu.h"
#include "debug.h"

// This file implements the CPU debugging functionality

void CPU::setDebugger(DebugTracer* tracer) {
    debugger = tracer;
}

// Helper function to check debugger and call methods safely
bool CPU::debuggerTraceInstruction(uint32_t addr, uint32_t opcode, bool isThumb) {
    if (debugger) {
        debugger->traceInstruction(addr, opcode, isThumb);
        return true;
    }
    return false;
}

bool CPU::debuggerCheckBreakpoint(uint32_t addr) {
    if (debugger) {
        return debugger->checkBreakpoint(addr);
    }
    return false;
}

void CPU::debuggerDumpRegisters() {
    if (debugger) {
        debugger->dumpRegisters();
    }
}

void CPU::debuggerTraceCall(uint32_t from, uint32_t to) {
    if (debugger) {
        debugger->traceCall(from, to);
    }
}

void CPU::debuggerTraceReturn(uint32_t from, uint32_t to) {
    if (debugger) {
        debugger->traceReturn(from, to);
    }
}

void CPU::debuggerTraceMemoryAccess(uint32_t addr, uint32_t value, bool isWrite, int size) {
    if (debugger) {
        debugger->traceMemoryAccess(addr, value, isWrite, size);
    }
}

void CPU::debuggerDumpMemoryRange(uint32_t start, uint32_t end) {
    if (debugger) {
        debugger->dumpMemoryRange(start, end);
    }
}
