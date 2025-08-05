#ifndef CPU_H
#define CPU_H

#pragma once
#include <cstdint>
#include "memory.h"

class CPU {
public:
    CPU(Memory& mem);
    void reset();
    void step();
    void decode(uint32_t opcode);
    void handleDataProcessing(uint32_t opcode);
    void handleLoadStore(uint32_t opcode);
    void handleBranch(uint32_t opcode);
    void updateFlags(uint32_t result, bool isAdd, uint32_t operand1, uint32_t operand2);
    
private:
    uint32_t registers[16];
    uint32_t cpsr;  // Current Program Status Register (includes flags)
    uint32_t pc;    // Program Counter
    Memory& memory; // Reference to memory object
    uint32_t fetchOpcode(uint32_t address);
    bool setFlags;
    bool checkCondition(uint32_t condition);
    uint32_t readMemory(uint32_t address, int size = 4); // size: 1=byte, 2=halfword, 4=word
    void writeMemory(uint32_t address, uint32_t value, int size = 4);
};

#endif // CPU_H
