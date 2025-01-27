#include "cpu.h"
#include <iostream>

CPU::CPU(Memory& mem) : memory(mem), setFlags(true) {
    reset();
}

void CPU::reset() {
    for (int i = 0; i < 16; ++i) {
        registers[i] = 0;
    }
    cpsr = 0x60000010; // Default CPSR state
    pc = 0x08000000;   // Start at cartridge ROM
}

uint32_t CPU::fetchOpcode(uint32_t address) {
    return memory.read(address) | 
           (memory.read(address + 1) << 8) |
           (memory.read(address + 2) << 16) |
           (memory.read(address + 3) << 24);
}

void CPU::step() {
    uint32_t opcode = fetchOpcode(pc); // Fetch
    std::cout << "PC: 0x" << std::hex << pc << " Opcode: 0x" << opcode << std::endl;
    decode(opcode); // Decode and execute
    pc += 4; // Increment PC by 4
}

void CPU::decode(uint32_t opcode) {
    uint32_t condition = (opcode >> 28) & 0xF; // Condition field (bits 31-28)
    if (!checkCondition(condition)) { // Only execute if condition passes
        std::cout << "Condition failed for instruction: 0x" << std::hex << opcode << std::endl;
        return;  // Skip execution of the instruction if condition doesn't match
    }

    uint32_t category = (opcode >> 25) & 0x7;  // Major opcode category (bits 27-25)

    if (category == 0b000) {
        if ((opcode & 0x0C000000) == 0x00000000) {
            // Data Processing
            handleDataProcessing(opcode);
        } else {
            std::cout << "Unknown instruction: 0x" << std::hex << opcode << std::endl;
        }
    } else if (category == 0b001) {
        // Load/Store
        handleLoadStore(opcode);
    } else if (category == 0b101) {
        // Branch
        handleBranch(opcode);
    } else {
        std::cout << "Unhandled instruction category: 0x" << std::hex << opcode << std::endl;
    }
}

void CPU::handleDataProcessing(uint32_t opcode) {
    uint32_t opcodeType = (opcode >> 21) & 0xF; // Bits 24-21
    uint32_t rn = (opcode >> 16) & 0xF;         // Bits 19-16 (source register)
    uint32_t rd = (opcode >> 12) & 0xF;         // Bits 15-12 (destination register)
    uint32_t operand2 = opcode & 0xFFF;         // Operand 2 (immediate or register)
    uint32_t operand1 = registers[rn];          // Rn is the first operand
    
    uint32_t result = 0;
    bool isAdd = false;

    switch (opcodeType) {
        case 0x4: { // ADD
            isAdd = true;
            result = operand1 + operand2;
            registers[rd] = result;
            if (setFlags) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "ADD instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }    
        case 0x2: { // SUB
            isAdd = false;
            result = operand1 - operand2;
            registers[rd] = result;
            if (setFlags) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "SUB instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x1: { // MOV
            registers[rd] = operand2;
            if (setFlags) updateFlags(registers[rd], isAdd, operand1, operand2);
            std::cout << "MOV instruction, Rd: R" << rd << ", Immediate: 0x"
                      << std::hex << operand2 << std::endl;
            break;
        }
        case 0x3: { // CMP
            result = operand1 - operand2;
            if (setFlags) updateFlags(result, false, operand1, operand2); // CMP only updates flags
            std::cout << "CMP instruction, Rn: R" << rn << ", Immediate: 0x"
                      << std::hex << operand2 << std::endl;
            break;
        }
        default:
            std::cout << "Unhandled Data Processing opcode type: 0x" << std::hex << opcodeType << std::endl;
            break;
    }
}

void CPU::handleLoadStore(uint32_t opcode) {
    bool isLoad = opcode & (1 << 20);       // Bit 20: Load or Store
    bool isPreIndexed = opcode & (1 << 25); // Bit 25: Pre/Post-indexing
    bool addOffset = opcode & (1 << 24);    // Bit 24: Add or Subtract offset
    bool isImmediate = !(opcode & (1 << 23)); // Bit 23: Immediate or Register offset

    uint32_t rn = (opcode >> 16) & 0xF; // Base register
    uint32_t rd = (opcode >> 12) & 0xF; // Destination/source register
    uint32_t offset = opcode & 0xFFF;   // Immediate offset (12 bits)

    if (isLoad) {
        uint32_t address = registers[rn] + (addOffset ? offset : -offset);
        registers[rd] = readMemory(address);
        std::cout << "LDR instruction, Rd: R" << rd << ", Base: R" << rn
                  << ", Address: 0x" << std::hex << address << ", Data: 0x" << registers[rd] << std::endl;
    } else {
        uint32_t address = registers[rn] + (addOffset ? offset : -offset);
        writeMemory(address, registers[rd]);
        std::cout << "STR instruction, Rd: R" << rd << ", Base: R" << rn
                  << ", Address: 0x" << std::hex << address << ", Data: 0x" << registers[rd] << std::endl;
    }
}

void CPU::handleBranch(uint32_t opcode) {
    bool link = opcode & (1 << 24); // Bit 24: Branch with Link
    int32_t offset = opcode & 0xFFFFFF; // Bits [23:0]: Signed 24-bit offset

    // Sign-extend the offset to 32 bits
    if (offset & 0x800000) { // If the sign bit (bit 23) is set
        offset |= 0xFF000000; // Extend the sign to 32 bits
    }

    // The offset is shifted left by 2 and added to PC
    offset <<= 2;

    std::cout << (link ? "BL" : "B") << " instruction, Offset: 0x"
              << std::hex << offset << ", Target Address: 0x" << std::hex << (pc + offset)
              << std::endl;

    // If we were executing, we would update PC and LR (for BL)
    if (link) {
        registers[14] = pc + 4; // Save return address in LR
    }
    pc += offset; // Update PC (branch to target)
}

void CPU::updateFlags(uint32_t result, bool isAdd, uint32_t operand1, uint32_t operand2) {
    // Set the Z and N flags based on the result
    if (result == 0) {
        cpsr |= (1 << 30); // Set Z flag
    } else {
        cpsr &= ~(1 << 30); // Clear Z flag
    }

    if (result & 0x80000000) {
        cpsr |= (1 << 31); // Set N flag
    } else {
        cpsr &= ~(1 << 31); // Clear N flag
    }

    // Handle Carry and Overflow flags based on the operation type
    if (isAdd) {
        // Add operation: Carry is set if there's a carry-out
        if (((operand1 & 0x80000000) && (operand2 & 0x80000000)) || ((operand1 & 0x80000000) == 0 && (operand2 & 0x80000000) == 0 && (result & 0x80000000))) {
            cpsr |= (1 << 29); // Set C flag
        } else {
            cpsr &= ~(1 << 29); // Clear C flag
        }

        // Overflow in addition: set if both operands are of the same sign and the result has a different sign
        if (((operand1 & 0x80000000) == (operand2 & 0x80000000)) && ((operand1 & 0x80000000) != (result & 0x80000000))) {
            cpsr |= (1 << 28); // Set V flag
        } else {
            cpsr &= ~(1 << 28); // Clear V flag
        }
    } else {
        // Subtract operation: Carry is set if there's no borrow
        if (((operand1 & 0x80000000) && !(operand2 & 0x80000000)) || ((operand1 & 0x80000000) == 0 && (operand2 & 0x80000000) == 0 && (result & 0x80000000) == 0)) {
            cpsr |= (1 << 29); // Set C flag
        } else {
            cpsr &= ~(1 << 29); // Clear C flag
        }

        // Overflow in subtraction: set if operands have different signs and the result has a different sign
        if (((operand1 & 0x80000000) != (operand2 & 0x80000000)) && ((operand1 & 0x80000000) != (result & 0x80000000))) {
            cpsr |= (1 << 28); // Set V flag
        } else {
            cpsr &= ~(1 << 28); // Clear V flag
        }
    }
}

bool CPU::checkCondition(uint32_t condition) {
    // Example for EQ: Check if Zero flag is set
    switch (condition) {
        case 0x0: // EQ (Equal)
            return (cpsr & (1 << 30)) != 0;  // Zero flag
        default:
            return true; // Default behavior (no condition, execute instruction)
    }
}

uint32_t CPU::readMemory(uint32_t address) {
    if (address >= memory.getSize()) {
        std::cerr << "Memory read error: Invalid address 0x" << std::hex << address << std::endl;
        return 0; // Handle error
    }
    return memory.read(address);
}

void CPU::writeMemory(uint32_t address, uint32_t value) {
    if (address >= memory.getSize()) {
        std::cerr << "Memory write error: Invalid address 0x" << std::hex << address << std::endl;
        return; // Handle error
    }
    memory.write(address, value);
}
