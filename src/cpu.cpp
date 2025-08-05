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
    return memory.read32(address);
}

void CPU::step() {
    uint32_t opcode = fetchOpcode(pc); // Fetch
    std::cout << "PC: 0x" << std::hex << pc << " Opcode: 0x" << opcode << std::endl;
    
    uint32_t oldPC = pc;
    pc += 4; // Increment PC by 4 first (normal case)
    
    decode(opcode); // Decode and execute
    
    // If this was a branch instruction, the PC will have been modified by the handler
    // Check if PC was changed by branch instruction
    if (pc != oldPC + 4) {
        // PC was modified by branch instruction, don't add extra increment
        return;
    }
}

void CPU::decode(uint32_t opcode) {
    uint32_t condition = (opcode >> 28) & 0xF; // Condition field (bits 31-28)
    if (!checkCondition(condition)) { // Only execute if condition passes
        std::cout << "Condition failed for instruction: 0x" << std::hex << opcode << std::endl;
        return;  // Skip execution of the instruction if condition doesn't match
    }

    // Check for specific instruction patterns first
    if ((opcode & 0x0FC000F0) == 0x00000090) {
        // Multiply instructions (MUL, MLA)
        handleMultiply(opcode);
        return;
    }
    
    if ((opcode & 0x0E000000) == 0x08000000) {
        // Load/Store Multiple
        handleLoadStoreMultiple(opcode);
        return;
    }
    
    if ((opcode & 0x0E400F90) == 0x00000090 || (opcode & 0x0E400F90) == 0x00400090) {
        // Load/Store Halfword/Signed byte
        handleLoadStoreHalfword(opcode);
        return;
    }

    // Check major instruction categories
    if ((opcode & 0x0C000000) == 0x00000000) {
        // Data Processing instructions (including immediate and register forms)
        handleDataProcessing(opcode);
    } else if ((opcode & 0x0C000000) == 0x04000000) {
        // Load/Store instructions
        handleLoadStore(opcode);
    } else if ((opcode & 0x0E000000) == 0x0A000000) {
        // Branch instructions
        handleBranch(opcode);
    } else {
        std::cout << "Unhandled instruction: 0x" << std::hex << opcode << std::endl;
    }
}

void CPU::handleDataProcessing(uint32_t opcode) {
    uint32_t opcodeType = (opcode >> 21) & 0xF; // Bits 24-21
    uint32_t rn = (opcode >> 16) & 0xF;         // Bits 19-16 (source register)
    uint32_t rd = (opcode >> 12) & 0xF;         // Bits 15-12 (destination register)
    bool sFlag = (opcode & (1 << 20)) != 0;    // Bit 20: Set condition codes
    
    uint32_t operand1 = registers[rn];          // Rn is the first operand
    bool carryOut = false;
    uint32_t operand2 = getOperand2(opcode, carryOut);
    
    uint32_t result = 0;
    bool isAdd = false;

    switch (opcodeType) {
        case 0x0: { // AND
            result = operand1 & operand2;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "AND instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x1: { // EOR (XOR)
            result = operand1 ^ operand2;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "EOR instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x2: { // SUB
            isAdd = false;
            result = operand1 - operand2;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "SUB instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x3: { // RSB (Reverse SUB)
            isAdd = false;
            result = operand2 - operand1;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand2, operand1);
            std::cout << "RSB instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x4: { // ADD
            isAdd = true;
            result = operand1 + operand2;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "ADD instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }    
        case 0x5: { // ADC (Add with Carry)
            isAdd = true;
            uint32_t carry = (cpsr & (1 << 29)) ? 1 : 0;
            result = operand1 + operand2 + carry;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "ADC instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x6: { // SBC (Subtract with Carry)
            isAdd = false;
            uint32_t carry = (cpsr & (1 << 29)) ? 0 : 1;
            result = operand1 - operand2 - carry;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "SBC instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x7: { // RSC (Reverse Subtract with Carry)
            isAdd = false;
            uint32_t carry = (cpsr & (1 << 29)) ? 0 : 1;
            result = operand2 - operand1 - carry;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand2, operand1);
            std::cout << "RSC instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0x8: { // TST (Test)
            result = operand1 & operand2;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "TST instruction, Rn: R" << rn << std::endl;
            break;
        }
        case 0x9: { // TEQ (Test Equivalence)
            result = operand1 ^ operand2;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "TEQ instruction, Rn: R" << rn << std::endl;
            break;
        }
        case 0xA: { // CMP (Compare)
            result = operand1 - operand2;
            if (sFlag) updateFlags(result, false, operand1, operand2);
            std::cout << "CMP instruction, Rn: R" << rn << std::endl;
            break;
        }
        case 0xB: { // CMN (Compare Negative)
            result = operand1 + operand2;
            if (sFlag) updateFlags(result, true, operand1, operand2);
            std::cout << "CMN instruction, Rn: R" << rn << std::endl;
            break;
        }
        case 0xC: { // ORR (OR)
            result = operand1 | operand2;
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "ORR instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0xD: { // MOV
            registers[rd] = operand2;
            if (sFlag) updateFlags(registers[rd], isAdd, operand1, operand2);
            std::cout << "MOV instruction, Rd: R" << rd << ", Operand2: 0x"
                      << std::hex << operand2 << std::endl;
            break;
        }
        case 0xE: { // BIC (Bit Clear)
            result = operand1 & (~operand2);
            registers[rd] = result;
            if (sFlag) updateFlags(result, isAdd, operand1, operand2);
            std::cout << "BIC instruction, Rn: R" << rn << ", Rd: R" << rd << std::endl;
            break;
        }
        case 0xF: { // MVN (Move NOT)
            registers[rd] = ~operand2;
            if (sFlag) updateFlags(registers[rd], isAdd, operand1, operand2);
            std::cout << "MVN instruction, Rd: R" << rd << std::endl;
            break;
        }
        default:
            std::cout << "Unhandled Data Processing opcode type: 0x" << std::hex << opcodeType << std::endl;
            break;
    }
}

void CPU::handleLoadStore(uint32_t opcode) {
    bool load = (opcode & (1 << 20)) != 0;       // Bit 20: Load or Store
    bool writeBack = (opcode & (1 << 21)) != 0;  // Bit 21: Write-back
    bool byteTransfer = (opcode & (1 << 22)) != 0; // Bit 22: Byte or Word
    bool up = (opcode & (1 << 23)) != 0;         // Bit 23: Up or Down
    bool preIndex = (opcode & (1 << 24)) != 0;   // Bit 24: Pre or Post indexing
    bool immediate = !(opcode & (1 << 25)) != 0; // Bit 25: Immediate (0) or Register (1) offset

    uint32_t rn = (opcode >> 16) & 0xF; // Base register
    uint32_t rd = (opcode >> 12) & 0xF; // Destination/source register
    
    uint32_t offset;
    bool carryOut = false;
    
    if (immediate) {
        offset = opcode & 0xFFF;   // Immediate offset (12 bits)
    } else {
        // Register offset with optional shift
        offset = getOperand2(opcode, carryOut);
    }
    
    uint32_t address = registers[rn];
    
    if (preIndex) {
        address += up ? offset : -offset;
    }

    if (load) {
        uint32_t value;
        if (byteTransfer) {
            value = readMemory(address, 1);
            std::cout << "LDRB instruction, Rd: R" << rd << ", Address: 0x" 
                      << std::hex << address << std::endl;
        } else {
            value = readMemory(address, 4);
            std::cout << "LDR instruction, Rd: R" << rd << ", Address: 0x" 
                      << std::hex << address << std::endl;
        }
        registers[rd] = value;
    } else {
        if (byteTransfer) {
            writeMemory(address, registers[rd], 1);
            std::cout << "STRB instruction, Rd: R" << rd << ", Address: 0x" 
                      << std::hex << address << std::endl;
        } else {
            writeMemory(address, registers[rd], 4);
            std::cout << "STR instruction, Rd: R" << rd << ", Address: 0x" 
                      << std::hex << address << std::endl;
        }
    }
    
    if (!preIndex) {
        address += up ? offset : -offset;
    }
    
    if (writeBack || !preIndex) {
        registers[rn] = address;
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

    // Save return address in LR for BL (PC is already incremented)
    if (link) {
        registers[14] = pc; // Save return address in LR
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
    bool N = (cpsr & (1 << 31)) != 0;  // Negative flag
    bool Z = (cpsr & (1 << 30)) != 0;  // Zero flag
    bool C = (cpsr & (1 << 29)) != 0;  // Carry flag
    bool V = (cpsr & (1 << 28)) != 0;  // Overflow flag
    
    switch (condition) {
        case 0x0: return Z;              // EQ - Equal
        case 0x1: return !Z;             // NE - Not equal
        case 0x2: return C;              // CS/HS - Carry set/unsigned higher or same
        case 0x3: return !C;             // CC/LO - Carry clear/unsigned lower
        case 0x4: return N;              // MI - Minus/negative
        case 0x5: return !N;             // PL - Plus/positive or zero
        case 0x6: return V;              // VS - Overflow set
        case 0x7: return !V;             // VC - Overflow clear
        case 0x8: return C && !Z;        // HI - Unsigned higher
        case 0x9: return !C || Z;        // LS - Unsigned lower or same
        case 0xA: return N == V;         // GE - Signed greater than or equal
        case 0xB: return N != V;         // LT - Signed less than
        case 0xC: return !Z && (N == V); // GT - Signed greater than
        case 0xD: return Z || (N != V);  // LE - Signed less than or equal
        case 0xE: return true;           // AL - Always (unconditional)
        case 0xF: return false;          // NV - Never (deprecated)
        default: return true;
    }
}

uint32_t CPU::readMemory(uint32_t address, int size) {
    if (!memory.isValidAddress(address)) {
        std::cerr << "Memory read error: Invalid address 0x" << std::hex << address << std::endl;
        return 0; // Handle error
    }
    
    switch (size) {
        case 1: return memory.read(address);
        case 2: return memory.read16(address);
        case 4: return memory.read32(address);
        default:
            std::cerr << "Invalid memory read size: " << size << std::endl;
            return 0;
    }
}

void CPU::writeMemory(uint32_t address, uint32_t value, int size) {
    if (!memory.isValidAddress(address)) {
        std::cerr << "Memory write error: Invalid address 0x" << std::hex << address << std::endl;
        return; // Handle error
    }
    if (!memory.isWritableAddress(address)) {
        std::cerr << "Memory write error: Read-only address 0x" << std::hex << address << std::endl;
        return; // Handle error
    }
    
    switch (size) {
        case 1: memory.write(address, value & 0xFF); break;
        case 2: memory.write16(address, value & 0xFFFF); break;
        case 4: memory.write32(address, value); break;
        default:
            std::cerr << "Invalid memory write size: " << size << std::endl;
            break;
    }
}

uint32_t CPU::performShift(uint32_t value, uint32_t shiftType, uint32_t shiftAmount, bool& carryOut) {
    carryOut = false; // Default carry out
    
    if (shiftAmount == 0) {
        return value; // No shift
    }
    
    switch (shiftType) {
        case 0: { // LSL (Logical Shift Left)
            if (shiftAmount > 32) {
                carryOut = false;
                return 0;
            } else if (shiftAmount == 32) {
                carryOut = (value & 1) != 0;
                return 0;
            } else {
                carryOut = (value & (1 << (32 - shiftAmount))) != 0;
                return value << shiftAmount;
            }
        }
        case 1: { // LSR (Logical Shift Right)
            if (shiftAmount > 32) {
                carryOut = false;
                return 0;
            } else if (shiftAmount == 32) {
                carryOut = (value & 0x80000000) != 0;
                return 0;
            } else {
                carryOut = (value & (1 << (shiftAmount - 1))) != 0;
                return value >> shiftAmount;
            }
        }
        case 2: { // ASR (Arithmetic Shift Right)
            if (shiftAmount >= 32) {
                if (value & 0x80000000) {
                    carryOut = true;
                    return 0xFFFFFFFF;
                } else {
                    carryOut = false;
                    return 0;
                }
            } else {
                carryOut = (value & (1 << (shiftAmount - 1))) != 0;
                return (int32_t)value >> shiftAmount;
            }
        }
        case 3: { // ROR (Rotate Right)
            shiftAmount %= 32;
            if (shiftAmount == 0) {
                return value;
            }
            carryOut = (value & (1 << (shiftAmount - 1))) != 0;
            return (value >> shiftAmount) | (value << (32 - shiftAmount));
        }
        default:
            return value;
    }
}

uint32_t CPU::getOperand2(uint32_t opcode, bool& carryOut) {
    carryOut = false;
    
    if (opcode & (1 << 25)) { // Immediate value
        uint32_t immediate = opcode & 0xFF;
        uint32_t rotateAmount = (opcode >> 8) & 0xF;
        rotateAmount *= 2; // Rotate amount is multiplied by 2
        
        if (rotateAmount != 0) {
            carryOut = (immediate & (1 << (rotateAmount - 1))) != 0;
            return (immediate >> rotateAmount) | (immediate << (32 - rotateAmount));
        }
        return immediate;
    } else { // Register with optional shift
        uint32_t rm = opcode & 0xF; // Source register
        uint32_t value = registers[rm];
        
        if ((opcode & 0x10) == 0) { // Immediate shift
            uint32_t shiftType = (opcode >> 5) & 0x3;
            uint32_t shiftAmount = (opcode >> 7) & 0x1F;
            return performShift(value, shiftType, shiftAmount, carryOut);
        } else { // Register shift
            uint32_t shiftType = (opcode >> 5) & 0x3;
            uint32_t rs = (opcode >> 8) & 0xF;
            uint32_t shiftAmount = registers[rs] & 0xFF;
            return performShift(value, shiftType, shiftAmount, carryOut);
        }
    }
}

void CPU::handleMultiply(uint32_t opcode) {
    bool accumulate = (opcode & (1 << 21)) != 0;  // Bit 21: MLA or MUL
    bool setFlags = (opcode & (1 << 20)) != 0;    // Bit 20: Set condition codes
    
    uint32_t rd = (opcode >> 16) & 0xF;  // Destination register
    uint32_t rn = (opcode >> 12) & 0xF;  // Accumulator register (for MLA)
    uint32_t rs = (opcode >> 8) & 0xF;   // Multiplier register
    uint32_t rm = opcode & 0xF;          // Multiplicand register
    
    uint32_t result = registers[rm] * registers[rs];
    
    if (accumulate) {
        result += registers[rn];
        std::cout << "MLA instruction, Rd: R" << rd << ", Rm: R" << rm 
                  << ", Rs: R" << rs << ", Rn: R" << rn << std::endl;
    } else {
        std::cout << "MUL instruction, Rd: R" << rd << ", Rm: R" << rm 
                  << ", Rs: R" << rs << std::endl;
    }
    
    registers[rd] = result;
    
    if (setFlags) {
        // Set N and Z flags
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
        
        // C and V flags are unpredictable for MUL/MLA
    }
}

void CPU::handleLoadStoreMultiple(uint32_t opcode) {
    bool load = (opcode & (1 << 20)) != 0;       // Bit 20: Load or Store
    bool writeBack = (opcode & (1 << 21)) != 0;  // Bit 21: Write-back
    bool psr = (opcode & (1 << 22)) != 0;        // Bit 22: PSR & force user mode
    bool up = (opcode & (1 << 23)) != 0;         // Bit 23: Up or Down
    bool preIndex = (opcode & (1 << 24)) != 0;   // Bit 24: Pre or Post indexing
    
    uint32_t rn = (opcode >> 16) & 0xF;          // Base register
    uint32_t registerList = opcode & 0xFFFF;     // Register list
    
    uint32_t address = registers[rn];
    uint32_t startAddress = address;
    
    // Count number of registers in list
    int numRegs = 0;
    for (int i = 0; i < 16; i++) {
        if (registerList & (1 << i)) {
            numRegs++;
        }
    }
    
    if (!up) {
        address -= numRegs * 4;
        startAddress = address;
    }
    
    if (preIndex) {
        if (up) address += 4;
        else address -= 4;
    }
    
    if (load) {
        std::cout << "LDM instruction, Base: R" << rn << ", Register list: 0x" 
                  << std::hex << registerList << std::endl;
        
        for (int i = 0; i < 16; i++) {
            if (registerList & (1 << i)) {
                if (!preIndex && up) address += 4;
                if (!preIndex && !up) address -= 4;
                
                registers[i] = readMemory(address, 4);
                
                if (preIndex && up) address += 4;
                if (preIndex && !up) address -= 4;
            }
        }
    } else {
        std::cout << "STM instruction, Base: R" << rn << ", Register list: 0x" 
                  << std::hex << registerList << std::endl;
        
        for (int i = 0; i < 16; i++) {
            if (registerList & (1 << i)) {
                if (!preIndex && up) address += 4;
                if (!preIndex && !up) address -= 4;
                
                writeMemory(address, registers[i], 4);
                
                if (preIndex && up) address += 4;
                if (preIndex && !up) address -= 4;
            }
        }
    }
    
    if (writeBack) {
        if (up) {
            registers[rn] = startAddress + numRegs * 4;
        } else {
            registers[rn] = startAddress;
        }
    }
}

void CPU::handleLoadStoreHalfword(uint32_t opcode) {
    bool load = (opcode & (1 << 20)) != 0;       // Bit 20: Load or Store
    bool writeBack = (opcode & (1 << 21)) != 0;  // Bit 21: Write-back
    bool immediate = (opcode & (1 << 22)) != 0;  // Bit 22: Immediate offset
    bool up = (opcode & (1 << 23)) != 0;         // Bit 23: Up or Down
    bool preIndex = (opcode & (1 << 24)) != 0;   // Bit 24: Pre or Post indexing
    
    uint32_t rn = (opcode >> 16) & 0xF;          // Base register
    uint32_t rd = (opcode >> 12) & 0xF;          // Destination/source register
    
    uint32_t offset;
    if (immediate) {
        // Immediate offset: bits [11:8][3:0]
        offset = ((opcode >> 4) & 0xF0) | (opcode & 0xF);
    } else {
        // Register offset
        uint32_t rm = opcode & 0xF;
        offset = registers[rm];
    }
    
    uint32_t address = registers[rn];
    
    if (preIndex) {
        address += up ? offset : -offset;
    }
    
    uint32_t shType = (opcode >> 5) & 0x3;  // Bits [6:5] - SH field
    
    if (load) {
        uint32_t value;
        switch (shType) {
            case 1: { // LDRH - Load halfword
                value = readMemory(address, 2);
                std::cout << "LDRH instruction, Rd: R" << rd << ", Address: 0x" 
                          << std::hex << address << std::endl;
                break;
            }
            case 2: { // LDRSB - Load signed byte
                value = readMemory(address, 1);
                if (value & 0x80) value |= 0xFFFFFF00; // Sign extend
                std::cout << "LDRSB instruction, Rd: R" << rd << ", Address: 0x" 
                          << std::hex << address << std::endl;
                break;
            }
            case 3: { // LDRSH - Load signed halfword
                value = readMemory(address, 2);
                if (value & 0x8000) value |= 0xFFFF0000; // Sign extend
                std::cout << "LDRSH instruction, Rd: R" << rd << ", Address: 0x" 
                          << std::hex << address << std::endl;
                break;
            }
            default:
                std::cout << "Invalid halfword load instruction" << std::endl;
                return;
        }
        registers[rd] = value;
    } else {
        // STRH - Store halfword
        writeMemory(address, registers[rd], 2);
        std::cout << "STRH instruction, Rd: R" << rd << ", Address: 0x" 
                  << std::hex << address << std::endl;
    }
    
    if (!preIndex) {
        address += up ? offset : -offset;
    }
    
    if (writeBack || !preIndex) {
        registers[rn] = address;
    }
}
