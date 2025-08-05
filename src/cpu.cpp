#include "cpu.h"
#include <iostream>
#include <iomanip>
#include <stdexcept>

// Popcount fallback
#if defined(__GNUC__) || defined(__clang__)
#define popcount __builtin_popcount
#else
int popcount(uint32_t i) {
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}
#endif

CPU::CPU(Memory& mem) : memory(mem) {
    reset();
}

void CPU::reset() {
    for (int i = 0; i < 13; ++i) { r[i] = 0; }
    cpsr = SVC_MODE | F_FLAG | I_FLAG;
    sp_usr = 0x03007F00; sp_svc = 0x03007FE0; sp_irq = 0x03007FA0;
    lr_usr = 0; lr_svc = 0; lr_irq = 0;
    spsr_svc = 0; spsr_irq = 0;
    pc = 0x08000000;
    ime = false; halted = false;
}

uint32_t CPU::getRegister(int reg) const {
    if (reg < 13) return r[reg];
    if (reg == PC) return pc + (isThumbMode() ? 4 : 8);
    switch (cpsr & MODE_MASK) {
        case SVC_MODE: if (reg == SP) return sp_svc; else if (reg == LR) return lr_svc; break;
        case IRQ_MODE: if (reg == SP) return sp_irq; else if (reg == LR) return lr_irq; break;
        default: if (reg == SP) return sp_usr; else if (reg == LR) return lr_usr; break;
    }
    return 0;
}
void CPU::setRegister(int reg, uint32_t value) {
    if (reg < 13) { r[reg] = value; return; }
    if (reg == PC) { pc = value; return; }
    switch (cpsr & MODE_MASK) {
        case SVC_MODE: if (reg == SP) sp_svc = value; else if (reg == LR) lr_svc = value; break;
        case IRQ_MODE: if (reg == SP) sp_irq = value; else if (reg == LR) lr_irq = value; break;
        default: if (reg == SP) sp_usr = value; else if (reg == LR) lr_usr = value; break;
    }
}
uint32_t CPU::getSpsr() const { switch (cpsr & MODE_MASK) { case SVC_MODE: return spsr_svc; case IRQ_MODE: return spsr_irq; default: return cpsr; } }

void CPU::setSpsr(uint32_t value) { switch (cpsr & MODE_MASK) { case SVC_MODE: spsr_svc = value; break; case IRQ_MODE: spsr_irq = value; break; default: break; } }

void CPU::switchMode(CpuMode newMode) { cpsr = (cpsr & ~MODE_MASK) | newMode; }

int CPU::step() {
    uint32_t currentExecutingAddress = pc;
    if (pc < 0x8000000 || pc >= 0x10000000) {
        std::cerr << "[cpu.cpp:" << __LINE__ << "]: PC outside valid memory range: 0x" 
                  << std::hex << pc << std::dec << std::endl;
        handleUndefinedInstruction();
        return 1;
    }
    if (halted) {
        uint16_t ie = readMemory16(REG_IE), iflag = readMemory16(REG_IF);
        if (ie & iflag) halted = false;
        return 1;
    }
    checkAndHandleInterrupts();
    if (currentExecutingAddress != pc) return 1;
    if (isThumbMode()) {
        uint16_t opcode = fetchThumbOpcode();
        std::cout << "[cpu.cpp:" << __LINE__ << "]: " << "Thumb PC: 0x" << std::hex << pc << " Opcode: 0x" << std::setw(4) << std::setfill('0') << opcode << std::dec << std::endl;
        decodeThumb(opcode);
        if (pc == currentExecutingAddress) {
            pc += 2;
            std::cout << "[PC_ADVANCE] Advanced PC from 0x" << std::hex << currentExecutingAddress 
                      << " to 0x" << pc << std::dec << std::endl;
        } else {
            std::cout << "[PC_BRANCH] PC changed from 0x" << std::hex << currentExecutingAddress 
                      << " to 0x" << pc << std::dec << std::endl;
        }
    } else {
        uint32_t opcode = fetchArmOpcode();
        std::cout << "[cpu.cpp:" << __LINE__ << "]: " << "ARM   PC: 0x" << std::hex << pc << " Opcode: 0x" << std::setw(8) << std::setfill('0') << opcode << std::dec << std::endl;
        if (checkCondition(opcode >> 28)) decodeArm(opcode);
        if (pc == currentExecutingAddress) {
            pc += 4;
            std::cout << "[PC_ADVANCE] Advanced PC from 0x" << std::hex << currentExecutingAddress 
                      << " to 0x" << pc << std::dec << std::endl;
        } else {
            std::cout << "[PC_BRANCH] PC changed from 0x" << std::hex << currentExecutingAddress 
                      << " to 0x" << pc << std::dec << std::endl;
        }
    }
    return 1;
}

// --- ARM DECODER ---
void CPU::decodeArm(uint32_t opcode) {
    if ((opcode & 0x0FFFFFF0) == 0x012FFF10) { handleBranchExchange(opcode); }
    else if ((opcode & 0x0FBF0FFF) == 0x010F0000 || (opcode & 0x0FB00000) == 0x03200000 || (opcode & 0x0FB00000) == 0x01200000) { handleMsr(opcode); }
    else if (((opcode >> 25) & 0b111) == 0b101)   { handleBranch(opcode); }
    else if (((opcode >> 26) & 0b11) == 0b01)     { handleLoadStore(opcode); }
    else if (((opcode >> 26) & 0b11) == 0b00)     { handleDataProcessing(opcode); }
    else { std::cerr << "[cpu.cpp:" << __LINE__ << "]: Unhandled ARM Opcode 0x" << std::hex << opcode << " at PC=0x" << pc - 4 << std::endl; }
}

void CPU::handleDataProcessing(uint32_t opcode) {
    uint32_t rn_idx = (opcode >> 16) & 0xF; uint32_t rd_idx = (opcode >> 12) & 0xF;
    bool setFlags = (opcode >> 20) & 1;
    uint32_t op1 = getRegister(rn_idx);
    if(rn_idx == PC) op1 = pc + 4; // PC is pipelined
    uint32_t op2;

    if (opcode & (1 << 25)) { // Immediate
        uint32_t imm = opcode & 0xFF; uint32_t rot = ((opcode >> 8) & 0xF) * 2;
        op2 = (imm >> rot) | (imm << (32 - rot));
    } else { // Register
        op2 = getRegister(opcode & 0xF);
    }
    
    uint8_t opc = (opcode >> 21) & 0xF; uint32_t result = 0;
    bool write = !(opc >= 0x8 && opc <= 0xB);

    switch(opc) {
        case 0x0: result=op1 & op2; break;   // AND
        case 0x1: result=op1 ^ op2; break;   // EOR
        case 0x2: result=op1 - op2; break;   // SUB
        case 0x3: result=op2 - op1; break;   // RSB
        case 0x4: result=op1 + op2; break;   // ADD
        case 0x8: result=op1 & op2; break;   // TST
        case 0x9: result=op1 ^ op2; break;   // TEQ
        case 0xA: result=op1 - op2; break;   // CMP
        case 0xB: result=op1 + op2; break;   // CMN
        case 0xC: result=op1 | op2; break;   // ORR
        case 0xD: result=op2; break;         // MOV
        case 0xE: result=op1 & ~op2; break;  // BIC
        case 0xF: result=~op2; break;        // MVN
    }
    
    if (setFlags && rd_idx == PC) { cpsr = getSpsr(); }
    else if(setFlags) {
        updateNZFlags(result);
        if(opc==2||opc==3||opc==0xA) updateCVFlagsForSub(op1,op2,result);
        else if (opc==4||opc==0xB) updateCVFlagsForAdd(op1,op2,result);
    }
    if(write) { setRegister(rd_idx, result); }
}

void CPU::handleLoadStore(uint32_t opcode) {
    uint32_t rd = (opcode>>12)&0xF, rn=(opcode>>16)&0xF;
    bool is_load = (opcode>>20)&1, pre_index=(opcode>>24)&1, add_offset=(opcode>>23)&1;
    bool byte_transfer=(opcode>>22)&1, writeback=(opcode>>21)&1;
    
    uint32_t offset = (opcode & 0xFFF); // Simple immediate offset
    uint32_t base = getRegister(rn);
    
    uint32_t addr = pre_index ? (add_offset ? base+offset : base-offset) : base;

    if (is_load) {
        uint32_t data = byte_transfer ? readMemory8(addr) : readMemory32(addr);
        setRegister(rd, data);
    } else { // Store
        writeMemory32(addr, getRegister(rd));
    }

    if (!pre_index || writeback) {
        setRegister(rn, add_offset ? base+offset : base-offset);
    }
}

void CPU::checkAndHandleInterrupts(){
    if (ime) {
        if (!(cpsr & I_FLAG)) {
            uint16_t ie = readMemory16(REG_IE);
            uint16_t iflag = readMemory16(REG_IF);
            uint16_t pending = ie & iflag;

            if (pending) {
                std::cout << "[cpu.cpp:" << __LINE__ << "]: " << "IRQ triggered! IE=0x" << std::hex << ie << " IF=0x" << iflag << std::dec << std::endl;
                setSpsr(cpsr);
                switchMode(IRQ_MODE);
                setRegister(LR, pc + (isThumbMode() ? 2:4) );

                cpsr |= I_FLAG;
                cpsr &= ~T_BIT;
                pc = 0x18;
            }
        }
    }
}

void CPU::handleUndefinedInstruction() {
    // Save current PC and CPSR
    setSpsr(cpsr);
    switchMode(UND_MODE);
    setRegister(LR, getRegister(PC) - (isThumbMode() ? 2 : 4));
    
    // Switch to ARM mode and disable interrupts
    cpsr &= ~T_BIT;
    cpsr |= I_FLAG;
    
    // Jump to undefined instruction vector
    pc = 0x04;  // Undefined instruction vector address
}

bool CPU::isValidMemoryAddress(uint32_t address) const {
    // ROM region
    if (address >= 0x08000000 && address < 0x0E000000) {
        return false;  // Can't write to ROM
    }
    
    // I/O registers (0x04000000-0x04000400)
    if (address >= 0x04000000 && address < 0x04000400) {
        return true;  // Allow I/O register writes
    }
    
    // Valid regions
    return (address >= 0x02000000 && address < 0x03000000) ||  // WRAM
           (address >= 0x03000000 && address < 0x04000000) ||  // IWRAM
           (address >= 0x05000000 && address < 0x06000000) ||  // Palette
           (address >= 0x06000000 && address < 0x07000000) ||  // VRAM
           (address >= 0x07000000 && address < 0x08000000);    // OAM
}

void CPU::handleSWI(uint8_t swiNumber) {
    std::cout << "[SWI] Handling SWI #" << (int)swiNumber << " at PC=0x" << std::hex << pc << std::dec << std::endl;
    
    switch (swiNumber) {
        case 0x00: // In this mode 3 test ROM, SWI 0 is used for VBlankIntrWait oops remember to delete in later stages
            std::cout << "[SWI] Treating SWI #0 as VBlankIntrWait" << std::endl;
            // Simulate VBlank interrupt occurred
            memory.setIME(true);
            memory.write16(REG_IF, memory.read16(REG_IF) | 1); // Set VBlank flag
            break;
        
        case 0x04: // IntrWait
            {
                // IntrWait(bool discard_current, uint16_t wait_flags)
                bool discard_current = getRegister(0) & 1;
                uint16_t wait_flags = getRegister(1) & 0xFFFF;
                
                std::cout << "[SWI] IntrWait: discard_current=" << discard_current 
                          << " wait_flags=0x" << std::hex << wait_flags << std::dec << std::endl;
                
                // Immediately return - in real hardware this would wait for interrupts
                // For now, just pretend an interrupt occurred
                ime = true;
                uint16_t if_flags = readMemory16(REG_IF);
                writeMemory16(REG_IF, if_flags | 1); // Set VBlank flag
            }
            break;
            
        case 0x05: // VBlankIntrWait
            {
                std::cout << "[SWI] VBlankIntrWait" << std::endl;
                
                // This is equivalent to IntrWait(true, 1)
                // For now, just pretend a VBlank interrupt occurred
                ime = true;
                uint16_t if_flags = readMemory16(REG_IF);
                writeMemory16(REG_IF, if_flags | 1); // Set VBlank flag
                
                // In a real emulator, you'd wait for the PPU to signal VBlank
                // But for now, just pretend it happened
            }
            break;
            
        /*case 0x00: // SoftReset
            std::cout << "[SWI] SoftReset" << std::endl;
            reset();
            break;
        */    
        case 0x01: // RegisterRamReset
            std::cout << "[SWI] RegisterRamReset" << std::endl;
            // Clear memory areas based on r0
            break;
            
        case 0x0E: // BgAffineSet
        case 0x0F: // ObjAffineSet
            std::cout << "[SWI] AffineSet" << std::endl;
            // These are math operations for transformations
            // For now, we'll skip implementation
            break;
            
        default:
            std::cout << "[SWI] Unhandled SWI number: " << (int)swiNumber << std::endl;
            break;
    }
}

void CPU::handleBranch(uint32_t opcode) { int32_t offset = ((int32_t)((opcode&0xFFFFFF)<<8))>>6; branch(getRegister(PC) + offset, (opcode>>24)&1); }

void CPU::handleBranchExchange(uint32_t opcode){ uint32_t rn_val = getRegister(opcode & 0xF); if(rn_val & 1)cpsr|=T_BIT; else cpsr&=~T_BIT; pc = rn_val & ~1;}

void CPU::handleMsr(uint32_t opcode) {
    bool to_spsr = (opcode>>22)&1;
    if((opcode&(1<<25))==0){
        uint32_t mask=(opcode>>16)&0xF; uint32_t rm_val=getRegister(opcode&0xF);
        if(to_spsr)setSpsr(rm_val); else { if((mask&1)) cpsr = (cpsr&0xFFFFFF00)|(rm_val&0xFF); }
    } else { //imm
        uint32_t imm=opcode&0xFF, rot=((opcode>>8)&0xF)*2; uint32_t val=(imm>>rot)|(imm<<(32-rot));
        if(!to_spsr) cpsr=(cpsr&~0xFF000000)|(val&0xFF000000);
    }
}

void CPU::dumpROMArea(uint32_t address, int num_bytes = 32) {
    std::cout << "\n=== ROM DUMP around 0x" << std::hex << address << " ===" << std::dec << std::endl;
    
    uint32_t start_addr = address & 0xFFFFFFFE;  // Align to even address
    for (int i = -16; i < num_bytes; i += 2) {
        uint32_t addr = start_addr + i;
        if (addr >= 0x8000000 && addr < 0x10000000) {
            uint16_t data = readMemory16(addr);
            std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << addr 
                      << ": 0x" << std::setw(4) << data;
            
            // Add instruction interpretation
            if (addr == address) std::cout << " <-- CURRENT";
            if ((data & 0xF800) == 0xF000) std::cout << " [BL_FIRST]";
            if ((data & 0xF800) == 0xF800) std::cout << " [BL_SECOND]";
            if ((data & 0xF000) == 0xD000) std::cout << " [COND_BRANCH]";
            
            std::cout << std::dec << std::endl;
        }
    }
    std::cout << "=== END ROM DUMP ===" << std::endl << std::endl;
}

// THUMB implementation 
void CPU::decodeThumb(uint16_t opcode) {
    //std::cout << "[DECODE_THUMB] Enter: PC=0x" << std::hex << pc 
    //          << " opcode=0x" << std::setw(4) << std::setfill('0') << opcode << std::dec << std::endl;
    
    // Format 1/2: Move shifted register (0x0000-0x07FF)
    if (((opcode >> 11) & 0b11111) <= 0b00010) {
        uint8_t opcode_type = (opcode >> 11) & 0x3;  // 00: LSL, 01: LSR, 10: ASR
        uint8_t offset = (opcode >> 6) & 0x1F;       // 5-bit immediate for shift amount
        uint8_t rs = (opcode >> 3) & 0x7;            // Source register
        uint8_t rd = opcode & 0x7;                   // Destination register
        
        uint32_t value = getRegister(rs);
        uint32_t result = 0;
        bool carry_out = cpsr & C_FLAG;  // Default to current carry flag
        
        switch (opcode_type) {
            case 0: // LSL
                if (offset == 0) {
                    result = value;
                } else {
                    carry_out = (value >> (32 - offset)) & 1;
                    result = value << offset;
                }
                break;
            case 1: // LSR
                if (offset == 0) {  // Special case: LSR #32
                    carry_out = value >> 31;
                    result = 0;
                } else {
                    carry_out = (value >> (offset - 1)) & 1;
                    result = value >> offset;
                }
                break;
            case 2: // ASR
                if (offset == 0) {  // Special case: ASR #32
                    carry_out = value >> 31;
                    result = (value & 0x80000000) ? 0xFFFFFFFF : 0;
                } else {
                    carry_out = (value >> (offset - 1)) & 1;
                    // Arithmetic shift maintains sign bit
                    result = ((int32_t)value) >> offset;
                }
                break;
        }
        
        setRegister(rd, result);
        
        // Update flags
        updateNZFlags(result);
        if (carry_out) cpsr |= C_FLAG;
        else cpsr &= ~C_FLAG;
        
        //std::cout << "[DECODE_THUMB] Exit: Format 1/2 completed" << std::endl;
        return;
    }
    
    // Format 3: Add/subtract (0x1800-0x1FFF)
    else if (((opcode >> 11) & 0b11111) == 0b00011) {
        bool immediate_operand = (opcode >> 10) & 1;
        bool is_subtraction = (opcode >> 9) & 1;
        uint8_t rn_or_imm3 = (opcode >> 6) & 0x7;
        uint8_t rs = (opcode >> 3) & 0x7;
        uint8_t rd = opcode & 0x7;
        
        uint32_t op1 = getRegister(rs);
        uint32_t op2 = immediate_operand ? rn_or_imm3 : getRegister(rn_or_imm3);
        uint32_t result;
        
        if (is_subtraction) {
            result = op1 - op2;
            updateCVFlagsForSub(op1, op2, result);
        } else {
            result = op1 + op2;
            updateCVFlagsForAdd(op1, op2, result);
        }
        
        setRegister(rd, result);
        updateNZFlags(result);
        
        //std::cout << "[DECODE_THUMB] Exit: Format 3 completed" << std::endl;
        return;
    }
    
    // Format 4/5: MOV/CMP/ADD/SUB immediate (0x2000-0x3FFF)
    else if (((opcode >> 13) & 0b111) == 0b001) {
        uint8_t op = (opcode >> 11) & 0x3;
        uint8_t rd = (opcode >> 8) & 0x7;
        uint8_t imm8 = opcode & 0xFF;
        
        uint32_t rd_val = getRegister(rd);
        uint32_t result;
        
        switch (op) {
            case 0: // MOV
                result = imm8;
                setRegister(rd, result);
                break;
            case 1: // CMP
                result = rd_val - imm8;
                updateCVFlagsForSub(rd_val, imm8, result);
                break;
            case 2: // ADD
                result = rd_val + imm8;
                setRegister(rd, result);
                updateCVFlagsForAdd(rd_val, imm8, result);
                break;
            case 3: // SUB
                result = rd_val - imm8;
                setRegister(rd, result);
                updateCVFlagsForSub(rd_val, imm8, result);
                break;
        }
        
        updateNZFlags(result);
        
        //std::cout << "[DECODE_THUMB] Exit: Format 4/5 completed" << std::endl;
        return;
    }
    
    // Format 6: ALU operations (0x4000-0x43FF)
    else if ((opcode & 0xFFC0) == 0x4000) {
        uint8_t op = (opcode >> 6) & 0xF;
        uint8_t rs = (opcode >> 3) & 0x7;
        uint8_t rd = opcode & 0x7;
        
        uint32_t rd_val = getRegister(rd);
        uint32_t rs_val = getRegister(rs);
        uint32_t result = 0;
        
        switch (op) {
            case 0x0: // AND
                result = rd_val & rs_val;
                setRegister(rd, result);
                break;
            case 0x1: // EOR (XOR)
                result = rd_val ^ rs_val;
                setRegister(rd, result);
                break;
            case 0x2: // LSL
                {
                    uint8_t shift = rs_val & 0xFF;
                    if (shift == 0) {
                        result = rd_val;
                    } else if (shift < 32) {
                        bool carry_out = (shift > 0) ? (rd_val >> (32 - shift)) & 1 : 0;
                        result = rd_val << shift;
                        if (carry_out) cpsr |= C_FLAG; else cpsr &= ~C_FLAG;
                    } else {
                        result = 0;
                        if (shift == 32) cpsr = (cpsr & ~C_FLAG) | ((rd_val & 1) ? C_FLAG : 0);
                        else cpsr &= ~C_FLAG;
                    }
                    setRegister(rd, result);
                }
                break;
            case 0x3: // LSR
                {
                    uint8_t shift = rs_val & 0xFF;
                    if (shift == 0) {
                        result = rd_val;
                    } else if (shift < 32) {
                        bool carry_out = (shift > 0) ? (rd_val >> (shift - 1)) & 1 : 0;
                        result = rd_val >> shift;
                        if (carry_out) cpsr |= C_FLAG; else cpsr &= ~C_FLAG;
                    } else {
                        result = 0;
                        if (shift == 32) cpsr = (cpsr & ~C_FLAG) | ((rd_val >> 31) ? C_FLAG : 0);
                        else cpsr &= ~C_FLAG;
                    }
                    setRegister(rd, result);
                }
                break;
            case 0x4: // ASR
                {
                    uint8_t shift = rs_val & 0xFF;
                    if (shift == 0) {
                        result = rd_val;
                    } else if (shift < 32) {
                        bool carry_out = (shift > 0) ? (rd_val >> (shift - 1)) & 1 : 0;
                        result = (int32_t)rd_val >> shift;
                        if (carry_out) cpsr |= C_FLAG; else cpsr &= ~C_FLAG;
                    } else {
                        result = (rd_val & 0x80000000) ? 0xFFFFFFFF : 0;
                        cpsr = (cpsr & ~C_FLAG) | ((rd_val >> 31) ? C_FLAG : 0);
                    }
                    setRegister(rd, result);
                }
                break;
            case 0x5: // ADC
                {
                    bool carry = (cpsr & C_FLAG) ? 1 : 0;
                    result = rd_val + rs_val + carry;
                    setRegister(rd, result);
                    updateCVFlagsForAdd(rd_val, rs_val + carry, result);
                }
                break;
            case 0x6: // SBC
                {
                    bool carry = (cpsr & C_FLAG) ? 1 : 0;
                    result = rd_val - rs_val - (!carry);
                    setRegister(rd, result);
                    updateCVFlagsForSub(rd_val, rs_val + (!carry), result);
                }
                break;
            case 0x7: // ROR
                {
                    uint8_t shift = rs_val & 0x1F;
                    if (shift == 0) {
                        result = rd_val;
                    } else {
                        result = (rd_val >> shift) | (rd_val << (32 - shift));
                        bool carry_out = (result >> 31) & 1;
                        if (carry_out) cpsr |= C_FLAG; else cpsr &= ~C_FLAG;
                    }
                    setRegister(rd, result);
                }
                break;
            case 0x8: // TST
                result = rd_val & rs_val;
                break;
            case 0x9: // NEG
                result = 0 - rs_val;
                setRegister(rd, result);
                updateCVFlagsForSub(0, rs_val, result);
                break;
            case 0xA: // CMP
                result = rd_val - rs_val;
                updateCVFlagsForSub(rd_val, rs_val, result);
                break;
            case 0xB: // CMN
                result = rd_val + rs_val;
                updateCVFlagsForAdd(rd_val, rs_val, result);
                break;
            case 0xC: // ORR
                result = rd_val | rs_val;
                setRegister(rd, result);
                break;
            case 0xD: // MUL
                result = rd_val * rs_val;
                setRegister(rd, result);
                break;
            case 0xE: // BIC
                result = rd_val & ~rs_val;
                setRegister(rd, result);
                break;
            case 0xF: // MVN
                result = ~rs_val;
                setRegister(rd, result);
                break;
        }
        
        updateNZFlags(result);
        
        //std::cout << "[DECODE_THUMB] Exit: Format 6 completed" << std::endl;
        return;
    }
    
     // For MULS (0x4391)
    else if ((opcode & 0xFF80) == 0x4380) {  // MULS instruction (0x4340-0x437F)
        uint8_t rd = opcode & 0x7;
        uint8_t rm = (opcode >> 3) & 0x7;
        uint32_t result = getRegister(rd) * getRegister(rm);
        setRegister(rd, result);
        updateNZFlags(result);
        // Note: GBA MULS doesn't update carry or overflow flags
        
        //std::cout << "[DECODE_THUMB] Exit: MULS completed" << std::endl;
        return;  // Add return to prevent falling through
    }
    
    // Handle all high register operations and BX (0x4400-0x47FF)
    else if ((opcode & 0xFF00) == 0x4400 || (opcode & 0xFF00) == 0x4600 || (opcode & 0xFF00) == 0x4700) {
        uint8_t op = (opcode >> 8) & 0x3;
        uint8_t rd = (opcode & 7) | ((opcode >> 4) & 8);
        uint8_t rm = (opcode >> 3) & 0xF;
        uint32_t rm_val = getRegister(rm);
        
        switch (op) {
            case 0: // ADD
                setRegister(rd, getRegister(rd) + rm_val);
                break;
            case 1: // CMP
                {
                    uint32_t result = getRegister(rd) - rm_val;
                    updateNZFlags(result);
                    updateCVFlagsForSub(getRegister(rd), rm_val, result);
                }
                break;
            case 2: // MOV
                if (rd == PC) {
                    branch(rm_val & ~1, false);
                    if (rm_val & 1) cpsr |= T_BIT;
                } else {
                    setRegister(rd, rm_val);
                }
                break;
            case 3: // BX/BLX
                if ((opcode & 0x0080) == 0) {
                    if ((rm_val & ~1) < 0x8000000 || (rm_val & ~1) >= 0x10000000) {
                        std::cerr << "[cpu.cpp:" << __LINE__ << "]: Invalid branch target: 0x" 
                                << std::hex << rm_val << std::dec << std::endl;
                        return;
                    }
                    if (rm_val & 1) cpsr |= T_BIT;
                    else cpsr &= ~T_BIT;
                    branch(rm_val & ~1, false);
                }
                break;
        }
        
        //std::cout << "[DECODE_THUMB] Exit: High register/BX completed" << std::endl;
        return;
    }
    
    // Format 8: PC-relative LDR (0x4800-0x4FFF)
    else if ((opcode & 0xF800) == 0x4800) {
        uint8_t rd = (opcode >> 8) & 0x7;
        uint16_t imm8 = opcode & 0xFF;
        uint32_t addr = ((pc + 4) & ~2) + (imm8 << 2);  // PC is aligned to 4 bytes
        
        setRegister(rd, readMemory32(addr));
        
        //std::cout << "[DECODE_THUMB] Exit: PC-relative LDR completed" << std::endl;
        return;
    }
    
    // Format 9: STR/LDR with immediate offset (0x6000-0x6FFF)
    else if ((opcode & 0xF000) == 0x6000) {
        uint8_t rd = opcode & 0x7;                    // destination register
        uint8_t rb = (opcode >> 3) & 0x7;            // base register
        uint32_t offset = (opcode >> 6) & 0x1F;      // 5-bit immediate offset
        bool is_load = (opcode >> 11) & 1;           // 1 for LDR, 0 for STR
        uint32_t addr = getRegister(rb) + (offset << 2); // offset is multiplied by 4

        if (is_load) {
            setRegister(rd, readMemory32(addr));
        } else {
            writeMemory32(addr, getRegister(rd));
        }
        
        //std::cout << "[DECODE_THUMB] Exit: Format 9 completed" << std::endl;
        return;
    }

    // Format 10-11: All other load/store operations (0x5000-0x5FFF, 0x7000-0x7FFF)
    else if ((opcode & 0xE000) == 0x5000) {
        uint8_t rd = opcode & 0x7;
        uint8_t rb = (opcode >> 3) & 0x7;
        uint32_t addr;
        
        if ((opcode & 0xF200) == 0x5000) {
            // Register offset
            uint8_t ro = (opcode >> 6) & 0x7;
            addr = getRegister(rb) + getRegister(ro);
        } else if ((opcode & 0xF000) == 0x8000) {
            // Immediate offset for H transfer
            addr = getRegister(rb) + (((opcode >> 6) & 0x1F) << 1);
        } else {
            // Immediate offset
            addr = getRegister(rb) + (((opcode >> 6) & 0x1F) << 2);
        }
        
        bool is_load = (opcode >> 11) & 1;
        uint8_t op = (opcode >> 10) & 3;
        
        if (is_load) {
            switch (op) {
                case 0: setRegister(rd, readMemory32(addr)); break;
                case 1: setRegister(rd, (int32_t)(int8_t)readMemory8(addr)); break;
                case 2: setRegister(rd, readMemory8(addr)); break;
                case 3: setRegister(rd, readMemory16(addr)); break;
            }
        } else {
            switch (op) {
                case 0: writeMemory32(addr, getRegister(rd)); break;
                case 1: writeMemory16(addr, getRegister(rd)); break;
                case 2: writeMemory8(addr, getRegister(rd)); break;
                case 3: writeMemory16(addr, getRegister(rd)); break;
            }
        }
        
        //std::cout << "[DECODE_THUMB] Exit: Format 10-11 completed" << std::endl;
        return;
    }
    
    // Format 12-15: Stack operations and load/store multiple (0x9000-0xBFFF)
    else if ((opcode & 0xE000) == 0xB000) {
        if ((opcode & 0xF600) == 0xB400) {
            // PUSH/POP
            bool is_pop = (opcode >> 11) & 1;
            bool pc_lr = (opcode >> 8) & 1;
            uint8_t rlist = opcode & 0xFF;
            uint32_t sp = getRegister(SP);
            
            if (!is_pop) {
                if (pc_lr) {
                    sp -= 4;
                    writeMemory32(sp, getRegister(LR));
                }
                for (int i = 7; i >= 0; --i) {
                    if (rlist & (1 << i)) {
                        sp -= 4;
                        writeMemory32(sp, getRegister(i));
                    }
                }
            } else {
                for (int i = 0; i < 8; i++) {
                    if (rlist & (1 << i)) {
                        setRegister(i, readMemory32(sp));
                        sp += 4;
                    }
                }
                if (pc_lr) {
                    setRegister(PC, readMemory32(sp) & ~1);
                    sp += 4;
                }
            }
            setRegister(SP, sp);
        }
        else if ((opcode & 0xFF80) == 0xB080) {
            // ADD/SUB SP
            uint32_t sp = getRegister(SP);
            uint32_t imm = (opcode & 0x7F) << 2;
            setRegister(SP, (opcode & 0x80) ? sp - imm : sp + imm);
        }
        
        //std::cout << "[DECODE_THUMB] Exit: Format 12-15 completed" << std::endl;
        return;
    }
    
    // Format 16: Load/store multiple (LDMIA/STMIA) (0xC000-0xCFFF)
    else if ((opcode & 0xF000) == 0xC000) { 
        bool is_load = (opcode >> 11) & 1;
        uint8_t rb = (opcode >> 8) & 7;
        uint8_t rlist = opcode & 0xFF;
        uint32_t addr = getRegister(rb);
        
        // If the address is in ROM, try to use WRAM instead
        if (addr >= 0x8000000 && addr < 0x10000000) {
            addr = 0x2000000 | (addr & 0xFFFF);  // Map to WRAM
            std::cout << "[cpu.cpp:" << __LINE__ << "]: Redirecting ROM access to WRAM: 0x" 
                      << std::hex << addr << std::dec << std::endl;
        }
        
        // Calculate number of registers to transfer
        int count = 0;
        for (int i = 0; i < 8; i++) {
            if (rlist & (1 << i)) count++;
        }
        
        uint32_t final_addr = addr + (count * 4);
        uint32_t curr_addr = addr;

        if (is_load) {  // LDMIA
            for (int i = 0; i < 8; i++) {
                if (rlist & (1 << i)) {
                    if (isValidMemoryAddress(curr_addr)) {
                        setRegister(i, readMemory32(curr_addr));
                    }
                    curr_addr += 4;
                }
            }
        } else {  // STMIA
            for (int i = 0; i < 8; i++) {
                if (rlist & (1 << i)) {
                    if (isValidMemoryAddress(curr_addr)) {
                        writeMemory32(curr_addr, getRegister(i));
                    }
                    curr_addr += 4;
                }
            }
        }
        
        // Write-back if Rb is not in Rlist
        if (!(rlist & (1 << rb))) {
            setRegister(rb, final_addr);
        }
        
        //std::cout << "[DECODE_THUMB] Exit: Format 16 completed" << std::endl;
        return;
    }
    
    // Format 17-18: Conditional branch and SWI (0xD000-0xDFFF)
    else if ((opcode & 0xF000) == 0xD000) {
        uint8_t cond = (opcode >> 8) & 0xF;
        if (cond == 0xF) {
             // Extract SWI number from the instruction
            uint8_t swiNumber = opcode & 0xFF;
            
            // Save state as if we were going to jump to BIOS
            setSpsr(cpsr);
            switchMode(SVC_MODE);
            setRegister(LR, getRegister(PC));
            
            // Instead of jumping to vector, call our handler
            handleSWI(swiNumber);
            
            // Don't modify PC, let the normal PC+2 happen
            //std::cout << "[DECODE_THUMB] Exit: SWI emulated" << std::endl;
            return;
        } else {
            // Conditional branch
            int8_t offset = (int8_t)(opcode & 0xFF);
            uint32_t target = pc + 2 + (offset << 1);
            bool should_branch = checkCondition(cond);
            
            std::cout << "[COND_BRANCH] PC=0x" << std::hex << pc 
                      << " opcode=0x" << std::setw(4) << std::setfill('0') << opcode
                      << " cond=0x" << (int)cond << " offset=" << std::dec << (int)offset
                      << " target=0x" << std::hex << target 
                      << " CPSR=0x" << cpsr << " branch=" << should_branch << std::dec << std::endl;
            
            if (should_branch) {
                pc = target;
            }
            
            //std::cout << "[DECODE_THUMB] Exit: Conditional branch completed" << std::endl;
            return;
        }
    }
    
    // Format 19-20: Branch and long branch with link (0xE000-0xFFFF)
    else if ((opcode & 0xE000) == 0xE000) {  // Branch and BL
        // First half of BL (0xF000-0xF7FF)
        if ((opcode & 0xF800) == 0xF000) {
            // Save high offset in LR (sign-extended)
            int32_t offset_high = (opcode & 0x7FF) << 12;
            if (offset_high & (1 << 22)) offset_high |= 0xFF800000;
            uint32_t lr_val = pc + 2 + offset_high;
            setRegister(LR, lr_val);
            
            /*std::cout << "[BL FIRST HALF] PC=0x" << std::hex << pc 
                      << " Opcode=0x" << std::setw(4) << std::setfill('0') << opcode
                      << " offset_high=0x" << offset_high 
                      << " LR_set_to=0x" << lr_val 
                      << " next_PC_should_be=0x" << (pc + 2) << std::dec << std::endl;
                      
            std::cout << "[DECODE_THUMB] Exit: BL first half completed" << std::endl;*/
            return;
        }
        // Second half of BL (0xF800-0xFFFF)
        else if ((opcode & 0xF800) == 0xF800) {
            uint32_t oldLR = getRegister(LR);
            int32_t offset_low = (opcode & 0x7FF) << 1;
            uint32_t target = oldLR + offset_low;
            uint32_t ret_addr = (pc + 2) | 1;
            setRegister(LR, ret_addr);
            
            /*std::cout << "[BL SECOND HALF] PC=0x" << std::hex << pc
                      << " Opcode=0x" << std::setw(4) << std::setfill('0') << opcode
                      << " oldLR=0x" << oldLR 
                      << " offset_low=0x" << offset_low
                      << " target=0x" << target 
                      << " new_LR=0x" << ret_addr << std::dec << std::endl;*/
            
            // Check if this looks like a valid BL sequence
            if (oldLR == 0) {
                std::cerr << "[BL ERROR] LR was zero! Previous BL first half was likely skipped or not executed!" << std::endl;
                dumpROMArea(pc, 32);  // Dump ROM to see what's actually there
            }
            
            pc = target & ~1;
            
            //std::cout << "[DECODE_THUMB] Exit: BL second half completed" << std::endl;
            return;
        }
        // Unconditional branch (0xE000-0xEFFF)
        else {
            int32_t offset = ((int32_t)((opcode & 0x7FF) << 21)) >> 20;
            uint32_t target = pc + 2 + offset;
            
            /*std::cout << "[THUMB BRANCH] PC=0x" << std::hex << pc 
                      << " Opcode=0x" << std::setw(4) << std::setfill('0') << opcode
                      << " offset=0x" << offset 
                      << " target=0x" << target << std::dec << std::endl;*/
                      
            pc = target;
            
            //std::cout << "[DECODE_THUMB] Exit: Unconditional branch completed" << std::endl;
            return;
        }
    }
   
    else {
        std::cerr << "[cpu.cpp:" << __LINE__ << "]: Unimplemented Thumb Opcode 0x" 
                  << std::hex << opcode << " at PC=0x" << (pc - 2) << std::dec << std::endl;
    }
    
    std::cout << "[DECODE_THUMB] Exit: Unhandled instruction" << std::endl;
}

// --- Helper Implementations ---
void CPU::branch(uint32_t targetAddress, bool link) {
    if (link) {
        setRegister(LR, getRegister(PC));
    }
    setRegister(PC, targetAddress);
}

void CPU::updateNZFlags(uint32_t result) {
    cpsr &= ~(N_FLAG | Z_FLAG);
    if (result == 0) cpsr |= Z_FLAG;
    if (result & N_FLAG) cpsr |= N_FLAG;
}
void CPU::updateCVFlagsForAdd(uint32_t op1, uint32_t op2, uint32_t result) {
    cpsr &= ~(C_FLAG | V_FLAG);
    if ((uint64_t)op1 + op2 > 0xFFFFFFFF) cpsr |= C_FLAG;
    if (~(op1 ^ op2) & (op1 ^ result) & N_FLAG) cpsr |= V_FLAG;
}
void CPU::updateCVFlagsForSub(uint32_t op1, uint32_t op2, uint32_t result) {
    cpsr &= ~(C_FLAG | V_FLAG);
    if (op1 >= op2) cpsr |= C_FLAG;
    if ((op1 ^ op2) & (op1 ^ result) & N_FLAG) cpsr |= V_FLAG;
}
bool CPU::checkCondition(uint32_t cond) const {
    bool n = cpsr & N_FLAG, z = cpsr & Z_FLAG, c = cpsr & C_FLAG, v = cpsr & V_FLAG;
    // Detailed flag debugging
    //std::cout << "[FLAG_DEBUG] CPSR=0x" << std::hex << cpsr << std::dec
    //          << " N=" << n << " Z=" << z << " C=" << c << " V=" << v << std::endl;
    bool result;
    switch (cond) {
        case 0x0: result = z; break;
        case 0x1: result = !z; break;
        case 0x2: result = c; break;
        case 0x3: result = !c; break;  // BCC - should be false when c=true
        case 0x4: result = n; break;
        case 0x5: result = !n; break;
        case 0x6: result = v; break;
        case 0x7: result = !v; break;
        case 0x8: result = c && !z; break;
        case 0x9: result = !c || z; break;
        case 0xA: result = n == v; break;
        case 0xB: result = n != v; break;
        case 0xC: result = !z && (n == v); break;
        case 0xD: result = z || (n != v); break;
        case 0xE: result = true; break;
        default: result = false; break;
    }
    if (cond == 0x3) {
        std::cout << "[BCC_DEBUG] cond=0x3 c=" << c << " !c=" << !c << " result=" << result << std::endl;
    }
    
    return result;
}

uint8_t CPU::readMemory8(uint32_t address) { return memory.read8(address); }
uint16_t CPU::readMemory16(uint32_t address) { return memory.read16(address); }
uint32_t CPU::readMemory32(uint32_t address) { return memory.read32(address); }
void CPU::writeMemory8(uint32_t address, uint8_t value) { memory.write8(address, value); }
void CPU::writeMemory16(uint32_t address, uint16_t value) { memory.write16(address, value); }
void CPU::writeMemory32(uint32_t address, uint32_t value) { memory.write32(address, value); }
uint32_t CPU::fetchArmOpcode() { return readMemory32(pc); }
uint16_t CPU::fetchThumbOpcode() { return readMemory16(pc); }
