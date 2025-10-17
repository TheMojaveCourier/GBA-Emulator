#include "memory.h"
#include "debug.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <iomanip>

Memory::Memory()
    : bios(16 * 1024, 0xFF),
      wramOnboard(256 * 1024),  // Fixed size (256KB)
      wramOnChip(32 * 1024),    // Fixed size (32KB)
      ioRegisters(1 * 1024),
      paletteRam(1 * 1024),
      vram(96 * 1024),
      oam(1 * 1024),
      rom() {}

bool Memory::isValidWriteAddress(uint32_t address) const {
    address &= 0x0FFFFFFF;  // Handle mirroring

    // ROM region - not writable
    if (address >= 0x08000000 && address < 0x0E000000) {
        return false;
    }
    
    // I/O registers (0x04000000-0x04000400)
    if (address >= 0x04000000 && address < 0x04000400) {
        return true;  // Allow I/O register writes
    }

    // Valid writable regions
    return (address >= 0x02000000 && address < 0x03000000) ||  // WRAM on-board
           (address >= 0x03000000 && address < 0x04000000) ||  // WRAM on-chip
           (address >= 0x05000000 && address < 0x06000000) ||  // Palette RAM
           (address >= 0x06000000 && address < 0x07000000) ||  // VRAM
           (address >= 0x07000000 && address < 0x08000000);    // OAM
}

uint8_t Memory::read8(uint32_t address) {
    address &= 0x0FFFFFFF;  // Handle mirroring
    
    if (address < 0x00004000) {
        return bios[address % bios.size()];
    }
    else if (address >= 0x02000000 && address < 0x03000000) {
        return wramOnboard[address & 0x3FFFF];
    }
    else if (address >= 0x03000000 && address < 0x04000000) {
        return wramOnChip[address & 0x7FFF];
    }
    else if (address >= 0x04000000 && address < 0x05000000) {
        return ioRegisters[address & 0x3FF];
    }
    else if (address >= 0x05000000 && address < 0x05000400) {
        return paletteRam[address & 0x3FF];
    }
    else if (address >= 0x06000000 && address < 0x06018000) {
        return vram[address & 0x17FFF];
    }
    else if (address >= 0x07000000 && address < 0x07000400) {
        return oam[address & 0x3FF];
    }
    else if (address >= 0x08000000 && address < 0x0E000000) {
        uint32_t offset = address - 0x08000000;
        return offset < rom.size() ? rom[offset] : 0;
    }
    return 0;
}

uint16_t Memory::read16(uint32_t address) {
    if (address & 1) {
        std::cerr << "[memory.cpp:" << __LINE__ << "]: Unaligned 16-bit read at 0x" 
                  << std::hex << address << std::dec << std::endl;
    }

    address &= 0x0FFFFFFF;
    uint32_t aligned_addr = address & ~1;

    // ROM region
    if (aligned_addr >= 0x08000000 && aligned_addr < 0x0E000000) {
        uint32_t offset = aligned_addr - 0x08000000;
        if (offset + 1 < rom.size()) {
            return rom[offset] | (rom[offset + 1] << 8);
        }
    }
    // WRAM regions
    else if (aligned_addr >= 0x02000000 && aligned_addr < 0x03000000) {
        uint32_t offset = aligned_addr & 0x3FFFF;
        return wramOnboard[offset] | (wramOnboard[offset + 1] << 8);
    }
    else if (aligned_addr >= 0x03000000 && aligned_addr < 0x04000000) {
        uint32_t offset = aligned_addr & 0x7FFF;
        return wramOnChip[offset] | (wramOnChip[offset + 1] << 8);
    }
    // I/O registers
    else if (aligned_addr >= 0x04000000 && aligned_addr < 0x04000400) {
        uint32_t offset = aligned_addr & 0x3FF;
        return ioRegisters[offset] | (ioRegisters[offset + 1] << 8);
    }
    // VRAM
    else if (aligned_addr >= 0x06000000 && aligned_addr < 0x06018000) {
        uint32_t offset = aligned_addr & 0x1FFFF;
        return vram[offset] | (vram[offset + 1] << 8);
    }

    // Default to read8 for other regions
    return read8(aligned_addr) | (read8(aligned_addr + 1) << 8);
}

uint32_t Memory::read32(uint32_t address) {
    // Force alignment to 4 bytes (clear bottom 2 bits)
    uint32_t aligned_address = address & ~0x3;
    
    // Mask to valid GBA address range (28 bits)
    aligned_address &= 0x0FFFFFFF;
    
    // ROM region (0x08000000-0x0E000000)
    if (aligned_address >= 0x08000000 && aligned_address < 0x0E000000) {
        uint32_t offset = aligned_address - 0x08000000;
        if (offset + 3 < rom.size()) {
            return rom[offset] |
                   (rom[offset + 1] << 8) |
                   (rom[offset + 2] << 16) |
                   (rom[offset + 3] << 24);
        }
        // Out of bounds ROM access - real hardware behavior depends on cartridge
        return 0;
    }
    // On-chip WRAM (0x03000000-0x03008000)
    else if (aligned_address >= 0x03000000 && aligned_address < 0x03008000) {
        uint32_t offset = aligned_address & 0x7FFF;
        return wramOnChip[offset] |
               (wramOnChip[offset + 1] << 8) |
               (wramOnChip[offset + 2] << 16) |
               (wramOnChip[offset + 3] << 24);
    }
    // On-board WRAM (0x02000000-0x02040000)
    else if (aligned_address >= 0x02000000 && aligned_address < 0x02040000) {
        uint32_t offset = aligned_address & 0x3FFFF;
        return wramOnboard[offset] |
               (wramOnboard[offset + 1] << 8) |
               (wramOnboard[offset + 2] << 16) |
               (wramOnboard[offset + 3] << 24);
    }
    // BIOS region (0x00000000-0x00004000)
    else if (aligned_address < 0x4000) {
        if (aligned_address + 3 < bios.size()) {
            return bios[aligned_address] |
                   (bios[aligned_address + 1] << 8) |
                   (bios[aligned_address + 2] << 16) |
                   (bios[aligned_address + 3] << 24);
        }
        else {
            return 0xE3A02004;  // Default BIOS instruction (MOV R2, #4)
        }
    }
    // I/O registers (0x04000000-0x04000400)
    else if (aligned_address >= 0x04000000 && aligned_address < 0x04000400) {
        uint32_t offset = aligned_address & 0x3FF;
        return ioRegisters[offset] |
               (ioRegisters[offset + 1] << 8) |
               (ioRegisters[offset + 2] << 16) |
               (ioRegisters[offset + 3] << 24);
    }
    // Palette RAM (0x05000000-0x05000400)
    else if (aligned_address >= 0x05000000 && aligned_address < 0x05000400) {
        uint32_t offset = aligned_address & 0x3FF;
        return paletteRam[offset] |
               (paletteRam[offset + 1] << 8) |
               (paletteRam[offset + 2] << 16) |
               (paletteRam[offset + 3] << 24);
    }
    // VRAM (0x06000000-0x07000000)
    else if (aligned_address >= 0x06000000 && aligned_address < 0x07000000) {
        uint32_t offset = aligned_address & 0x1FFFF; // VRAM is 128KB with mirroring
        return vram[offset] |
               (vram[offset + 1] << 8) |
               (vram[offset + 2] << 16) |
               (vram[offset + 3] << 24);
    }
    // OAM (0x07000000-0x07000400)
    else if (aligned_address >= 0x07000000 && aligned_address < 0x07000400) {
        uint32_t offset = aligned_address & 0x3FF;
        return oam[offset] |
               (oam[offset + 1] << 8) |
               (oam[offset + 2] << 16) |
               (oam[offset + 3] << 24);
    }

    // For debugging only - real GBA returns 0
    if (debugEnabled) {
        std::cerr << "[memory.cpp:" << __LINE__ << "]: Invalid memory read32 at 0x" 
                  << std::hex << address << std::dec << std::endl;
    }
    
    // Real GBA hardware returns 0 for undefined memory
    return 0x00000000;
}

void Memory::write8(uint32_t address, uint8_t value) {
    address &= 0x0FFFFFFF;
    
    if (!isValidWriteAddress(address)) {
        // Use debugger instead of direct console output
        if (debugger) {
            debugger->traceMemoryAccess(address, value, true, 1);
        }
        return;
    }
    
    // Prevent writes to ROM region
    if (address >= 0x08000000 && address < 0x0E000000) {
        if (debugger) {
            debugger->traceMemoryAccess(address, value, true, 1);
        }
        return;
    }

    // Special handling for 8-bit writes to VRAM
    if (address >= 0x06000000 && address < 0x07000000) {
        // GBA hardware behavior: 8-bit writes to VRAM get converted to 16-bit
        uint32_t aligned_addr = address & ~1; // Align to 16-bit boundary
        uint16_t existing = read16(aligned_addr);
        uint16_t new_value;
        
        if (address & 1) {
            // High byte
            new_value = (existing & 0x00FF) | (value << 8);
        } else {
            // Low byte
            new_value = (existing & 0xFF00) | value;
        }
        
        // Use debugger interface instead of direct console output
        if (debugger && debugVRAM) {
            std::stringstream ss;
            ss << "8-bit write converted: 0x" << std::hex << address 
               << "=" << (int)value << " -> 16-bit: 0x" << aligned_addr 
               << "=" << new_value << std::dec;
            debugger->traceVRAMWrite(aligned_addr, new_value);
        }
        
        write16(aligned_addr, new_value);
        return;
    }
    
    // Memory region handling
    if (address >= 0x02000000 && address < 0x03000000) {
        // WRAM on-board
        wramOnboard[address & 0x3FFFF] = value;
    }
    else if (address >= 0x03000000 && address < 0x04000000) {
        // WRAM on-chip
        wramOnChip[address & 0x7FFF] = value;
    }
    else if (address >= 0x04000000 && address < 0x05000000) {
        // IO registers
        uint32_t offset = address & 0x3FF;
        
        // Special handling for DISPCNT
        if (offset < 0x10 && debugger && debugDISPCNT) {
            std::stringstream ss;
            ss << "DISPCNT write [" << (offset) << "] = 0x" << std::hex << (int)value;
            debugger->traceMemoryAccess(address, value, true, 1);
        }
        
        ioRegisters[offset] = value;
    }
    else if (address >= 0x05000000 && address < 0x06000000) {
        // Palette RAM
        paletteRam[address & 0x3FF] = value;
    }
    else if (address >= 0x07000000 && address < 0x08000000) {
        // OAM
        oam[address & 0x3FF] = value;
    }
    
    // Trace memory access through debugger
    if (debugger) {
        debugger->traceMemoryAccess(address, value, true, 1);
    }
}

void Memory::write16(uint32_t address, uint16_t value) {
    // GBA hardware automatically aligns halfword accesses by clearing the lowest bit
    uint32_t aligned_address = address & ~1;  // Force alignment
    
    if (!isValidWriteAddress(aligned_address)) {
        if (debugEnabled) {  // Only show this message when debugging is enabled
            std::cerr << "[memory.cpp:" << __LINE__ << "]: Invalid write16 address: 0x" 
                      << std::hex << aligned_address << std::dec << std::endl;
        }
        return;
    }
    
    aligned_address &= 0x0FFFFFFF;  // Handle mirroring
    
    // Prevent writes to ROM region
    if (aligned_address >= 0x08000000 && aligned_address < 0x0E000000) {
        if (debugEnabled) {  // Only show this message when debugging is enabled
            std::cerr << "[memory.cpp:" << __LINE__ << "]: Attempted write to ROM at 0x" 
                      << std::hex << aligned_address << std::dec << std::endl;
        }
        return;
    }
    
    // Use public trace methods instead of private log method
    // Special debug for key memory regions
    if (aligned_address >= 0x05000000 && aligned_address < 0x05000400) {
        if (debugger) {
            debugger->tracePaletteWrite(aligned_address, value);  // Use our new method
            debugger->traceMemoryAccess(aligned_address, value, true, 2);
        }
        
        // Only log to console if debug is enabled, preferably remove once debugger is working
        if (debugEnabled) {
            std::cout << "PALETTE WRITE: 0x" << std::hex << aligned_address 
                      << " = 0x" << value << std::dec << std::endl;
        }
    }
    else if (aligned_address >= 0x06000000 && aligned_address < 0x06018000) {
        if (debugger) {
            debugger->traceVRAMWrite(aligned_address, value);
            debugger->traceMemoryAccess(aligned_address, value, true, 2);
        }
    }
    
    // Debug display registers 
    if (aligned_address == REG_DISPCNT && debugger) {
        debugger->traceDisplayControl(value);
    }
    else if ((aligned_address >= REG_BG0CNT && aligned_address <= REG_BG3CNT) && debugger) {
        int bg_index = (aligned_address - REG_BG0CNT) / 2;
        debugger->traceBGControl(bg_index, value);
    }
    
    // Special case for I/O register writes that need different handling
    if (aligned_address >= 0x04000000 && aligned_address < 0x04000400) {
        // Handle I/O registers
        uint32_t offset = aligned_address & 0x3FF;
        
        // Special handling for timer registers
        if (aligned_address >= 0x04000100 && aligned_address <= 0x0400010E) {
            uint8_t timer_id = (aligned_address - 0x04000100) / 4;
            
            // Handle timer control register (TM0CNT_H, TM1CNT_H, etc.)
            if ((aligned_address & 2) == 2) {  // TIMxCNT_H (control)
                if (value & 0x80) {  // Timer enabled
                    // Store current timer value
                    uint16_t reload = timer_reload[timer_id];
                    
                    // Store the control value
                    ioRegisters[offset] = value & 0xFF;
                    ioRegisters[offset + 1] = (value >> 8) & 0xFF;
                    
                    // Reset timer with reload value
                    timer_value[timer_id] = reload;
                    timer_running[timer_id] = true;
                } else {
                    // Timer disabled
                    timer_running[timer_id] = false;
                    ioRegisters[offset] = value & 0xFF;
                    ioRegisters[offset + 1] = (value >> 8) & 0xFF;
                }
            } else {  // TIMxCNT_L (reload value)
                // Store reload value
                timer_reload[timer_id] = value;
                ioRegisters[offset] = value & 0xFF;
                ioRegisters[offset + 1] = (value >> 8) & 0xFF;
                
                // If timer is running, immediately update the current value
                if (timer_running[timer_id]) {
                    timer_value[timer_id] = value;
                }
            }
            
            return;
        }
        
        // Handle DMA registers
        else if (aligned_address >= 0x040000B0 && aligned_address <= 0x040000DF) {
            int dma_id = (aligned_address - 0x040000B0) / 12;
            int reg_offset = (aligned_address - 0x040000B0) % 12;
            
            ioRegisters[offset] = value & 0xFF;
            ioRegisters[offset + 1] = (value >> 8) & 0xFF;
            
            // Check if this is writing to DMAxCNT_H (DMA control) and DMA is being enabled
            if (reg_offset == 10 && (value & 0x8000)) {
                executeDMA(dma_id);
            }
            
            return;
        }
        
        // Special case for DISPCNT register
        else if (aligned_address == REG_DISPCNT) {
            // Only output when debug is enabled
            if (debugEnabled) {
                std::cout << "[DISPLAY] Setting DISPCNT = 0x" << std::hex << value 
                          << " (Mode " << (value & 0x7) 
                          << ", BG0: " << ((value & 0x0100) ? "ON" : "off")
                          << ", BG1: " << ((value & 0x0200) ? "ON" : "off")
                          << ", BG2: " << ((value & 0x0400) ? "ON" : "off")
                          << ", BG3: " << ((value & 0x0800) ? "ON" : "off")
                          << ", OBJ: " << ((value & 0x1000) ? "ON" : "off")
                          << ", Page: " << ((value & 0x0010) ? "1" : "0")
                          << ")" << std::dec << std::endl;
            }
            
            // Store the value in the IO registers
            ioRegisters[offset] = value & 0xFF;
            ioRegisters[offset + 1] = (value >> 8) & 0xFF;
            
            return;
        }
        
        // Special handling for read-only registers (IF, etc.)
        else if (aligned_address == REG_IF) {
            // IF register is write-0-to-acknowledge
            // When a bit is written as 0, the corresponding IF bit is cleared
            // Writing 1 has no effect
            uint16_t current = (ioRegisters[offset] | (ioRegisters[offset + 1] << 8));
            uint16_t result = current & ~value;  // Clear bits that are 0 in the value
            
            ioRegisters[offset] = result & 0xFF;
            ioRegisters[offset + 1] = (result >> 8) & 0xFF;
            return;
        }
        
        // Default handling for other I/O registers
        ioRegisters[offset] = value & 0xFF;
        ioRegisters[offset + 1] = (value >> 8) & 0xFF;
        
        return;
    }
    
    // Handle memory regions
    if (aligned_address >= 0x02000000 && aligned_address < 0x03000000) {
        // WRAM on-board
        uint32_t offset = aligned_address & 0x3FFFF;
        wramOnboard[offset] = value & 0xFF;
        wramOnboard[offset + 1] = (value >> 8) & 0xFF;
    }
    else if (aligned_address >= 0x03000000 && aligned_address < 0x04000000) {
        // WRAM on-chip
        uint32_t offset = aligned_address & 0x7FFF;
        wramOnChip[offset] = value & 0xFF;
        wramOnChip[offset + 1] = (value >> 8) & 0xFF;
    }
    else if (aligned_address >= 0x05000000 && aligned_address < 0x06000000) {
        // Palette RAM
        uint32_t offset = aligned_address & 0x3FF;
        paletteRam[offset] = value & 0xFF;
        paletteRam[offset + 1] = (value >> 8) & 0xFF;
    }
    else if (aligned_address >= 0x06000000 && aligned_address < 0x07000000) {
        // VRAM
        uint32_t offset = aligned_address & 0x1FFFF;
        
        if (debugEnabled) {
            static bool vram_write_detected = false;
            if (!vram_write_detected) {
                std::cout << "[VRAM] First VRAM write detected @ 0x" 
                          << std::hex << aligned_address << " = 0x" << value << std::dec << std::endl;
                vram_write_detected = true;
            }
            
            if (debugVRAM && (aligned_address & 0xFFF) == 0) {
                std::cout << "[VRAM] write16 @ 0x" 
                          << std::hex << aligned_address << " = 0x" << value << std::dec << std::endl;
            }
        }
        
        vram[offset] = value & 0xFF;
        vram[offset + 1] = (value >> 8) & 0xFF;
    }
    else if (aligned_address >= 0x07000000 && aligned_address < 0x08000000) {
        // OAM
        uint32_t offset = aligned_address & 0x3FF;
        oam[offset] = value & 0xFF;
        oam[offset + 1] = (value >> 8) & 0xFF;
    }
}

void Memory::write32(uint32_t address, uint32_t value) {
    // Address masking for mirroring
    address &= 0x0FFFFFFF;
    
    if (!isValidWriteAddress(address)) {
        // Use traceMemoryAccess instead of log
        if (debugger) {
            debugger->traceMemoryAccess(address, value, true, 4);
        }
        return;
    }
    
    // Handle unaligned writes - GBA rotates data
    uint32_t aligned_address = address & ~3;
    uint32_t rotated_value = value;
    if (address & 3) {
        int shift = (address & 3) * 8;
        rotated_value = (value >> shift) | (value << (32 - shift));
        
        // Use traceMemoryAccess for debugging
        if (debugger) {
            debugger->traceMemoryAccess(address, rotated_value, true, 4);
        }
    }
    
    // Special case for I/O registers
    if (aligned_address >= 0x04000000 && aligned_address < 0x04000400) {
        // Use traceMemoryAccess for IO register access
        if (debugger) {
            debugger->traceMemoryAccess(aligned_address, rotated_value, true, 4);
        }
        
        // Handle special registers
        if (aligned_address == REG_IME) {
            ime = (rotated_value & 1) != 0;
            
            // Use existing traceMemoryAccess method
            if (debugger) {
                debugger->traceMemoryAccess(REG_IME, rotated_value, true, 4);
            }
        }
        else if (aligned_address == REG_IF) {
            // Interrupt flags - write 1 to clear (GBA hardware behavior)
            uint16_t if_value = (ioRegisters[REG_IF & 0x3FF] | 
                               (ioRegisters[(REG_IF & 0x3FF) + 1] << 8));
            if_value &= ~(rotated_value & 0xFFFF);
            ioRegisters[REG_IF & 0x3FF] = if_value & 0xFF;
            ioRegisters[(REG_IF & 0x3FF) + 1] = (if_value >> 8) & 0xFF;
            
            // Only write the upper bits normally
            ioRegisters[(aligned_address & 0x3FF) + 2] = (rotated_value >> 16) & 0xFF;
            ioRegisters[(aligned_address & 0x3FF) + 3] = (rotated_value >> 24) & 0xFF;
            return;
        }
        
        // Write the value to I/O register memory
        uint32_t offset = aligned_address & 0x3FF;
        ioRegisters[offset] = rotated_value & 0xFF;
        ioRegisters[offset + 1] = (rotated_value >> 8) & 0xFF;
        
        // Use safer array bounds checking
        if (offset + 2 < ioRegisters.size()) {
            ioRegisters[offset + 2] = (rotated_value >> 16) & 0xFF;
        }
        if (offset + 3 < ioRegisters.size()) {
            ioRegisters[offset + 3] = (rotated_value >> 24) & 0xFF;
        }
        return;
    }
    
    // Prevent writes to ROM region
    if (aligned_address >= 0x08000000 && aligned_address < 0x0E000000) {
        if (debugger) {
            // Use traceMemoryAccess with meaningful data
            debugger->traceMemoryAccess(aligned_address, value, true, 4);
        }
        return;
    }
    
    // Handle memory writes by region
    if (aligned_address >= 0x02000000 && aligned_address < 0x03000000) {
        // WRAM - write directly 
        uint32_t offset = aligned_address & 0x3FFFF;
        wramOnboard[offset] = rotated_value & 0xFF;
        wramOnboard[offset + 1] = (rotated_value >> 8) & 0xFF;
        wramOnboard[offset + 2] = (rotated_value >> 16) & 0xFF;
        wramOnboard[offset + 3] = (rotated_value >> 24) & 0xFF;
    }
    else if (aligned_address >= 0x03000000 && aligned_address < 0x04000000) {
        // IWRAM
        uint32_t offset = aligned_address & 0x7FFF;
        wramOnChip[offset] = rotated_value & 0xFF;
        wramOnChip[offset + 1] = (rotated_value >> 8) & 0xFF;
        wramOnChip[offset + 2] = (rotated_value >> 16) & 0xFF;
        wramOnChip[offset + 3] = (rotated_value >> 24) & 0xFF;
    }
    else if (aligned_address >= 0x05000000 && aligned_address < 0x06000000) {
        // Palette
        uint32_t offset = aligned_address & 0x3FF;
        paletteRam[offset] = rotated_value & 0xFF;
        paletteRam[offset + 1] = (rotated_value >> 8) & 0xFF;
        paletteRam[offset + 2] = (rotated_value >> 16) & 0xFF;
        paletteRam[offset + 3] = (rotated_value >> 24) & 0xFF;
    }
    else if (aligned_address >= 0x06000000 && aligned_address < 0x07000000) {
        // VRAM
        uint32_t offset = aligned_address & 0x1FFFF;
        vram[offset] = rotated_value & 0xFF;
        vram[offset + 1] = (rotated_value >> 8) & 0xFF;
        vram[offset + 2] = (rotated_value >> 16) & 0xFF;
        vram[offset + 3] = (rotated_value >> 24) & 0xFF;
    }
    else if (aligned_address >= 0x07000000 && aligned_address < 0x08000000) {
        // OAM
        uint32_t offset = aligned_address & 0x3FF;
        oam[offset] = rotated_value & 0xFF;
        oam[offset + 1] = (rotated_value >> 8) & 0xFF;
        oam[offset + 2] = (rotated_value >> 16) & 0xFF;
        oam[offset + 3] = (rotated_value >> 24) & 0xFF;
    }
    //Cartridge SRAM support (if needed) Add later!!!!
    /*else if (aligned_address >= 0x0E000000 && aligned_address < 0x0E010000) {
        // Cartridge SRAM/Flash - if implemented
        if (sram.size() > 0) {
            uint32_t offset = aligned_address & 0xFFFF;
            if (offset < sram.size()) {
                // SRAM is typically 8-bit, so only write lower byte
                sram[offset] = rotated_value & 0xFF;
            }
        }
    }*/
    else {
            if (debugger) {
                // Use traceMemoryAccess for invalid address
                debugger->traceMemoryAccess(aligned_address, rotated_value, true, 4);
            }
    }
}

void Memory::executeDMA(int dma_id) {
    // Get DMA registers
    uint32_t baseAddr = 0x040000B0 + dma_id * 12;
    uint32_t src_addr = read32(baseAddr);
    uint32_t dst_addr = read32(baseAddr + 4);
    uint16_t control = read16(baseAddr + 8);
    
    // Extract parameters
    int count = control & 0x3FFF;
    if (count == 0) {
        // Count of 0 means:
        // DMA0, DMA1, DMA2: 0x4000 (16384) units
        // DMA3: 0x10000 (65536) units
        count = (dma_id == 3) ? 0x10000 : 0x4000;
    }
    
    bool word_size = (control & 0x400); // 0=16-bit, 1=32-bit
    int dest_adj = (control >> 5) & 0x3; // 00=inc, 01=dec, 10=fixed, 11=inc+reload
    int src_adj = (control >> 7) & 0x3;  // 00=inc, 01=dec, 10=fixed, 11=prohibited
    
    // Adjust addresses based on internal registers if needed
    if (dma_id == 0 && (src_addr >> 24) == 0x08) src_addr &= 0x07FFFFFF;
    if (dma_id == 3 && (src_addr >> 24) == 0x08) src_addr &= 0x07FFFFFF;
    
    // Destination address masks
    dst_addr &= 0x07FFFFFF;
    
    // Perform the transfer
    for (int i = 0; i < count; i++) {
        if (word_size) {
            // 32-bit transfer
            uint32_t data = read32(src_addr);
            write32(dst_addr, data);
            
            // Adjust addresses
            if (src_adj == 0) src_addr += 4;
            else if (src_adj == 1) src_addr -= 4;
            
            if (dest_adj == 0) dst_addr += 4;
            else if (dest_adj == 1) dst_addr -= 4;
        } else {
            // 16-bit transfer
            uint16_t data = read16(src_addr);
            write16(dst_addr, data);
            
            // Adjust addresses
            if (src_adj == 0) src_addr += 2;
            else if (src_adj == 1) src_addr -= 2;
            
            if (dest_adj == 0) dst_addr += 2;
            else if (dest_adj == 1) dst_addr -= 2;
        }
    }
    
    // Handle repeat
    bool repeat = (control & 0x200);
    if (!repeat) {
        // Turn off DMA if not repeating
        write16(baseAddr + 10, control & ~0x8000);
    } else if (dest_adj == 3) {
        // Reload destination address if required
        // The original destination is stored in DMAxD
        uint32_t original_dst = read32(baseAddr + 4);
        write32(baseAddr + 4, original_dst);
    }
    
    // Set IRQ if enabled
    if (control & 0x4000) {
        uint16_t irq_flag = read16(REG_IF);
        write16(REG_IF, irq_flag | (1 << (8 + dma_id)));
    }
}

void Memory::tickTimers(int cycles) {
    static const int prescalers[4] = {1, 64, 256, 1024};
    
    for (int i = 0; i < 4; i++) {
        if (!timer_running[i]) continue;
        
        // Get control register
        uint16_t control = read16(0x04000100 + i * 4 + 2);
        int prescaler_idx = control & 0x3;
        int prescaler = prescalers[prescaler_idx];
        
        // Increment timer
        timer_value[i] += cycles / prescaler;
        
        // Check overflow
        if (timer_value[i] >= 0x10000) {
            // Timer overflow
            timer_value[i] = timer_reload[i] + (timer_value[i] - 0x10000);
            
            // Check if IRQ is enabled
            if (control & 0x40) {
                // Trigger timer interrupt
                uint16_t irq_flag = read16(REG_IF);
                write16(REG_IF, irq_flag | (1 << (3 + i)));
            }
            
            // Check if this timer cascades to the next one
            if (i < 3 && (read16(0x04000100 + (i+1) * 4 + 2) & 0x4)) {
                timer_value[i+1]++;
            }
        }
        
        // Update the timer register with current value
        // Use a direct update to avoid recursive calls
        uint32_t addr = 0x04000100 + i * 4;
        uint32_t offset = addr & 0x3FF;
        ioRegisters[offset] = timer_value[i] & 0xFF;
        ioRegisters[offset + 1] = (timer_value[i] >> 8) & 0xFF;
    }
}

void Memory::setIME(bool value) { 
    ime = value; 
    ioRegisters[0x208 & 0x3FF] = value ? 1 : 0;
}

bool Memory::getIME() const { 
    return ime; 
}

void Memory::loadRom(const std::vector<uint8_t>& romData) {
    if (romData.empty()) {
        throw std::runtime_error("Attempted to load empty ROM data");
    }
    
    rom = romData;
    
    if (debugEnabled){
		std::cout << "ROM loaded. Size: " << rom.size() << " bytes" << std::endl;
		std::cout << "First 32 bytes:";
		for (size_t i = 0; i < 32 && i < rom.size(); i++) {
		    if (i % 8 == 0) std::cout << "\n  ";
		    std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') 
		              << (int)rom[i] << " ";
		}
		std::cout << std::dec << std::endl;
    }
}
