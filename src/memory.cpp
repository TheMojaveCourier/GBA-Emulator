#include "memory.h"
#include <iostream>

Memory::Memory()
    : bios(16 * 1024),           // 16 KB BIOS
      workRAM(256 * 1024),       // 256 KB Work RAM  
      chipRAM(32 * 1024),        // 32 KB Chip RAM
      ioRegisters(1024),         // 1 KB I/O registers
      palette(1024),             // 1 KB Palette RAM
      vram(96 * 1024),           // 96 KB Video RAM
      oam(1024),                 // 1 KB Object Attribute Memory
      rom(32 * 1024 * 1024),     // 32 MB ROM space
      sram(64 * 1024)            // 64 KB SRAM
{}

uint8_t Memory::read(uint32_t address) {
    // BIOS region (0x00000000-0x00003FFF)
    if (address < 0x4000) {
        return bios[address];
    }
    // Work RAM (0x02000000-0x0203FFFF)
    else if (address >= 0x02000000 && address < 0x02040000) {
        return workRAM[address - 0x02000000];
    }
    // Chip RAM (0x03000000-0x03007FFF)  
    else if (address >= 0x03000000 && address < 0x03008000) {
        return chipRAM[address - 0x03000000];
    }
    // I/O Registers (0x04000000-0x040003FF)
    else if (address >= 0x04000000 && address < 0x04000400) {
        return ioRegisters[address - 0x04000000];
    }
    // Palette RAM (0x05000000-0x050003FF)
    else if (address >= 0x05000000 && address < 0x05000400) {
        return palette[address - 0x05000000];
    }
    // Video RAM (0x06000000-0x06017FFF)
    else if (address >= 0x06000000 && address < 0x06018000) {
        return vram[address - 0x06000000];
    }
    // Object Attribute Memory (0x07000000-0x070003FF)
    else if (address >= 0x07000000 && address < 0x07000400) {
        return oam[address - 0x07000000];
    }
    // ROM region with mirroring (0x08000000-0x0DFFFFFF)
    else if (address >= 0x08000000 && address < 0x0E000000) {
        uint32_t rom_address = (address - 0x08000000) % rom.size();
        return rom[rom_address];
    }
    // SRAM region (0x0E000000-0x0E00FFFF)
    else if (address >= 0x0E000000 && address < 0x0E010000) {
        return sram[address - 0x0E000000];
    }
    
    // Invalid address
    std::cerr << "Invalid memory read at address: 0x" << std::hex << address << std::endl;
    return 0;
}

void Memory::write(uint32_t address, uint8_t value) {
    // Work RAM (0x02000000-0x0203FFFF)
    if (address >= 0x02000000 && address < 0x02040000) {
        workRAM[address - 0x02000000] = value;
    }
    // Chip RAM (0x03000000-0x03007FFF)
    else if (address >= 0x03000000 && address < 0x03008000) {
        chipRAM[address - 0x03000000] = value;
    }
    // I/O Registers (0x04000000-0x040003FF)
    else if (address >= 0x04000000 && address < 0x04000400) {
        ioRegisters[address - 0x04000000] = value;
    }
    // Palette RAM (0x05000000-0x050003FF)
    else if (address >= 0x05000000 && address < 0x05000400) {
        palette[address - 0x05000000] = value;
    }
    // Video RAM (0x06000000-0x06017FFF)
    else if (address >= 0x06000000 && address < 0x06018000) {
        vram[address - 0x06000000] = value;
    }
    // Object Attribute Memory (0x07000000-0x070003FF)
    else if (address >= 0x07000000 && address < 0x07000400) {
        oam[address - 0x07000000] = value;
    }
    // ROM region (usually read-only, but allow for development)
    else if (address >= 0x08000000 && address < 0x0E000000) {
        uint32_t rom_address = (address - 0x08000000) % rom.size();
        rom[rom_address] = value;
    }
    // SRAM region (0x0E000000-0x0E00FFFF)
    else if (address >= 0x0E000000 && address < 0x0E010000) {
        sram[address - 0x0E000000] = value;
    }
    else {
        // Ignore writes to BIOS or invalid addresses
        std::cerr << "Invalid memory write at address: 0x" << std::hex << address << std::endl;
    }
}

uint32_t Memory::getSize() const {
    return 0x10000000; // 256MB address space for GBA
}

uint16_t Memory::read16(uint32_t address) {
    return read(address) | (read(address + 1) << 8);
}

uint32_t Memory::read32(uint32_t address) {
    return read(address) | 
           (read(address + 1) << 8) |
           (read(address + 2) << 16) |
           (read(address + 3) << 24);
}

void Memory::write16(uint32_t address, uint16_t value) {
    write(address, value & 0xFF);
    write(address + 1, (value >> 8) & 0xFF);
}

void Memory::write32(uint32_t address, uint32_t value) {
    write(address, value & 0xFF);
    write(address + 1, (value >> 8) & 0xFF);
    write(address + 2, (value >> 16) & 0xFF);
    write(address + 3, (value >> 24) & 0xFF);
}

bool Memory::isValidAddress(uint32_t address) const {
    return (address < 0x4000) ||                           // BIOS
           (address >= 0x02000000 && address < 0x02040000) || // Work RAM
           (address >= 0x03000000 && address < 0x03008000) || // Chip RAM
           (address >= 0x04000000 && address < 0x04000400) || // I/O Registers
           (address >= 0x05000000 && address < 0x05000400) || // Palette RAM
           (address >= 0x06000000 && address < 0x06018000) || // Video RAM
           (address >= 0x07000000 && address < 0x07000400) || // OAM
           (address >= 0x08000000 && address < 0x0E000000) || // ROM (mirrored)
           (address >= 0x0E000000 && address < 0x0E010000);   // SRAM
}

bool Memory::isWritableAddress(uint32_t address) const {
    return (address >= 0x02000000 && address < 0x02040000) || // Work RAM
           (address >= 0x03000000 && address < 0x03008000) || // Chip RAM
           (address >= 0x04000000 && address < 0x04000400) || // I/O Registers
           (address >= 0x05000000 && address < 0x05000400) || // Palette RAM
           (address >= 0x06000000 && address < 0x06018000) || // Video RAM
           (address >= 0x07000000 && address < 0x07000400) || // OAM
           (address >= 0x08000000 && address < 0x0E000000) || // ROM (for development)
           (address >= 0x0E000000 && address < 0x0E010000);   // SRAM
}
