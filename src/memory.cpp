#include "memory.h"

Memory::Memory()
    : bios(16 * 1024), workRAM(256 * 1024), ioRegisters(1024), rom(32 * 1024 * 1024) {}

uint8_t Memory::read(uint32_t address) {
    if (address < 0x4000) { // BIOS
        return bios[address];
    } else if (address >= 0x08000000 && address < 0x0A000000) { // ROM region
        return rom[address - 0x08000000];
    } else if (address >= 0x02000000 && address < 0x02040000) { // Work RAM
        return workRAM[address - 0x02000000];
    } else if (address >= 0x04000000 && address < 0x04000400) { // I/O Registers
        return ioRegisters[address - 0x04000000];
    }
    // For now, return 0 if address is invalid
    return 0;
}

void Memory::write(uint32_t address, uint8_t value) {
    if (address >= 0x02000000 && address < 0x02040000) { // Work RAM
        workRAM[address - 0x02000000] = value;
    } else if (address >= 0x04000000 && address < 0x04000400) { // I/O Registers
        ioRegisters[address - 0x04000000] = value;
    } else if (address >= 0x08000000 && address < 0x0A000000) { // ROM region
        rom[address - 0x08000000] = value;
    }
    // Ignore writes to BIOS or invalid addresses
}
