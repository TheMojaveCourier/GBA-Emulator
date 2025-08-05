#ifndef MEMORY_H
#define MEMORY_H

#include <cstdint>
#include <vector>

class Memory {
public:
    Memory();
    uint8_t read(uint32_t address);
    void write(uint32_t address, uint8_t value);
    uint32_t getSize() const;
    
    // Enhanced read/write methods for different data sizes
    uint16_t read16(uint32_t address);
    uint32_t read32(uint32_t address);
    void write16(uint32_t address, uint16_t value);
    void write32(uint32_t address, uint32_t value);
    
    // Memory region info
    bool isValidAddress(uint32_t address) const;
    bool isWritableAddress(uint32_t address) const;
   
private:
    std::vector<uint8_t> bios;        // 16 KB BIOS (0x00000000-0x00003FFF)
    std::vector<uint8_t> workRAM;     // 256 KB Work RAM (0x02000000-0x0203FFFF)
    std::vector<uint8_t> chipRAM;     // 32 KB Chip RAM (0x03000000-0x03007FFF)
    std::vector<uint8_t> ioRegisters; // I/O registers (0x04000000-0x040003FF)
    std::vector<uint8_t> palette;     // 1 KB Palette RAM (0x05000000-0x050003FF)
    std::vector<uint8_t> vram;        // 96 KB Video RAM (0x06000000-0x06017FFF)
    std::vector<uint8_t> oam;         // 1 KB Object Attribute Memory (0x07000000-0x070003FF)
    std::vector<uint8_t> rom;         // Cartridge ROM (0x08000000-0x0DFFFFFF, mirrored)
    std::vector<uint8_t> sram;        // 64 KB SRAM (0x0E000000-0x0E00FFFF)
};

#endif
