#ifndef MEMORY_H
#define MEMORY_H

#include <cstdint>
#include <vector>

class Memory {
public:
    Memory();
    uint8_t read(uint32_t address);
    void write(uint32_t address, uint8_t value);
   
private:
    std::vector<uint8_t> bios;     // 16 KB BIOS
    std::vector<uint8_t> workRAM; // 256 KB Work RAM
    std::vector<uint8_t> ioRegisters; // I/O registers (dummy for now)
    std::vector<uint8_t> rom;      // Cartridge ROM (up to 32 MB)
};

#endif
