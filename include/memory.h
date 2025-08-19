// memory.h
#ifndef MEMORY_H
#define MEMORY_H

#include <cstdint>
#include <vector>
#include <stdexcept>

// Memory-mapped IO registers
constexpr uint32_t REG_DISPCNT  = 0x04000000;  // Display control
constexpr uint32_t REG_VCOUNT   = 0x04000006;  // Vertical counter
constexpr uint32_t REG_IE       = 0x04000200;  // Interrupt Enable
constexpr uint32_t REG_IF       = 0x04000202;  // Interrupt Flags
constexpr uint32_t REG_IME      = 0x04000208;  // Interrupt Master Enable


// Forward declaration instead of including cpu.h
class CPU;

class Memory {
public:
    Memory();
    uint8_t read8(uint32_t address);
    uint16_t read16(uint32_t address);
    uint32_t read32(uint32_t address);
    void write8(uint32_t address, uint8_t value);
    void write16(uint32_t address, uint16_t value);
    void write32(uint32_t address, uint32_t value);
    void loadRom(const std::vector<uint8_t>& romData);
    void debugLogVRAMWrites(bool enable) { debugVRAM = enable; }
    void debugLogDISPCNTWrites(bool enable) { debugDISPCNT = enable; }
    void debugLogVRAM(bool enable) { debugVRAM = enable; }
    void setIME(bool value);
    bool getIME() const;
    void setDebugEnabled(bool enable) { debugEnabled = enable; }
    
private:
    std::vector<uint8_t> bios;
    std::vector<uint8_t> wramOnboard;
    std::vector<uint8_t> wramOnChip;
    std::vector<uint8_t> ioRegisters;
    std::vector<uint8_t> paletteRam;
    std::vector<uint8_t> vram;
    std::vector<uint8_t> oam;
    std::vector<uint8_t> rom;
    bool debugDISPCNT = false;
    bool debugVRAM = false;
    bool debugEnabled = false;
    bool ime = false;
    
    bool isValidWriteAddress(uint32_t address) const;
};

#endif // MEMORY_H
