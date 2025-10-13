#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cstdint>

class CPU;
class Memory;

class DebugTracer {
public:
    enum TraceLevel {
        NONE = 0,
        MINIMAL = 1,
        NORMAL = 2,
        VERBOSE = 3,
        FULL = 4
    };
    
    DebugTracer(CPU& cpu, Memory* mem = nullptr, TraceLevel level = NORMAL);
    void setTraceLevel(TraceLevel level);
    void setLogFile(const std::string& filename);
    void setConsoleOutput(bool enable) { outputToConsole = enable; }
    //void setMemory(Memory* mem);
    
    //Memory Dump
    void setMemory(Memory* mem) { memory = mem; }
    void dumpVRAMContents();
    void dumpPaletteContents();
    void dumpTileMap(int screenBlock);
    
    // Tracing functions
    void traceInstruction(uint32_t address, uint32_t opcode, bool isThumb);
    void traceRegisters();
    void traceFlags();
    void traceMemoryAccess(uint32_t address, uint32_t value, bool isWrite, int size);
    void traceCall(uint32_t from, uint32_t to);
    void traceReturn(uint32_t from, uint32_t to);
    void traceInterrupt(int type);
    void traceDisplayControl(uint16_t dispcnt);
    void traceBGControl(int bg_index, uint16_t bgcnt);
    void traceVRAMWrite(uint32_t address, uint16_t value);
    void tracePaletteWrite(uint32_t address, uint16_t value);
    void tracePPUMode4State(uint16_t dispcnt, uint32_t frameCount);
    void traceBitmapData(uint32_t vramAddr, uint32_t length, uint32_t frameOffset);
    
    // Helper utilities
    void dumpMemoryRange(uint32_t start, uint32_t end);
    void dumpRegisters();
    void addBreakpoint(uint32_t address);
    void removeBreakpoint(uint32_t address);
    bool checkBreakpoint(uint32_t address);
    std::string getCurrentDateTime() const;
    
private:
    CPU& cpu;
    Memory* memory;
    TraceLevel traceLevel;
    std::ofstream logFile;
    std::vector<uint32_t> breakpoints;
    
    std::string getRegisterName(int index);
    std::string getFlagString();
    void log(const std::string& message);
    bool outputToConsole = true;  // Default to true for backward compatibility
};

#endif // DEBUG_H
