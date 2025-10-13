#include "debug.h"
#include "cpu.h"
#include <chrono>
#include <iomanip>

DebugTracer::DebugTracer(CPU& cpu, Memory* mem, TraceLevel level)
    : cpu(cpu), memory(mem), traceLevel(level), outputToConsole(false) {
}

void DebugTracer::setTraceLevel(TraceLevel level) {
    traceLevel = level;
}


void DebugTracer::setLogFile(const std::string& filename) {
    if (logFile.is_open()) {
        logFile.close();
    }
    
    logFile.open(filename);
    
    if (!logFile.is_open()) {
        std::cerr << "Error: Could not open debug log file: " << filename << std::endl;
        // Fall back to console output if file can't be opened
        outputToConsole = true;
    } else {
        // Add a header to the log file
        logFile << "=== GBA Emulator Debug Log ===" << std::endl;
        logFile << "Started: " << getCurrentDateTime() << std::endl;
        logFile << "================================" << std::endl;
    }
}

// Helper function to get current date/time
std::string DebugTracer::getCurrentDateTime() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

void DebugTracer::tracePaletteWrite(uint32_t address, uint16_t value) {
    std::stringstream ss;
    ss << "PALETTE WRITE: 0x" << std::hex << address << " = 0x" << value 
       << " RGB=(" << (value & 0x1F) << "," 
       << ((value >> 5) & 0x1F) << "," 
       << ((value >> 10) & 0x1F) << ")";
    log(ss.str());
}

void DebugTracer::traceInstruction(uint32_t address, uint32_t opcode, bool isThumb) {
    if (traceLevel < MINIMAL) return;
    
    std::stringstream ss;
    ss << "[EXEC] " << (isThumb ? "THUMB" : "ARM") << " @ 0x" 
       << std::hex << std::setw(8) << std::setfill('0') << address
       << ": 0x" << std::setw(isThumb ? 4 : 8) << opcode;
    
    log(ss.str());
    
    if (traceLevel >= NORMAL) {
        traceRegisters();
        if (traceLevel >= VERBOSE) {
            traceFlags();
        }
    }
}

void DebugTracer::traceRegisters() {
    std::stringstream ss;
    ss << "[REGS] ";
    
    for (int i = 0; i < 16; i++) {
        if (i > 0 && i % 4 == 0) {
            ss << "\n       ";
        }
        ss << getRegisterName(i) << "=0x" << std::hex << std::setw(8) << std::setfill('0')
           << cpu.getRegister(i) << " ";
    }
    
    log(ss.str());
}

void DebugTracer::traceFlags() {
    std::stringstream ss;
    ss << "[FLAGS] " << getFlagString();
    log(ss.str());
}

void DebugTracer::traceMemoryAccess(uint32_t address, uint32_t value, bool isWrite, int size) {
    if (traceLevel < VERBOSE) return;
    
    std::stringstream ss;
    ss << "[MEM] " << (isWrite ? "Write" : "Read") << size * 8 << " @ 0x"
       << std::hex << std::setw(8) << std::setfill('0') << address
       << " = 0x" << std::setw(size * 2) << value;
    
    log(ss.str());
}

void DebugTracer::traceCall(uint32_t from, uint32_t to) {
    if (traceLevel < NORMAL) return;
    
    std::stringstream ss;
    ss << "[CALL] from 0x" << std::hex << std::setw(8) << std::setfill('0') << from
       << " to 0x" << std::setw(8) << to;
    
    log(ss.str());
}

void DebugTracer::traceReturn(uint32_t from, uint32_t to) {
    if (traceLevel < NORMAL) return;
    
    std::stringstream ss;
    ss << "[RET] from 0x" << std::hex << std::setw(8) << std::setfill('0') << from
       << " to 0x" << std::setw(8) << to;
    
    log(ss.str());
}

void DebugTracer::traceInterrupt(int type) {
    if (traceLevel < MINIMAL) return;
    
    std::stringstream ss;
    ss << "[IRQ] Type " << type;
    log(ss.str());
}

void DebugTracer::dumpMemoryRange(uint32_t start, uint32_t end) {
    std::stringstream ss;
    ss << "[MEM_DUMP] Range 0x" << std::hex << std::setw(8) << std::setfill('0') << start
       << " - 0x" << std::setw(8) << end << ":\n";
    
    const int BYTES_PER_ROW = 16;
    
    for (uint32_t addr = start & ~(BYTES_PER_ROW - 1); addr <= end; addr += BYTES_PER_ROW) {
        ss << std::hex << std::setw(8) << std::setfill('0') << addr << ": ";
        
        for (int i = 0; i < BYTES_PER_ROW; i++) {
            uint32_t curAddr = addr + i;
            if (curAddr >= start && curAddr <= end) {
                // For simplicity, we'll use 8-bit reads here
                uint8_t value = cpu.readMemory8(curAddr);
                ss << std::hex << std::setw(2) << std::setfill('0') << (int)value << " ";
            } else {
                ss << "   ";
            }
            
            if (i % 4 == 3) ss << " ";
        }
        
        ss << " | ";
        
        for (int i = 0; i < BYTES_PER_ROW; i++) {
            uint32_t curAddr = addr + i;
            if (curAddr >= start && curAddr <= end) {
                uint8_t value = cpu.readMemory8(curAddr);
                ss << (value >= 32 && value < 127 ? (char)value : '.');
            } else {
                ss << " ";
            }
        }
        
        ss << "\n";
    }
    
    log(ss.str());
}

void DebugTracer::dumpRegisters() {
    traceRegisters();
    traceFlags();
}

void DebugTracer::addBreakpoint(uint32_t address) {
    breakpoints.push_back(address);
}

void DebugTracer::removeBreakpoint(uint32_t address) {
    for (auto it = breakpoints.begin(); it != breakpoints.end(); ++it) {
        if (*it == address) {
            breakpoints.erase(it);
            break;
        }
    }
}

bool DebugTracer::checkBreakpoint(uint32_t address) {
    for (auto bp : breakpoints) {
        if (bp == address) {
            std::stringstream ss;
            ss << "[BREAKPOINT] Hit at 0x" << std::hex << std::setw(8) << std::setfill('0') << address;
            log(ss.str());
            return true;
        }
    }
    return false;
}

std::string DebugTracer::getRegisterName(int index) {
    static const std::string names[] = {
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
        "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"
    };
    
    if (index >= 0 && index < 16) {
        return names[index];
    }
    
    return "r?";
}

std::string DebugTracer::getFlagString() {
    uint32_t cpsr = cpu.getCPSR();
    std::stringstream ss;
    ss << "CPSR=0x" << std::hex << std::setw(8) << std::setfill('0') << cpsr
       << " [" 
       << (cpsr & N_FLAG ? "N" : "-")
       << (cpsr & Z_FLAG ? "Z" : "-")
       << (cpsr & C_FLAG ? "C" : "-")
       << (cpsr & V_FLAG ? "V" : "-")
       << (cpsr & I_FLAG ? "I" : "-")
       << (cpsr & F_FLAG ? "F" : "-")
       << (cpsr & T_BIT ? "T" : "-")
       << "] "
       << "Mode=";
    
    switch (cpsr & MODE_MASK) {
        case USR_MODE: ss << "USR"; break;
        case FIQ_MODE: ss << "FIQ"; break;
        case IRQ_MODE: ss << "IRQ"; break;
        case SVC_MODE: ss << "SVC"; break;
        case ABT_MODE: ss << "ABT"; break;
        case UND_MODE: ss << "UND"; break;
        case SYS_MODE: ss << "SYS"; break;
        default: ss << "???"; break;
    }
    
    return ss.str();
}

void DebugTracer::traceDisplayControl(uint16_t dispcnt) {
    if (traceLevel < NORMAL) return;
    
    std::stringstream ss;
    ss << "[DISPLAY] DISPCNT=0x" << std::hex << std::setw(4) << std::setfill('0') << dispcnt
       << " Mode=" << (dispcnt & 0x7)
       << " BG0=" << ((dispcnt & 0x0100) ? "ON" : "off")
       << " BG1=" << ((dispcnt & 0x0200) ? "ON" : "off")
       << " BG2=" << ((dispcnt & 0x0400) ? "ON" : "off")
       << " BG3=" << ((dispcnt & 0x0800) ? "ON" : "off");
    log(ss.str());
}

void DebugTracer::traceBGControl(int bg_index, uint16_t bgcnt) {
    if (traceLevel < NORMAL) return;
    
    std::stringstream ss;
    ss << "[DISPLAY] BG" << bg_index << "CNT=0x" << std::hex << std::setw(4) << std::setfill('0') << bgcnt
       << " Priority=" << (bgcnt & 0x3)
       << " CharBaseBlock=" << ((bgcnt >> 2) & 0x3)
       << " ScreenBaseBlock=" << ((bgcnt >> 8) & 0x1F);
    log(ss.str());
}

void DebugTracer::traceVRAMWrite(uint32_t address, uint16_t value) {
    if (traceLevel < VERBOSE) return;
    
    // Only trace writes to key areas to avoid overwhelming logs
    if ((address & 0xFFFF) == 0 || (address & 0xFF) == 0) {
        std::stringstream ss;
        ss << "[VRAM] Write to 0x" << std::hex << std::setw(8) << std::setfill('0') << address
           << " = 0x" << std::setw(4) << value;
        log(ss.str());
    }
}
void DebugTracer::tracePPUMode4State(uint16_t dispcnt, uint32_t frameCount) {
    if (traceLevel < NORMAL) return;
    
    std::stringstream ss;
    ss << "[PPU] Mode 4 frame " << frameCount 
       << " page " << ((dispcnt & 0x10) ? "1" : "0");
    log(ss.str());
}

void DebugTracer::traceBitmapData(uint32_t vramAddr, uint32_t length, uint32_t frameOffset) {
    if (traceLevel < VERBOSE) return;
    
    std::stringstream ss;
    ss << "[VRAM] First " << length << " pixels of Mode 4 bitmap:";
    log(ss.str());
    
    ss.str("");
    for (int i = 0; i < length; i++) {
        uint8_t idx = memory->read8(vramAddr + frameOffset + i);
        ss << std::hex << (int)idx << " ";
    }
    log(ss.str());
}

void DebugTracer::dumpVRAMContents() {
    if (!memory || traceLevel < VERBOSE) return;
    
    std::stringstream ss;
    ss << "[VRAM] Checking for character data...";
    log(ss.str());
    
    // Check first few character blocks for data
    for (int block = 0; block < 4; block++) {
        uint32_t baseAddr = 0x06000000 + block * 0x4000;
        bool foundData = false;
        
        for (int offset = 0; offset < 0x1000 && !foundData; offset += 0x100) {
            uint32_t addr = baseAddr + offset;
            uint32_t data = 0;
            for (int i = 0; i < 16; i += 2) {
                data |= memory->read16(addr + i);
            }
            
            if (data != 0) {
                ss.str("");
                ss << "[VRAM] Found data in CharBlock " << block << " at offset 0x" 
                   << std::hex << offset << std::dec;
                log(ss.str());
                foundData = true;
            }
        }
        
        if (!foundData) {
            ss.str("");
            ss << "[VRAM] No data found in CharBlock " << block;
            log(ss.str());
        }
    }
}

void DebugTracer::dumpPaletteContents() {
    if (!memory || traceLevel < VERBOSE) return;
    
    std::stringstream ss;
    ss << "[PALETTE] First 16 entries:";
    log(ss.str());
    
    for (int i = 0; i < 16; i++) {
        uint16_t color = memory->read16(0x05000000 + i*2);
        if (color != 0 || i == 0) {
            ss.str("");
            ss << "[PALETTE] " << i << " = 0x" << std::hex << std::setw(4) 
               << std::setfill('0') << color << std::dec 
               << " (R:" << ((color & 0x1F)) 
               << " G:" << ((color >> 5) & 0x1F) 
               << " B:" << ((color >> 10) & 0x1F) << ")";
            log(ss.str());
        }
    }
}

void DebugTracer::dumpTileMap(int screenBlock) {
    if (!memory || traceLevel < VERBOSE) return;
    
    std::stringstream ss;
    ss << "[TILEMAP] Screen Block " << screenBlock << " entries:";
    log(ss.str());
    
    uint32_t baseAddr = 0x06000000 + screenBlock * 0x800;
    bool foundEntries = false;
    
    // Just check first few rows (30 entries per row)
    for (int i = 0; i < 60; i++) {
        uint16_t entry = memory->read16(baseAddr + i*2);
        if (entry != 0) {
            ss.str("");
            ss << "[TILEMAP] Entry " << i << " = 0x" << std::hex << entry 
               << " (TileID: " << (entry & 0x3FF) 
               << ", PalBank: " << ((entry >> 12) & 0xF) << ")" << std::dec;
            log(ss.str());
            foundEntries = true;
        }
    }
    
    if (!foundEntries) {
        ss.str("");
        ss << "[TILEMAP] No non-zero entries found in first 60 positions";
        log(ss.str());
    }
}

void DebugTracer::log(const std::string& message) {
    // Only write to the log file, not to the console
    if (logFile.is_open()) {
        logFile << message << std::endl;
    } else if (outputToConsole) {
        // Fall back to console only if no log file is open and console output is enabled
        std::cout << message << std::endl;
    }
}
