#include <iostream>
#include "memory.h"
#include "cpu.h"
#include "ppu.h"
#include "display.h"
#include "debug.h"
#include <fstream>
#include <vector>
#include <stdexcept>
#include <iomanip>
#include <SDL2/SDL.h>

std::vector<uint8_t> loadROM(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open ROM: " + filename);
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    std::vector<uint8_t> rom(size);
    if (!file.read(reinterpret_cast<char*>(rom.data()), size)) {
        throw std::runtime_error("Failed to read ROM");
    }
    return rom;
}

int main(int argc, char* argv[]) {
    try {
        std::string romPath = "";
        if (argc > 1) romPath = argv[1];
        
        std::vector<uint8_t> romData = loadROM(romPath);
        //std::cout << "ROM loaded (" << romData.size() << " bytes)" << std::endl;

        // Initialize components
        bool enableLegacyDebug = false;
        bool debuggerConsoleOutput = false;  // Default to file-only for new debugger
        std::string debugLogFile = "debug.log";
        
        Display display;
        
        Memory memory;
        memory.debugLogVRAM(true);
        memory.loadRom(romData);
        memory.debugLogVRAMWrites(true);
        memory.debugLogDISPCNTWrites(true);
        memory.setDebugEnabled(enableLegacyDebug);
        
        CPU cpu(memory);
        cpu.enableLegacyDebug(enableLegacyDebug);
        
        PPU ppu(memory);
        ppu.setDebugEnabled(enableLegacyDebug);
        
        // Debug configuration
        DebugTracer::TraceLevel level = DebugTracer::NONE;
        
        // Parse command line arguments
        for (int i = 1; i < argc; i++) {
		    std::string arg = argv[i];
		    if (arg == "--legacy-debug") {
		        enableLegacyDebug = true;
		    } else if (arg.find("--debug=") == 0) {
		        // Already handled for new debugger
		    } else if (arg == "--debug-console") {
		        debuggerConsoleOutput = true;
		    } else if (arg.find("--log-file=") == 0) {
		        debugLogFile = arg.substr(11);  // Extract filename after --log-file=
		    } else if (arg[0] != '-') {
		        // Assume it's a ROM path
		        romPath = arg;
		    }
		}
        
        // Command line options to control debugging level
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--debug=none") level = DebugTracer::NONE;
            else if (arg == "--debug=minimal") level = DebugTracer::MINIMAL;
            else if (arg == "--debug=normal") level = DebugTracer::NORMAL;
            else if (arg == "--debug=verbose") level = DebugTracer::VERBOSE;
            else if (arg == "--debug=full") level = DebugTracer::FULL;
        }
        
        DebugTracer tracer(cpu, level);
        tracer.setConsoleOutput(debuggerConsoleOutput);
    	tracer.setLogFile(debugLogFile);
        cpu.setDebugger(&tracer);
        
        // Set up log file
        tracer.setLogFile("debug.log");
        
        // Add breakpoints for key functions/memory points
        tracer.addBreakpoint(0x8000000);  // ROM entry point
        tracer.addBreakpoint(0x80000c0);  // Key position observed in logs
        tracer.addBreakpoint(0x8000ab0);  // First function call
        
        // If you want to see the memory state at initialization
        if (level >= DebugTracer::NORMAL) {
            tracer.dumpMemoryRange(0x3007fc0, 0x3007fe8);  // Stack memory region
            tracer.dumpMemoryRange(0x4000000, 0x4000010);  // IO registers
        }

        cpu.reset();
                
        while (!display.should_close()) {
            display.handle_events();
            
            while (!ppu.is_frame_ready()) {
                int cycles_ran = cpu.step();
                ppu.step(cycles_ran);
            }
            
            if (level >= DebugTracer::MINIMAL) {
                std::cout << "[MAIN] Updating display with new frame" << std::endl;
            }
            
            display.update(ppu.get_pixel_buffer());
            ppu.frame_rendered();
            
            SDL_Delay(16); // ~60 FPS instead of 10ms
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
