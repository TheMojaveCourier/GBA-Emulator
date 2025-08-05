#include <iostream>
#include "memory.h"
#include "cpu.h"
#include "ppu.h"
#include "display.h"
#include <fstream>
#include <vector>
#include <stdexcept>
#include <iomanip>
#include <SDL2/SDL.h>

void forceDisplaySetup(Memory& memory);

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

void forceDisplaySetup(Memory& memory) {
    std::cout << "[DEBUG] Forcing Mode 3 display setup for testing" << std::endl;
    
    // Force DISPCNT = MODE_3 | BG2_ON (0x0403)
    memory.write16(REG_DISPCNT, 0x0403);
    
    // Fill VRAM with red pixels (RGB555: 31,0,0 = 0x001F)
    std::cout << "[DEBUG] Filling VRAM with red pixels" << std::endl;
    for (uint32_t i = 0; i < 240*160; i++) {
        memory.write16(0x06000000 + (i*2), 0x001F);
    }
    
    std::cout << "[DEBUG] Display setup complete" << std::endl;
}

int main(int argc, char* argv[]) {
    try {
        std::string romPath = "roms/test_mmz.gba";
        if (argc > 1) romPath = argv[1];
        
        std::vector<uint8_t> romData = loadROM(romPath);
        std::cout << "ROM loaded (" << romData.size() << " bytes)" << std::endl;

        // Initialize components
        Display display;
        Memory memory;
        memory.debugLogVRAM(true);
        memory.loadRom(romData);
        memory.debugLogVRAMWrites(true);
        memory.debugLogDISPCNTWrites(true);
        
        CPU cpu(memory);
        PPU ppu(memory);

        cpu.reset();
        
        // ADD THIS LINE - Force display setup for testing
        forceDisplaySetup(memory);
        
        // Rest of your existing main loop...
        while (!display.should_close()) {
            display.handle_events();
            
            while (!ppu.is_frame_ready()) {
                int cycles_ran = cpu.step();
                ppu.step(cycles_ran);
            }
            
            std::cout << "[MAIN] Updating display with new frame" << std::endl;
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
