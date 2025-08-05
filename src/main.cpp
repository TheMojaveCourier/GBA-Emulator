#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "memory.h"
#include "cpu.h"

// Function that loads the ROM
std::vector<uint8_t> loadROM(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open ROM file: " + filename);
    }
    
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    if (size <= 0) {
        throw std::runtime_error("ROM file is empty or invalid: " + filename);
    }
    
    std::vector<uint8_t> rom(size);
    
    if (!file.read(reinterpret_cast<char*>(rom.data()), size)) {
        throw std::runtime_error("Failed to read ROM file: " + filename);
    }
    
    return rom;
}

int main(int argc, char* argv[]) {
    try {
        std::string romPath;
        
        // Check for ROM path argument
        if (argc > 1) {
            romPath = argv[1];
        } else {
            std::cout << "GBA Emulator" << std::endl;
            std::cout << "Usage: " << argv[0] << " <rom_file.gba>" << std::endl;
            std::cout << "Please provide a GBA ROM file to load." << std::endl;
            return 1;
        }
        
        // Load the ROM
        std::vector<uint8_t> rom = loadROM(romPath);
        std::cout << "ROM loaded successfully!" << std::endl;
        std::cout << "ROM size: " << rom.size() << " bytes (" 
                  << (rom.size() / 1024.0 / 1024.0) << " MB)" << std::endl;
        
        // Basic ROM validation
        if (rom.size() < 192) { // Minimum size for GBA ROM header
            std::cerr << "Warning: ROM file appears to be too small for a valid GBA ROM" << std::endl;
        }
        
        if (rom.size() > 32 * 1024 * 1024) { // 32MB max
            std::cerr << "Warning: ROM file is larger than maximum GBA cartridge size" << std::endl;
        }
        
        // Initialize components
        Memory memory;
        CPU cpu(memory);
        
        // Copy ROM to memory (ROM region starts at 0x08000000)
        // Ensure we don't exceed memory bounds
        size_t copySize = std::min(rom.size(), static_cast<size_t>(32 * 1024 * 1024));
        for (size_t i = 0; i < copySize; i++) {
            memory.write(0x08000000 + i, rom[i]);
        }
        
        std::cout << "ROM loaded into memory at 0x08000000" << std::endl;
        
        // Reset CPU to initial state
        cpu.reset();
        std::cout << "CPU initialized and reset" << std::endl;
        
        // Start emulation loop
        std::cout << "Starting emulation..." << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        
        // Simple emulation loop - in a real emulator you'd want better control
        int instructionCount = 0;
        const int maxInstructions = 1000; // Limit for safety during development
        
        while (instructionCount < maxInstructions) {
            try {
                cpu.step();
                instructionCount++;
            } catch (const std::exception& e) {
                std::cerr << "CPU execution error: " << e.what() << std::endl;
                break;
            }
        }
        
        if (instructionCount >= maxInstructions) {
            std::cout << "Reached instruction limit (" << maxInstructions 
                      << ") - stopping for safety" << std::endl;
        }
        
        std::cout << "Emulation finished after " << instructionCount << " instructions" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
