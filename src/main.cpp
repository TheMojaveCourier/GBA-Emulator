#include <iostream>
#include "memory.h"
#include "cpu.h"
#include <fstream>


//Function That loads the ROM
std::vector<uint8_t> loadROM(const std::string& filename) {
	std::ifstream file(filename,std::ios::binary | std::ios::ate);
	
	if(!file.is_open()) {
		throw std::runtime_error("Failed to open ROM file");
	}
	
	std:: streamsize size = file.tellg();
	file.seekg(0, std::ios::beg);
	
	std::vector<uint8_t> rom(size);
	
	if(!file.read(reinterpret_cast<char*>(rom.data()), size)) {
		throw std::runtime_error("Failed to read ROM file!");
	}
	
	return rom;

}


int main() {
    try {
    	//Load the ROM
    	std::string romPath = "/home/users/GBAEmulator/roms/test.gba"; //Change according to your dir
    	std::vector<uint8_t> rom = loadROM(romPath);
    	std::cout << "ROM loaded successfully! ROM size: " << rom.size() << " bytes" << std::endl;
    	    		    	 
        // Initialize components
        Memory memory;
        CPU cpu(memory);
        
        //Copy ROM to memory (0x08000000 ROM region)
		for(size_t i = 0; i < rom.size() && i < 0x02000000; i++) {
			memory.write(0x08000000 + i, rom[i]);
		}

        // Reset CPU
        cpu.reset();
        
        while(true) {
        //for(int i = 0; i < 100; i++){	
        	cpu.step();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
     
    return 0;
}
