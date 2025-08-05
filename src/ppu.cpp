#include "ppu.h"
#include <stdexcept>
#include <iostream>

PPU::PPU(Memory& mem) : memory(mem), pixel_buffer(240 * 160), 
    cycle_count(0), current_scanline(0), frame_ready_flag(false) {}

bool PPU::is_frame_ready() const {  // Add const
    return frame_ready_flag;
}

const uint32_t* PPU::get_pixel_buffer() const {  // Add const
    return pixel_buffer.data();
}

void PPU::frame_rendered() {  // This one doesn't need const
    frame_ready_flag = false;
}

void PPU::step(int cycles) {
    cycle_count += cycles;

    if (cycle_count >= CYCLES_PER_SCANLINE) {
        cycle_count -= CYCLES_PER_SCANLINE;
        
        // Handle V-Blank and frame readiness
        if (current_scanline == VISIBLE_HEIGHT) {
            uint16_t if_flags = memory.read16(REG_IF);
            memory.write16(REG_IF, if_flags | 1);
            frame_ready_flag = true;
            std::cout << "[PPU] VBlank started" << std::endl;
        }

        // Increment the scanline counter first
        current_scanline++;
        
        // Check for wrapping and reset if needed
        if (current_scanline >= TOTAL_HEIGHT) {
            current_scanline = 0;
        }

        // Update the VCOUNT register with the correct value
        memory.write8(REG_VCOUNT, current_scanline);

        // Render visible scanlines
        if (current_scanline < VISIBLE_HEIGHT) {
            render_scanline();
        }
    }
}

void PPU::render_scanline() {
    uint16_t dispcnt = memory.read16(REG_DISPCNT);
    uint8_t mode = dispcnt & 0x7;

    static uint8_t last_mode = 0xFF; // Track mode changes
    if (mode != last_mode) {
        std::cout << "[PPU] Display mode changed from " << (int)last_mode 
                  << " to " << (int)mode << " (DISPCNT=0x" << std::hex << dispcnt << ")" << std::dec << std::endl;
        last_mode = mode;
    }

    if (current_scanline == 0 && mode == 3) {
        std::cout << "[PPU] Rendering frame in Mode 3" << std::endl;
    }

    switch (mode) {
        case 3: 
            render_mode3_scanline(); 
            break;
        default:
            // Fill with black for unsupported modes
            uint32_t offset = current_scanline * 240;
            for (int x = 0; x < 240; ++x) {
                pixel_buffer[offset + x] = 0xFF000000; // Black
            }
            if (current_scanline == 0 && mode != 0) {
                std::cout << "[PPU] Unsupported mode " << (int)mode << ", filling with black" << std::endl;
            }
            break;
    }
}

void PPU::render_mode3_scanline() {
    // Debugging to see if it's rendering
    // Only log on first scanline
    if (current_scanline == 0) {
        std::cout << "[PPU] Rendering scanline 0 in Mode 3" << std::endl;
        
        // Check a sample pixel from VRAM to verify data
        uint32_t vram_addr = 0x06000000;
        uint16_t sample_pixel = memory.read16(vram_addr);
        std::cout << "[PPU] VRAM sample at 0x" << std::hex << vram_addr 
                  << " = 0x" << sample_pixel << std::dec << std::endl;
    }

    // Each pixel in Mode 3 is a 16-bit color value.
    // The screen is 240 pixels wide.
    // The total size of a scanline in VRAM is 240 * 2 bytes.
    uint32_t scanline_vram_offset = current_scanline * 240 * 2;
    uint32_t pixel_buffer_offset = current_scanline * 240;

    for (int x = 0; x < 240; ++x) {
        // Calculate the address for the current pixel within VRAM.
        // VRAM starts at 0x06000000.
        uint32_t vram_address = 0x06000000 + scanline_vram_offset + (x * 2);
        
        // Read the 16-bit color value from memory.
        uint16_t color = memory.read16(vram_address);
        
        // Convert the 16-bit GBA color to a 32-bit color for display and
        // place it in our pixel buffer.
        pixel_buffer[pixel_buffer_offset + x] = convert_color(color);
    }
}

uint32_t PPU::convert_color(uint16_t gba_color) {
    // GBA format is BGR555: bbbbbgggggrrrrr
    
    // Extract the 5-bit color channels
    uint8_t r_5bit = gba_color & 0x1F;
    uint8_t g_5bit = (gba_color >> 5) & 0x1F;
    uint8_t b_5bit = (gba_color >> 10) & 0x1F;

    // Scale the 5-bit values to 8-bit (0-255)
    uint8_t r_8bit = (r_5bit * 255) / 31;
    uint8_t g_8bit = (g_5bit * 255) / 31;
    uint8_t b_8bit = (b_5bit * 255) / 31;

    // Compose the final 32-bit ARGB pixel (format: 0xAARRGGBB)
    return (0xFF << 24) | (r_8bit << 16) | (g_8bit << 8) | b_8bit;
}
