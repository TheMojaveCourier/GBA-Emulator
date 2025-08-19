#include "ppu.h"
#include <iostream>
#include <cstring>
#include <iomanip>

PPU::PPU(Memory& mem) : memory(mem), cycle_count(0), current_scanline(0), frame_ready_flag(false) {
    // Initialize pixel buffer (240x160 RGBA)
    pixel_buffer.resize(GBA_SCREEN_WIDTH * GBA_SCREEN_HEIGHT, 0);
    
    // Set debug flags - true for development, false for production
    debug_enabled = true;
    frame_count = 0;
    
	if (debugEnabled) {
		std::cout << "[PPU] Initialized with debug mode enabled" << std::endl;
	}
}

void PPU::step(int cycles) {
    cycle_count += cycles;
    
    // Check if we're ready to render a scanline
    if (cycle_count >= CYCLES_PER_SCANLINE) {
        cycle_count -= CYCLES_PER_SCANLINE;
        
        // Update VCOUNT register
        memory.write16(REG_VCOUNT, current_scanline);
        
        // If we're in the visible area, render the scanline
        if (current_scanline < VISIBLE_HEIGHT) {
            render_scanline();
        }
        
        // Update scanline counter
        current_scanline++;
        if (current_scanline >= TOTAL_HEIGHT) {
            current_scanline = 0;
            frame_ready_flag = true;
            
            // Set VBlank interrupt flag
            uint16_t if_value = memory.read16(REG_IF);
            memory.write16(REG_IF, if_value | 0x0001);
            
            // Debug dump of key registers on frame boundary
            if (debug_enabled && frame_count++ % 60 == 0) {
                dump_registers();
            }
        }
    }
}

void PPU::dump_registers() {
    // If debug is disabled, don't do any of the logging
    if (!debugEnabled) {
        return;
    }
    
    // DISPCNT
    uint16_t dispcnt = memory.read16(REG_DISPCNT);
    std::cout << "DISPCNT = 0x" << std::hex << std::setw(4) << std::setfill('0') << dispcnt << std::dec 
              << " (Mode " << (dispcnt & 0x7) << ", ";
    if (dispcnt & 0x0100) std::cout << "BG0 ";
    if (dispcnt & 0x0200) std::cout << "BG1 ";
    if (dispcnt & 0x0400) std::cout << "BG2 ";
    if (dispcnt & 0x0800) std::cout << "BG3 ";
    std::cout << ")" << std::endl;
    
    // BG Control registers
    uint16_t bg0cnt = memory.read16(REG_BG0CNT);
    std::cout << "BG0CNT = 0x" << std::hex << std::setw(4) << std::setfill('0') << bg0cnt 
              << " (Priority " << ((bg0cnt >> 2) & 0x3)
              << ", CBB " << ((bg0cnt >> 2) & 0x3)
              << ", SBB " << ((bg0cnt >> 8) & 0x1F)
              << ")" << std::endl;
    
    // Dump first 16 palette entries
    std::cout << "Palette RAM:" << std::endl;
    for (int i = 0; i < 16; i++) {
        uint16_t color = memory.read16(PALETTE_RAM + i * 2);
        std::cout << "  [" << std::setw(2) << i << "]: 0x" << std::hex << std::setw(4) 
                  << color << " (";
        
        // Show RGB components
        int r = color & 0x1F;
        int g = (color >> 5) & 0x1F;
        int b = (color >> 10) & 0x1F;
        std::cout << "R:" << std::setw(2) << r << " G:" << std::setw(2) << g 
                  << " B:" << std::setw(2) << b << ")" << std::dec << std::endl;
    }
    
    // Dump tile data from the first character block
    std::cout << "Checking for tile data:" << std::endl;
    bool found_nonzero = false;
    for (int tile = 0; tile < 16 && !found_nonzero; tile++) {
        for (int y = 0; y < 8 && !found_nonzero; y++) {
            uint32_t addr = VRAM_BASE + tile * 32 + y * 4;
            uint32_t data = memory.read32(addr);
            if (data != 0) {
                found_nonzero = true;
                std::cout << "  Nonzero tile data found at tile " << tile 
                          << ", row " << y << ": 0x" << std::hex << data << std::dec << std::endl;
            }
        }
    }
    if (!found_nonzero) {
        std::cout << "  All checked tile data is zero" << std::endl;
    }
    
    // Check the first few screen entries in screen block 1 (where the ROM is writing)
    std::cout << "Screen entries (Block 1):" << std::endl;
    for (int i = 0; i < 8; i++) {
        uint16_t entry = memory.read16(VRAM_BASE + 0x4000 + i * 2);
        std::cout << "  [" << std::setw(2) << i << "]: 0x" << std::hex << std::setw(4) 
                  << entry << " (Tile: " << (entry & 0x3FF) 
                  << ", Palette: " << ((entry >> 12) & 0xF) << ")" << std::dec << std::endl;
    }
    
    std::cout << "=== END PPU REGISTERS ===\n" << std::endl;
}

void PPU::render_scanline() {
    // Get display control register
    uint16_t dispcnt = memory.read16(REG_DISPCNT);
    
    // Extract video mode (0-5)
    uint8_t mode = dispcnt & 0x7;
    
    // Initialize scanline buffer with backdrop color (palette entry 0)
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    uint16_t backdrop_color = memory.read16(PALETTE_RAM);
    uint32_t backdrop_rgba = convert_color(backdrop_color);
    
    // Fill with backdrop color
    for (int i = 0; i < GBA_SCREEN_WIDTH; i++) {
        scanline_buffer[i] = backdrop_rgba;
    }
    
    if (debugEnabled && current_scanline == 0) {
        std::cout << "[PPU] Rendering in mode " << (int)mode 
                  << ", backdrop color = 0x" << std::hex << backdrop_color << std::dec << std::endl;
    }
    
    // Dispatch to appropriate mode renderer
    switch (mode) {
        case 0:
            // Text mode (4 backgrounds)
            render_mode0_scanline(scanline_buffer, dispcnt);
            break;
        case 1:
            // Mixed mode (2 text, 1 affine background)
            render_mode1_scanline();
            break;
        case 2:
            // Affine mode (2 affine backgrounds)
            render_mode2_scanline();
            break;
        case 3:
            // Direct color bitmap mode (1 16-bit bitmap)
            render_mode3_scanline();
            break;
        case 4:
            // Indexed bitmap mode (1 8-bit bitmap)
            render_mode4_scanline();
            break;
        case 5:
            // Smaller bitmap mode (1 16-bit bitmap, smaller)
            render_mode5_scanline();
            break;
        default:
            std::cerr << "[PPU] Unsupported video mode: " << (int)mode << std::endl;
            break;
    }
    
    // Copy the scanline buffer to the pixel buffer
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

// Implement a simple version of Mode 0 rendering
void PPU::render_mode0_scanline(uint32_t* scanline_buffer, uint16_t dispcnt) {
    // Check which backgrounds are enabled
    bool bg0_enabled = (dispcnt & 0x0100) != 0;
    bool bg1_enabled = (dispcnt & 0x0200) != 0;
    bool bg2_enabled = (dispcnt & 0x0400) != 0;
    bool bg3_enabled = (dispcnt & 0x0800) != 0;
    
    // For debugging - in the first frame, output what BGs are enabled
    if (debugEnabled && current_scanline == 0 && frame_count == 0) {
        std::cout << "[PPU] Mode 0 BGs enabled: " 
                  << (bg0_enabled ? "BG0 " : "")
                  << (bg1_enabled ? "BG1 " : "")
                  << (bg2_enabled ? "BG2 " : "")
                  << (bg3_enabled ? "BG3 " : "")
                  << std::endl;
    }
    
    // Debug options
    if (debug_enabled) {
        // Create a debug pattern based on frame count
        uint8_t pattern_type = (frame_count / 30) % 3;
        
        // Different debug patterns
        switch (pattern_type) {
            case 0: // RGB gradient
                for (int x = 0; x < GBA_SCREEN_WIDTH; x++) {
                    uint8_t r = x / 8 * 16;  // Horizontal gradient
                    uint8_t g = current_scanline / 8 * 16;  // Vertical gradient
                    uint8_t b = (x + current_scanline) / 16 * 16;  // Diagonal gradient
                    
                    scanline_buffer[x] = 0xFF000000 | (r << 16) | (g << 8) | b;
                }
                break;
                
            case 1: // Checkerboard
                for (int x = 0; x < GBA_SCREEN_WIDTH; x++) {
                    bool isWhite = ((x / 16) + (current_scanline / 16)) % 2 == 0;
                    scanline_buffer[x] = isWhite ? 0xFFFFFFFF : 0xFF000000;
                }
                break;
                
            case 2: // Rainbow
                for (int x = 0; x < GBA_SCREEN_WIDTH; x++) {
                    int hue = (x + current_scanline + frame_count) % 360;
                    float h = hue / 60.0f;
                    float s = 1.0f;
                    float v = 1.0f;
                    
                    // Simple HSV to RGB conversion
                    int i = (int)h;
                    float f = h - i;
                    float p = v * (1.0f - s);
                    float q = v * (1.0f - s * f);
                    float t = v * (1.0f - s * (1.0f - f));
                    
                    uint8_t r, g, b;
                    switch (i) {
                        case 0: r = v * 255; g = t * 255; b = p * 255; break;
                        case 1: r = q * 255; g = v * 255; b = p * 255; break;
                        case 2: r = p * 255; g = v * 255; b = t * 255; break;
                        case 3: r = p * 255; g = q * 255; b = v * 255; break;
                        case 4: r = t * 255; g = p * 255; b = v * 255; break;
                        default: r = v * 255; g = p * 255; b = q * 255; break;
                    }
                    
                    scanline_buffer[x] = 0xFF000000 | (r << 16) | (g << 8) | b;
                }
                break;
        }
        return;
    }
    
    // Normal Mode 0 rendering when not in debug mode
    // Proceed with normal BG rendering if debug pattern is disabled
    if (bg0_enabled) {
        render_text_bg(0, scanline_buffer);
    }
    
    if (bg1_enabled) {
        render_text_bg(1, scanline_buffer);
    }
    
    if (bg2_enabled) {
        render_text_bg(2, scanline_buffer);
    }
    
    if (bg3_enabled) {
        render_text_bg(3, scanline_buffer);
    }
}

void PPU::render_text_bg(int bg_index, uint32_t* scanline_buffer) {
    // Read BG control register for this background
    uint16_t bg_cnt = memory.read16(REG_BG0CNT + bg_index * 2);
    
    // Extract BG parameters
    uint8_t priority = (bg_cnt >> 2) & 0x3;
    uint8_t char_base_block = (bg_cnt >> 2) & 0x3;
    uint8_t mosaic = (bg_cnt >> 6) & 0x1;
    uint8_t colors = (bg_cnt >> 7) & 0x1; // 0=16/16, 1=256/1
    uint8_t screen_base_block = (bg_cnt >> 8) & 0x1F;
    uint8_t screen_size = (bg_cnt >> 14) & 0x3;
    
    // Calculate base addresses
    uint32_t char_base_addr = VRAM_BASE + char_base_block * 0x4000;
    uint32_t screen_base_addr = VRAM_BASE + screen_base_block * 0x800;
    
    // Read scroll registers
    uint16_t h_offset = memory.read16(REG_BG0HOFS + bg_index * 4) & 0x1FF;
    uint16_t v_offset = memory.read16(REG_BG0VOFS + bg_index * 4) & 0x1FF;
    
    // Calculate screen size in tiles
    int screen_width, screen_height;
    switch (screen_size) {
        case 0: screen_width = 32; screen_height = 32; break;
        case 1: screen_width = 64; screen_height = 32; break;
        case 2: screen_width = 32; screen_height = 64; break;
        case 3: screen_width = 64; screen_height = 64; break;
    }
    
    // Calculate the pixel Y coordinate in the tilemap
    int y = (current_scanline + v_offset) & 0x1FF;
    int tile_y = y & 7;
    int map_y = y >> 3;
    
    // Log some details for debugging
    if (current_scanline == 0 && frame_count == 0) {
        std::cout << "[BG" << bg_index << "] Priority: " << (int)priority
                  << ", Char Base: " << (int)char_base_block
                  << ", Screen Base: " << (int)screen_base_block
                  << ", " << (colors ? "256/1" : "16/16") << " colors"
                  << ", Scroll: (" << h_offset << "," << v_offset << ")"
                  << std::endl;
    }
    
    // For each pixel in the scanline
    for (int x = 0; x < GBA_SCREEN_WIDTH; x++) {
        // Calculate the pixel X coordinate in the tilemap
        int map_x = (x + h_offset) & 0x1FF;
        int tile_x = map_x & 7;
        int map_tile_x = map_x >> 3;
        
        // Calculate which section of the map we're in based on screen size
        int section_x = map_tile_x / 32;
        int section_y = map_y / 32;
        int section = section_y * (screen_width / 32) + section_x;
        
        // Get the map entry address
        uint32_t map_addr = screen_base_addr + (((map_y % 32) * 32) + (map_tile_x % 32)) * 2;
        if (section > 0) {
            map_addr += section * 0x800;
        }
        
        // Read the map entry
        uint16_t map_entry = memory.read16(map_addr);
        uint16_t tile_id = map_entry & 0x3FF;
        bool h_flip = (map_entry >> 10) & 1;
        bool v_flip = (map_entry >> 11) & 1;
        uint8_t palette_num = (map_entry >> 12) & 0xF;
        
        // Account for flipping
        uint8_t final_tile_x = h_flip ? (7 - tile_x) : tile_x;
        uint8_t final_tile_y = v_flip ? (7 - tile_y) : tile_y;
        
        // Calculate the address of the tile data
        uint32_t tile_addr;
        uint8_t color_index = 0;
        
        if (colors) {  // 256 colors / 1 palette
            tile_addr = char_base_addr + tile_id * 64 + final_tile_y * 8 + final_tile_x;
            color_index = memory.read8(tile_addr);
        } else {  // 16 colors / 16 palettes
            tile_addr = char_base_addr + tile_id * 32 + final_tile_y * 4 + (final_tile_x >> 1);
            uint8_t tile_data = memory.read8(tile_addr);
            
            // Extract the correct 4 bits
            if (final_tile_x & 1) {
                color_index = tile_data >> 4;
            } else {
                color_index = tile_data & 0xF;
            }
            
            // Apply palette number if the color isn't transparent
            if (color_index != 0) {
                color_index += palette_num * 16;
            }
        }
        
        // Only draw if the color isn't transparent (0)
        if (color_index != 0) {
            // Get the color from the palette
            uint16_t color16 = memory.read16(PALETTE_RAM + color_index * 2);
            scanline_buffer[x] = convert_color(color16);
        }
    }
}

// For now, implement simplified versions of other modes
void PPU::render_mode1_scanline() {
    // Fill with a blue color for testing
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    for (int i = 0; i < GBA_SCREEN_WIDTH; i++) {
        scanline_buffer[i] = 0xFF0000FF; // Blue color (RGBA)
    }
    
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

void PPU::render_mode2_scanline() {
    // Fill with a green color for testing
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    for (int i = 0; i < GBA_SCREEN_WIDTH; i++) {
        scanline_buffer[i] = 0xFF00FF00; // Green color (RGBA)
    }
    
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

void PPU::render_mode3_scanline() {
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    
    // In Mode 3, each pixel is a 16-bit color directly in VRAM
    uint32_t vram_offset = current_scanline * GBA_SCREEN_WIDTH * 2;
    
    for (int x = 0; x < GBA_SCREEN_WIDTH; x++) {
        uint16_t color16 = memory.read16(VRAM_BASE + vram_offset + x * 2);
        scanline_buffer[x] = convert_color(color16);
    }
    
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

void PPU::render_mode4_scanline() {
    // Fill with a yellow color for testing
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    for (int i = 0; i < GBA_SCREEN_WIDTH; i++) {
        scanline_buffer[i] = 0xFFFFFF00; // Yellow color (RGBA)
    }
    
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

void PPU::render_mode5_scanline() {
    // Fill with a purple color for testing
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    for (int i = 0; i < GBA_SCREEN_WIDTH; i++) {
        scanline_buffer[i] = 0xFFFF00FF; // Purple color (RGBA)
    }
    
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

uint32_t PPU::convert_color(uint16_t gba_color) {
    // Convert GBA RGB555 color to RGBA8888
    uint8_t r = (gba_color & 0x1F) << 3;
    uint8_t g = ((gba_color >> 5) & 0x1F) << 3;
    uint8_t b = ((gba_color >> 10) & 0x1F) << 3;
    
    // Apply a small correction to make colors more accurate
    r |= r >> 5;
    g |= g >> 5;
    b |= b >> 5;
    
    return 0xFF000000 | (r << 16) | (g << 8) | b;
}

const uint32_t* PPU::get_pixel_buffer() const {
    return pixel_buffer.data();
}

bool PPU::is_frame_ready() const {
    return frame_ready_flag;
}

void PPU::frame_rendered() {
    frame_ready_flag = false;
}
