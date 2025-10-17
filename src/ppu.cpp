#include "ppu.h"
#include "debug.h"
#include <iostream>
#include <cstring>
#include <iomanip>

PPU::PPU(Memory& mem) : memory(mem), cycle_count(0), current_scanline(0), frame_ready_flag(false) {
    // Initialize pixel buffer (240x160 RGBA)
    pixel_buffer.resize(GBA_SCREEN_WIDTH * GBA_SCREEN_HEIGHT, 0);
    
    // Set debug flags to false by default
    debug_enabled = false;
    frame_count = 0;
    
    if (debugEnabled) {
        std::cout << "[PPU] Initialized with debug mode disabled" << std::endl;
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
    
    if (debugEnabled && current_scanline == 0 && frame_count % 60 == 0) {
        std::cout << "[PPU] Rendering in mode " << (int)mode 
                  << ", backdrop color = 0x" << std::hex << backdrop_color << std::dec << std::endl;
    }
    
    // Only use debug patterns if debug mode is enabled
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
    } else {
        // Actual ROM graphics rendering based on video mode
        switch (mode) {
            case 0:
                render_mode0_scanline(scanline_buffer, dispcnt);
                break;
            case 1:
                render_mode1_scanline();
                break;
            case 2:
                render_mode2_scanline();
                break;
            case 3:
                render_mode3_scanline();
                break;
            case 4:
                render_mode4_scanline();
                break;
            case 5:
                render_mode5_scanline();
                break;
            default:
                std::cerr << "[PPU] Unsupported video mode: " << (int)mode << std::endl;
                break;
        }
        
        // After rendering backgrounds, render sprites if enabled
        if (dispcnt & 0x1000) { // OBJ (sprites) enabled
            render_sprites_scanline(scanline_buffer, current_scanline);
        }
    }
    
    // Copy the scanline buffer to the pixel buffer
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

void PPU::render_sprites_scanline(uint32_t* scanline_buffer, int line) {
    // OAM (Object Attribute Memory) contains sprite data
    // Each sprite takes 8 bytes in OAM
    // There are 128 sprites in total (0x400 bytes of OAM)
    
    uint16_t dispcnt = memory.read16(REG_DISPCNT);
    bool one_dimensional_mapping = (dispcnt & 0x0200) != 0;
    
    // Start from the last sprite (lowest priority) and work upward
    for (int sprite = 127; sprite >= 0; sprite--) {
        uint32_t oam_addr = 0x07000000 + (sprite * 8);
        
        // Read sprite attributes
        uint16_t attr0 = memory.read16(oam_addr);
        uint16_t attr1 = memory.read16(oam_addr + 2);
        uint16_t attr2 = memory.read16(oam_addr + 4);
        
        // Check if sprite is disabled
        if (((attr0 >> 8) & 0x3) == 0x2) {
            continue; // Sprite is disabled
        }
        
        // Extract sprite information
        int y = attr0 & 0xFF;
        if (y > 160) y -= 256; // Handle wrap-around
        
        bool affine = (attr0 & 0x0100) != 0;
        bool double_size = affine && (attr0 & 0x0200);
        bool mosaic = (attr0 & 0x1000) != 0;
        bool palette_mode = (attr0 & 0x2000) != 0; // 0=16/16, 1=256/1
        
        int x = attr1 & 0x1FF;
        if (x > 240) x -= 512; // Handle wrap-around
        
        bool h_flip = !affine && (attr1 & 0x1000);
        bool v_flip = !affine && (attr1 & 0x2000);
        
        int size_key = ((attr0 >> 14) & 0x3);
        int shape_key = ((attr1 >> 14) & 0x3);
        
        // Determine sprite dimensions based on shape and size
        int width, height;
        
        // Shape: 0=Square, 1=Horizontal, 2=Vertical, 3=Prohibited
        switch (shape_key) {
            case 0: // Square
                switch (size_key) {
                    case 0: width = 8; height = 8; break;
                    case 1: width = 16; height = 16; break;
                    case 2: width = 32; height = 32; break;
                    case 3: width = 64; height = 64; break;
                    default: width = 8; height = 8; break;
                }
                break;
            case 1: // Horizontal
                switch (size_key) {
                    case 0: width = 16; height = 8; break;
                    case 1: width = 32; height = 8; break;
                    case 2: width = 32; height = 16; break;
                    case 3: width = 64; height = 32; break;
                    default: width = 8; height = 8; break;
                }
                break;
            case 2: // Vertical
                switch (size_key) {
                    case 0: width = 8; height = 16; break;
                    case 1: width = 8; height = 32; break;
                    case 2: width = 16; height = 32; break;
                    case 3: width = 32; height = 64; break;
                    default: width = 8; height = 8; break;
                }
                break;
            default:
                continue; // Invalid shape, skip this sprite
        }
        
        // Apply double size if needed (for affine sprites)
        if (double_size) {
            width *= 2;
            height *= 2;
        }
        
        // Check if this scanline intersects with the sprite
        if (line < y || line >= y + height) {
            continue; // Scanline doesn't intersect the sprite
        }
        
        // Calculate which line of the sprite we're drawing
        int sprite_line = line - y;
        if (v_flip) sprite_line = height - 1 - sprite_line;
        
        // Extract other attributes
        int tile_num = attr2 & 0x3FF;
        int priority = (attr2 >> 10) & 0x3;
        int palette_bank = (attr2 >> 12) & 0xF; // Only used for 16-color sprites
        
        // Now render the sprite line
        for (int sx = 0; sx < width; sx++) {
            int screen_x = x + sx;
            
            // Skip if off-screen
            if (screen_x < 0 || screen_x >= GBA_SCREEN_WIDTH) {
                continue;
            }
            
            // Calculate which pixel of the sprite to draw
            int sprite_x = sx;
            if (h_flip) sprite_x = width - 1 - sx;
            
            // Calculate tile and pixel coordinates
            int tile_x = sprite_x / 8;
            int tile_y = sprite_line / 8;
            int pixel_x = sprite_x % 8;
            int pixel_y = sprite_line % 8;
            
            // Calculate which tile to use
            int tile_id;
            if (one_dimensional_mapping) {
                // 1D mapping - tiles are arranged in a single row
                tile_id = tile_num + tile_y * (width / 8) + tile_x;
            } else {
                // 2D mapping - tiles are arranged in a 2D grid (32 tiles wide)
                tile_id = tile_num + tile_y * 32 + tile_x;
            }
            
            // Calculate the tile data address
            uint32_t tile_addr;
            uint8_t color_idx;
            
            if (palette_mode) {
                // 256-color / 1 palette mode (8 bits per pixel)
                tile_addr = 0x06010000 + (tile_id * 64) + (pixel_y * 8) + pixel_x;
                color_idx = memory.read8(tile_addr);
            } else {
                // 16-color / 16 palette mode (4 bits per pixel)
                tile_addr = 0x06010000 + (tile_id * 32) + (pixel_y * 4) + (pixel_x / 2);
                uint8_t byte = memory.read8(tile_addr);
                
                // Extract the correct 4 bits
                if (pixel_x & 1) {
                    color_idx = byte >> 4;
                } else {
                    color_idx = byte & 0xF;
                }
                
                // If not transparent, apply palette bank
                if (color_idx != 0) {
                    color_idx += palette_bank * 16;
                }
            }
            
            // Skip transparent pixels
            if (color_idx == 0) {
                continue;
            }
            
            // Get the color from the palette
            uint16_t color;
            if (palette_mode) {
                // 256-color sprites use the second half of palette memory
                color = memory.read16(PALETTE_RAM + 0x200 + color_idx * 2);
            } else {
                // 16-color sprites also use the second half of palette memory
                color = memory.read16(PALETTE_RAM + 0x200 + color_idx * 2);
            }
            
            // Draw the pixel (for now, ignore priority and alpha blending)
            // In a more complete implementation, you'd check layer priorities
            scanline_buffer[screen_x] = convert_color(color);
        }
    }
}

// Implement a simple version of Mode 0 rendering
void PPU::render_mode0_scanline(uint32_t* scanline_buffer, uint16_t dispcnt) {
    // Check which backgrounds are enabled
    bool bg0_enabled = (dispcnt & 0x0100) != 0;
    bool bg1_enabled = (dispcnt & 0x0200) != 0;
    bool bg2_enabled = (dispcnt & 0x0400) != 0;
    bool bg3_enabled = (dispcnt & 0x0800) != 0;
    // Render backgrounds in priority order (3->2->1->0)
    // This ensures lower priority BGs appear behind higher priority ones
    
    // Create a priority buffer to track what's been drawn
    uint8_t priority_buffer[GBA_SCREEN_WIDTH] = {4}; // 4 = nothing drawn yet (0-3 are valid priorities)
    
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
    
    // Draw backgrounds in reverse order (lowest priority first)
    if (bg3_enabled) render_text_bg(3, scanline_buffer, priority_buffer);
    if (bg2_enabled) render_text_bg(2, scanline_buffer, priority_buffer);
    if (bg1_enabled) render_text_bg(1, scanline_buffer, priority_buffer);
    if (bg0_enabled) render_text_bg(0, scanline_buffer, priority_buffer);
}

void PPU::render_text_bg(int bg_index, uint32_t* scanline_buffer, uint8_t* priority_buffer) {
    // Read BG control register for this background
    uint16_t bg_cnt = memory.read16(REG_BG0CNT + bg_index * 2);
    
    // Extract BG parameters
    uint8_t priority = bg_cnt & 0x3;  // Priority (0-3, lower = higher priority)
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
            tile_addr = char_base_addr + tile_id * 32 + final_tile_y * 4 + (final_tile_x / 2);
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
        
        // Only draw if the color isn't transparent (0) and this BG has higher priority
        if (color_index != 0 && priority <= priority_buffer[x]) {
            // Get the color from the palette
            uint16_t color16 = memory.read16(PALETTE_RAM + color_index * 2);
            scanline_buffer[x] = convert_color(color16);
            priority_buffer[x] = priority;  // Update priority buffer
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
    // In Mode 3, VRAM is treated as a 240x160 16-bit color framebuffer
    // Each pixel is a 16-bit direct color value (RGB555)
    
    // Calculate offset into VRAM for the current scanline
    uint32_t vram_offset = current_scanline * GBA_SCREEN_WIDTH * 2; // 2 bytes per pixel
    
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    
    // Debug display of first frame VRAM values
    if (debugEnabled && frame_count == 0 && current_scanline == 0) {
        std::cout << "[PPU] Mode 3 VRAM first few pixels: ";
        for (int i = 0; i < 8; i++) {
            uint16_t color = memory.read16(VRAM_BASE + i * 2);
            std::cout << "0x" << std::hex << color << " ";
        }
        std::cout << std::dec << std::endl;
    }
    
    for (int x = 0; x < GBA_SCREEN_WIDTH; x++) {
        // Read the 16-bit color directly from VRAM
        uint16_t color16 = memory.read16(VRAM_BASE + vram_offset + x * 2);
        
        // Convert GBA RGB555 to RGBA8888 for display
        scanline_buffer[x] = convert_color(color16);
    }
    
    // Copy to pixel buffer - handled by render_scanline
    memcpy(
        pixel_buffer.data() + (current_scanline * GBA_SCREEN_WIDTH),
        scanline_buffer,
        GBA_SCREEN_WIDTH * sizeof(uint32_t)
    );
}

void PPU::render_mode4_scanline() {
    uint16_t dispcnt = memory.read16(REG_DISPCNT);
    
    // Determine which page/framebuffer to use (bit 4 of DISPCNT)
    uint32_t frame_offset = (dispcnt & 0x10) ? 0xA000 : 0;
    
    // Calculate offset into VRAM for the current scanline
    uint32_t vram_offset = current_scanline * GBA_SCREEN_WIDTH;
    
    uint32_t scanline_buffer[GBA_SCREEN_WIDTH];
    
    // Debug display of first frame VRAM values - use debugger instead of direct console output
    if (current_scanline == 0 && frame_count == 0 && debugger != nullptr) {
        // First frame, first scanline - good place for initial debug info
        std::stringstream ss;
        ss << "Mode 4 VRAM first few pixels (page " << (frame_offset ? "1" : "0") << "): ";
        
        for (int i = 0; i < 8; i++) {
            uint8_t index = memory.read8(VRAM_BASE + frame_offset + i);
            uint16_t color = memory.read16(PALETTE_RAM + index * 2);
            ss << "idx=" << (int)index << "->0x" << std::hex << color << " ";
        }
        
        // Use debugger's traceVRAMWrite or similar method
        debugger->traceMemoryAccess(VRAM_BASE + frame_offset, 0, false, 8);
    }
    
    // Enhanced debug - check beginning of every frame
    if (current_scanline == 0 && frame_count % 60 == 0 && debugger != nullptr) {
        debugger->tracePPUMode4State(dispcnt, frame_count);
        debugger->dumpPaletteContents();
        debugger->traceBitmapData(VRAM_BASE, 16, frame_offset);
    }
    
    // Actual rendering code - this is unchanged
    for (int x = 0; x < GBA_SCREEN_WIDTH; x++) {
        // Read the 8-bit palette index from VRAM
        uint8_t palette_idx = memory.read8(VRAM_BASE + frame_offset + vram_offset + x);
        
        // Read the corresponding color from palette RAM
        uint16_t color16 = memory.read16(PALETTE_RAM + palette_idx * 2);
        
        // Convert to RGBA8888
        scanline_buffer[x] = convert_color(color16);
    }
    
    // Copy to pixel buffer - handled by render_scanline
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

void PPU::triggerDiagnostics() {
    if (debugger) {
        debugger->dumpVRAMContents();
        debugger->dumpPaletteContents();
        
        // Check multiple screen blocks where the tile map might be
        for (int block = 16; block <= 20; block++) {
            debugger->dumpTileMap(block);
        }
    }
}
