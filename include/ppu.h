#ifndef PPU_H
#define PPU_H

#include <cstdint>
#include <vector>
#include "memory.h"
#include <iostream>

// GBA display dimensions
#define GBA_SCREEN_WIDTH 240
#define GBA_SCREEN_HEIGHT 160

// Memory addresses
#define VRAM_BASE 0x06000000
#define PALETTE_RAM 0x05000000

// IO registers
#define REG_BG0CNT 0x04000008
#define REG_BG1CNT 0x0400000A
#define REG_BG2CNT 0x0400000C
#define REG_BG3CNT 0x0400000E
#define REG_BG0HOFS 0x04000010
#define REG_BG0VOFS 0x04000012
#define REG_BG1HOFS 0x04000014
#define REG_BG1VOFS 0x04000016
#define REG_BG2HOFS 0x04000018
#define REG_BG2VOFS 0x0400001A
#define REG_BG3HOFS 0x0400001C
#define REG_BG3VOFS 0x0400001E

class PPU {
public:
    PPU(Memory& mem);

    void step(int cycles);
    const uint32_t* get_pixel_buffer() const;
    bool is_frame_ready() const;
    void frame_rendered();
    void toggle_debug_mode() { 
    debug_enabled = !debug_enabled; 
    std::cout << "[PPU] Debug mode " << (debug_enabled ? "enabled" : "disabled") << std::endl;
	}
	bool is_debug_enabled() const { return debug_enabled; }
	void setDebugEnabled(bool enable) { debugEnabled = enable; }

private:
    void render_scanline();
    void dump_registers();
    
    // Rendering functions for different modes
    void render_mode0_scanline(uint32_t* scanline_buffer, uint16_t dispcnt);
    void render_mode1_scanline();
    void render_mode2_scanline();
    void render_mode3_scanline();
    void render_mode4_scanline();
    void render_mode5_scanline();
    
    // Background rendering functions
    void render_text_bg(int bg_index, uint32_t* scanline_buffer);
    
    uint32_t convert_color(uint16_t gba_color);

    Memory& memory;
    std::vector<uint32_t> pixel_buffer;

    int cycle_count;
    int current_scanline;
    bool frame_ready_flag;
    bool debug_enabled;
    int frame_count = 0;

    static const int CYCLES_PER_SCANLINE = 1232;
    static const int HBLANK_CYCLES = 272;
    static const int SCANLINE_CYCLES = 960; // H-Draw
    static const int VISIBLE_HEIGHT = 160;
    static const int VBLANK_HEIGHT = 68;
    static const int TOTAL_HEIGHT = VISIBLE_HEIGHT + VBLANK_HEIGHT;
    
    //Debug
    bool debugEnabled = false;
};

#endif // PPU_H
