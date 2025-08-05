#ifndef PPU_H
#define PPU_H

#include <cstdint>
#include <vector>
#include "memory.h"

class PPU {
public:
    PPU(Memory& mem);

    void step(int cycles);
    const uint32_t* get_pixel_buffer() const;
    bool is_frame_ready() const;
    void frame_rendered();

private:
    void render_scanline();
    void render_mode3_scanline();
    // Add other mode renderers later
    uint32_t convert_color(uint16_t gba_color);

    Memory& memory;
    std::vector<uint32_t> pixel_buffer;

    int cycle_count;
    int current_scanline;
    bool frame_ready_flag;

    static const int CYCLES_PER_SCANLINE = 1232;
    static const int HBLANK_CYCLES = 272;
    static const int SCANLINE_CYCLES = 960; // H-Draw
    static const int VISIBLE_HEIGHT = 160;
    static const int VBLANK_HEIGHT = 68;
    static const int TOTAL_HEIGHT = VISIBLE_HEIGHT + VBLANK_HEIGHT;
};

#endif // PPU_H
