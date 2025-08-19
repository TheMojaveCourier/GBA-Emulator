#include "display.h"
#include <SDL2/SDL.h>
#include <stdexcept>
#include <iostream>

Display::Display() : quit(false) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        throw std::runtime_error("SDL could not initialize! SDL_Error: " + std::string(SDL_GetError()));
    }

    window = SDL_CreateWindow("GBA Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              SCREEN_WIDTH * WINDOW_SCALE, SCREEN_HEIGHT * WINDOW_SCALE, SDL_WINDOW_SHOWN);
    if (!window) {
        throw std::runtime_error("Window could not be created! SDL_Error: " + std::string(SDL_GetError()));
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        throw std::runtime_error("Renderer could not be created! SDL_Error: " + std::string(SDL_GetError()));
    }

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                SCREEN_WIDTH, SCREEN_HEIGHT);
    if (!texture) {
        throw std::runtime_error("Texture could not be created! SDL_Error: " + std::string(SDL_GetError()));
    }
}

Display::~Display() {
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void Display::update(const uint32_t* pixel_buffer) {
    static int frame_count = 0;
    frame_count++;
    
    if (debugEnabled && frame_count % 60 == 0) {
        std::cout << "[DISPLAY] Updated frame " << frame_count << std::endl;
        
        // In debug mode, verify a few sample pixels
        uint32_t sample1 = pixel_buffer[0];          // Top-left
        uint32_t sample2 = pixel_buffer[240*80+120]; // Middle
        
        std::cout << "[DISPLAY] Sample pixels: TL=0x" << std::hex << sample1 
                  << " MID=0x" << sample2 << std::dec << std::endl;
    }
    
    SDL_UpdateTexture(texture, nullptr, pixel_buffer, SCREEN_WIDTH * sizeof(uint32_t));
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
}

void Display::handle_events() {
    SDL_Event e;
    while (SDL_PollEvent(&e) != 0) {
        if (e.type == SDL_QUIT) {
            quit = true;
        }
    }
}

bool Display::should_close() const {
    return quit;
}
