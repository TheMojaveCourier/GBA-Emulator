#ifndef DISPLAY_H
#define DISPLAY_H

#include <cstdint>

// Forward declare SDL types to avoid including SDL headers in our header
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

class Display {
public:
    Display();
    ~Display();

    void update(const uint32_t* pixel_buffer);
    bool should_close() const;
    void handle_events();

private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    bool quit;

    static const int SCREEN_WIDTH = 240;
    static const int SCREEN_HEIGHT = 160;
    static const int WINDOW_SCALE = 3;
};

#endif // DISPLAY_H
