CXX = g++
# Add SDL2 flags
CXXFLAGS = -std=c++17 -Wall -g -O0 `sdl2-config --cflags` -Iinclude
# Add SDL2 linker flags
LDFLAGS = `sdl2-config --libs`

SRCDIR = src
OBJDIR = obj

# List sources in compilation order to ensure core dependencies are built first
SOURCES = \
    $(SRCDIR)/memory.cpp \
    $(SRCDIR)/debug.cpp \
    $(SRCDIR)/cpu_debug.cpp \
    $(SRCDIR)/cpu.cpp \
    $(SRCDIR)/ppu.cpp \
    $(SRCDIR)/display.cpp \
    $(SRCDIR)/main.cpp

OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

EXECUTABLE = gba_emulator

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJDIR) $(EXECUTABLE)

.PHONY: all clean
