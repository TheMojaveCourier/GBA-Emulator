# GBA-Emulator

A Game Boy Advance emulator implementation with comprehensive ARM7TDMI CPU instruction support.

## Features

- **Complete ARM7TDMI CPU implementation** with support for:
  - All data processing instructions (MOV, ADD, SUB, AND, ORR, etc.)
  - Load/Store instructions with all addressing modes
  - Load/Store Multiple (LDM/STM) for block transfers  
  - Multiply instructions (MUL/MLA)
  - Halfword and byte transfers (LDRH/STRH, LDRB/STRB) with sign extension
  - All ARM condition codes (EQ, NE, CS, CC, MI, PL, VS, VC, HI, LS, GE, LT, GT, LE, AL)
  - Complete shift operations (LSL, LSR, ASR, ROR)

- **Accurate GBA memory mapping**:
  - ROM mirroring across 0x08000000-0x0DFFFFFF region
  - SRAM support for save data (0x0E000000-0x0E00FFFF)
  - All GBA memory regions (BIOS, Work RAM, Chip RAM, I/O, Palette, VRAM, OAM)
  - Proper memory access validation and error handling

- **Enhanced emulation features**:
  - Robust instruction decoding and execution
  - Proper PC handling for branch instructions
  - Memory access with different data sizes (8, 16, 32-bit)
  - Command-line ROM loading with validation

## Building

```bash
make clean && make
```

## Usage

```bash
./gba_emulator <rom_file.gba>
```

## Example

```bash
./gba_emulator rom/game.gba
```

The emulator will load the ROM, initialize the CPU, and begin execution with instruction-level debugging output.

## Current Status

The emulator successfully handles complex instruction sequences including arithmetic operations, memory transfers, conditional branching, and multiply operations. It's ready for basic ROM compatibility testing and supports the critical instruction set needed for commercial games.

## Architecture

- `src/cpu.cpp` - ARM7TDMI CPU implementation with complete instruction set
- `src/memory.cpp` - GBA memory mapping and access handling  
- `src/main.cpp` - ROM loading and emulation control
- `include/` - Header files with class definitions  
