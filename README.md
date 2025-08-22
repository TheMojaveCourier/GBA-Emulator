# GBA-Emulator
## W.I.P. Loads ROMS to memory and can now run a basic execution loop. 
Basic display capabilities, it now runs a basic render test regardless the rom loaded so you won't have to look at a blank void. It requires a `.gba` rom to function. SDL2 dev library is required to build.

## To compile simply use the make file
```
make
```

## Run Debug Logging:      
* Run with full logging:
```‎
./gba_emulator roms/test.gba --debug=full
```
‎
* Run with minimal logging:
```
./gba_emulator roms/test.gba --debug=minimal
```
* Run with no extra logging:

```
./gba_emulator roms/test.gba --debug=none

```
* No debug outputalso :
```
./gba_emulator roms/test.gba
```
* New debugger only:
```
./gba_emulator roms/test.gba --debug=normal
```
* Legacy debug only:
```
./gba_emulator roms/test.gba --legacy-debug
```
* Both debug systems: 
```
./gba_emulator roms/test.gba --debug=normal --legacy-debug
```
* Debug to console as well as file: 
```
./gba_emulator roms/test.gba --debug-console
```
* Specify a custom log file:
```
./gba_emulator roms/test.gba --log-file=my_debug_output.log
```
