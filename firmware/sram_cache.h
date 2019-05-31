#ifndef CACHEH
#define CACHEH

#include <Arduino.h>
#include <SdFat.h>

#define SRAM_SIZE 256

bool sram_init();
bool sram_cache_data(byte *specifiers, int num_spec, byte *data, int bytes);
bool sram_copy_data (SdFile* the_file);
bool sram_has_space (int num_bytes);

#endif
