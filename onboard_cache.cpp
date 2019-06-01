#include "onboard_cache.h"
#include "stdint.h"

int ptr = 0;
byte SRAM[SRAM_SIZE];

void sram_set_pointer(int p) {
  ptr = p;
}

bool sram_write_byte(byte b) {
  if (ptr < SRAM_SIZE) {
    SRAM[ptr] = b;
    ptr += 1;
    return true;
  } else {
    return false;
  }
}

byte sram_get_byte (int p) {
  return SRAM[p];
}

void sram_clear_byte() {
  SRAM[ptr] = 0x00;
}

bool sram_init() {
  for (int i = 0 ; i < SRAM_SIZE  ; i++) {
    SRAM[i] = 0x00;
  }
  ptr = 0;
  return true;
}

bool sram_has_space(int num_bytes) {
  return ((ptr + num_bytes) < SRAM_SIZE);
}

bool sram_cache_data (byte *specifiers, int num_specifiers, byte *data, int bytes) {

  bool ok = true;

  ok &= sram_write_byte(num_specifiers);

  for (int i = 0 ; i < num_specifiers ; i++) {
    //Serial.println("S[" + String(i) + "] = " + String(specifiers[i]));
    ok &= sram_write_byte(specifiers[i]);
  }
  for (int i = 0 ; i < bytes ; i++) {
    //Serial.println("D[" + String(i) + "] = " + String(data[i]));
    ok &= sram_write_byte(data[i]);
  }

  return ok;
}

void output_datum (SdFile* the_file, int ndx, int leng, bool last) {
  unsigned int  bits16 = 0x0000;
  unsigned long bits32 = 0x00000000UL;

  switch (leng) {

    case 1:
      //Serial.print("D1[" + String(sram_get_byte(ndx)) + "]");
      the_file->print(sram_get_byte(ndx));
      break;

    case 2:
      {
        unsigned int msb = (unsigned int)sram_get_byte(ndx);
        unsigned int lsb = (unsigned int)sram_get_byte(ndx + 1);
        bits16 = msb << 8 | lsb;

        //Serial.print("D2[" + String(msb) + " " + String(lsb) + + " " + String(n2) + "]");
        the_file->print(bits16);
      }
      break;

    case 3:
      {
        unsigned long b1  = (unsigned long)sram_get_byte(ndx);
        unsigned long b2  = (unsigned long)sram_get_byte(ndx + 1);
        unsigned long b3  = (unsigned long)sram_get_byte(ndx + 2);
        bits32 = (b1 << 16) | (b2 << 8) | b3;

        //Serial.print("D3[" + String(b1, HEX) + " " + String(b2, HEX) + " " + String(b3, HEX) + " " + String(n3, HEX) + "]");
        the_file->print(bits32);

      }
      break;
      
    case 4:
      {
        unsigned long b1  = (unsigned long)sram_get_byte(ndx);
        unsigned long b2  = (unsigned long)sram_get_byte(ndx + 1);
        unsigned long b3  = (unsigned long)sram_get_byte(ndx + 2);
        unsigned long b4  = (unsigned long)sram_get_byte(ndx + 3);
        bits32 = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4;

        //Serial.print("D3[" + String(b1, HEX) + " " + String(b2, HEX) + " " + String(b3, HEX) + " " + String(n3, HEX) + "]");
        the_file->print(bits32);

      }
      break;
    default:
      the_file->print("OVER");
      break;
  }
  if (!last) {
    the_file->print(",");
  } else {
    the_file->println();
    //Serial.println();
  }
}

bool sram_copy_data (SdFile* the_file) {
  int ndx = 0;

  while (ndx < ptr) {
    int num_specifiers = sram_get_byte(ndx);
    // The data starts at my current index plus the number of specifiers.
    int data_start = ndx + num_specifiers + 1;
    int next_datum = data_start;
    bool last = false;

    for (int i = 0 ; i < num_specifiers ; i++) {
      if (i == num_specifiers - 1) {
        last = true;
      }

      // Now, I know how many specifiers there are.
      // They are the next *num_specifier* bytes.
      byte specifier = sram_get_byte(ndx + i + 1);

      //Serial.print("S: " + String(specifier) + " ");

      // I can output the next datum.
      // It is at the location of *next_datum*, and is of length *specifier*
      output_datum(the_file, next_datum, specifier, last);

      // The *next_datum* pointer needs to be bumped up by the appropriate length.
      next_datum += specifier;
    }

    // *next_datum* is now pointing at the next packet index.
    ndx = next_datum;
  }
}
