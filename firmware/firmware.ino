
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <EEPROM.h>

#include "sram_cache.h"

//Needed for sleep_mode
#include <avr/sleep.h>
//Needed for powering down perihperals such as the ADC/TWI and Timers
#include <avr/power.h>

///////////////////////////////////////////////////////////////
// I2C CONFIGURATION

#define MY_ADDRESS   0x2A  // Any number from 0x01 to 0x7F
#define FALSE           0
#define TRUE            1
#define BUFFSIZE        128
#define SPECSIZE        8

#define LED            13
#define I2C_CLOCK  100000

enum MachineState {
  LED_ON       = 0x00,
  LED_OFF      = 0x01,
  LED_MANY     = 0x02,

  DATA     = 0x10,

  FLUSH_CACHE_TO_SD = 0x30,
  ERASE_CACHE = 0x31,
  RESET_SEQUENCE_NUMBER = 0x3F,


  FROM_I2C     = 0xBE,
};

byte i2c_buffer[BUFFSIZE];
byte specifiers[SPECSIZE];
byte data[BUFFSIZE];
bool has_message = false;
unsigned long sequence_number = 0;

///////////////////////////////////////////////////////////////
// MICROSD CONFIGURATION

//On OpenLog this is pin 10
#define SD_CHIP_SELECT 10
//This is the name for the file when you're in sequential mode
#define SEQ_FILENAME "LOBSTER.TXT"

SdFat sd;


//Blinking LED error codes
#define ERROR_SD_INIT      3
#define ERROR_NEW_BAUD    5
#define ERROR_CARD_INIT   6
#define ERROR_VOLUME_INIT 7
#define ERROR_ROOT_INIT   8
#define ERROR_FILE_OPEN   9


// // // // // // // // //
// SETUP
void setup() {
  Wire.begin(MY_ADDRESS);
  Wire.setClock(I2C_CLOCK);

  Wire.onRequest(returnData);
  Wire.onReceive(incomingCommand);

  Serial.begin(115200);
  bool clean_start = true;

  for (int ndx ; ndx < BUFFSIZE ; ndx++) {
    i2c_buffer[ndx] = 0x00;
  }

  //Setup SD & FAT
  if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
    systemError(ERROR_CARD_INIT);
    clean_start = false;
  }

  //Change to root directory. All new file creation will be in root.
  if (!sd.chdir()) {
    systemError(ERROR_ROOT_INIT);
    clean_start = false;
  }

  if (!sram_init()) {
    clean_start = false;
  }

  if (clean_start) {
    blinkN(3, 100);
    Serial.println("CLEAN START");
    init_card();
  } else {
    blinkN(10, 100);
  }

}

// // // // // // // // //
// LOOP
void loop() {
  if (has_message) {
    has_message = false;
    interpret();
  }
}

bool interpret() {
  int received = i2c_buffer[0];
  byte cmd = i2c_buffer[1];

  switch (cmd) {
    case LED_ON:
      digitalWrite(LED, HIGH);
      break;

    case LED_OFF:
      digitalWrite(LED, LOW);
      break;

    case LED_MANY:
      {
        byte count = i2c_buffer[2];
        byte dur   = i2c_buffer[3];
        blinkN(count, dur);
        break;
      }

    case DATA:
      {
        // Increment the sequence number.
        // This always happens. If a packet fails the CRC,
        // or some other failure happens, then the sequence number
        // will skip.
        sequence_number += 1;

        // I send in:
        // [byte] The number of length specifiers, N.
        // [... N] Length specifiers.
        // [... N*] Values of the lengths specified.
        // [CRC] A single-byte CRC (sum % 128)

        // [0] is the total bytes received.
        // [1] is the command.
        // [2] is the number of length specifiers.
        // The last value is the CRC.
        int pkt_length = i2c_buffer[2];

        // The length value is part of the CRC.
        int crc = pkt_length;

        // Read the specifiers into their own buffer.
        // The specifiers are part of the CRC.
        for (int ndx = 0 ; ndx < pkt_length ; ndx++) {
          specifiers[ndx] = i2c_buffer[ndx + 3];
          crc += specifiers[ndx];
        }

        // Find out how many bytes of actual data need to be read.
        // Do this by adding up the length specifiers.
        int msg_length = 0;
        for (int ndx = 0 ; ndx < pkt_length ; ndx++) {
          msg_length += specifiers[ndx];
        }

        // Buffer the data.
        // All of the data is part of the CRC.
        for (int ndx = 0 ; ndx < msg_length ; ndx++) {
          byte v = i2c_buffer[ndx + 3 + pkt_length];
          // Serial.println("V: " + String(v));
          data[ndx] = v;
          crc = crc + v;
        }

        // Finally, grab the CRC that was transmitted to me.
        int received_crc = i2c_buffer[3 + msg_length + pkt_length];
        int loc_crc = crc % 128;
        // Serial.println("LOC CRC: " + String(loc_crc) + " RCV: " + String(received_crc));
        if (loc_crc == received_crc) {
          // Now, cache the data to the SRAM.

          // If I'm about to walk off the end of the cache, then I should
          // flush the cache first. After that, re-initialize the SRAM.
          // At the least, this should reset the pointer.
          if (!sram_has_space(pkt_length + msg_length + 1)) {
            flush_cache_to_sd();
            sram_init();
          }
          // Now, I know I have space for this data.

          cache_data(specifiers, pkt_length, data, msg_length);
        } else {
          // Serial.println("CX");
        }

        break;
      }

    case FLUSH_CACHE_TO_SD:
      // Serial.println("FLUSHING CACHE TO SD");
      // Here I would need to copy from flash memory to the SD card.
      flush_cache_to_sd();
      break;

    case ERASE_CACHE:
      // Serial.println("ERASING CACHE");
      sram_init();
      break;

    case RESET_SEQUENCE_NUMBER:
      {
        sequence_number = 0;
        break;
      }

    default:
      // blinkN(3, 100);
      Serial.println("X");
      break;
  }
}

void cache_data(byte *specifiers, int num_specifiers, byte *data, int num_bytes) {
  // Here I would need to store to the flash memory.
  // Serial.println(sequence_number);
  sram_cache_data(specifiers, num_specifiers, data, num_bytes);

}

void returnData() {
  byte arr[2];
  arr[0] = 0xBE;
  arr[1] = 0xEF;
  Wire.write(arr, 2);
}

void incomingCommand (int bytes_received) {
  has_message = true;
  i2c_buffer[0] = bytes_received;

  for (int ndx = 0; ndx < bytes_received ; ndx++) {
    i2c_buffer[ndx + 1] = Wire.read();
  }
}

void blinkN (int n, int dur) {
  pinMode(LED, OUTPUT);
  for (int c = 0 ; c < n ; c++) {
    digitalWrite(LED, HIGH);
    delay(dur);
    digitalWrite(LED, LOW);
    delay(dur);
  }
}

///////////////////////////////////////////////////////////////
// OPENLOG THEFT
// This is where I grab and hack code from OpenLog.
// I always write to the same file.

void init_card(void)
{
  SdFile the_file;

  // Try to create sequential file
  if (!the_file.open(SEQ_FILENAME, O_CREAT | O_WRITE))
  {
    // The file could not be created. Something must be wrong.
    Serial.println(F("Error creating SEQLOG"));
  } else {
    // Immediately close the file. This makes sure it exists.
    the_file.close();
  }
}

//Log to the same file every time the system boots, sequentially
//Checks to see if the file SEQLOG.txt is available
//If not, create it
//If yes, append to it
//Return 0 on error
//Return anything else on success
void flush_cache_to_sd(void)
{
  SdFile the_file;

  // Try to create sequential file
  if (!the_file.open(SEQ_FILENAME, O_CREAT | O_WRITE))
  {
    // The file could not be created. Something must be wrong.
    Serial.println(F("Error creating SEQLOG"));
  } else {
    // Immediately close the file. This makes sure it exists.
    the_file.close();
    append_cache_to_datafile();
  }
}

//This is the most important function of the device. These loops have been tweaked as much as possible.
//Modifying this loop may negatively affect how well the device can record at high baud rates.
//Appends a stream of serial data to a given file
//Does not exit until escape character is received the correct number of times
//Returns 0 on error
//Returns 1 on success
byte append_cache_to_datafile() {
  SdFile the_file;

  // O_CREAT - create the file if it does not exist
  // O_APPEND - seek to the end of the file prior to each write
  // O_WRITE - open for write
  if (!the_file.open(SEQ_FILENAME, O_CREAT | O_APPEND | O_WRITE)) {
    systemError(ERROR_FILE_OPEN);
  }

  // This is a trick to make sure first cluster is allocated
  // Found in Bill's example/beta code (?)
  if (the_file.fileSize() == 0) {
    the_file.rewind();
    the_file.sync();
  }

  sram_copy_data(&the_file);

  the_file.sync(); //Sync the card before we go to sleep

  // power_timer0_disable(); //Shut down peripherals we don't need
  // power_spi_disable();

  //Driving SPI pins low before sleep to attempt to lower microSD card stand-by current
  //Pins: 10, 11, 12, 13
  for (byte x = 10 ; x < 14 ; x++)
  {
    pinMode(x, OUTPUT);
    digitalWrite(x, LOW);
  }

  // sleep_mode(); //Stop everything and go to sleep. Wake up if serial character received

  // power_spi_enable(); //After wake up, power up peripherals
  // power_timer0_enable();

  the_file.close(); // Done recording, close out the file

  Serial.println(F("~")); // Indicate a successful record

  return (1); //Success!
}


/*

  #define ERROR_SD_INIT      3
  #define ERROR_NEW_BAUD    5
  #define ERROR_CARD_INIT   6
  #define ERROR_VOLUME_INIT 7
  #define ERROR_ROOT_INIT   8
  #define ERROR_FILE_OPEN   9
*/
void systemError(byte errorType) {
  switch (errorType) {
    case ERROR_SD_INIT:
      Serial.println(F("ERROR_SD_INIT"));
      break;
    case ERROR_NEW_BAUD:
      Serial.println(F("ERROR_NEW_BAUD"));
      break;
    case ERROR_CARD_INIT:
      Serial.println(F("ERROR_CARD_INIT"));
      break;
    case ERROR_VOLUME_INIT:
      Serial.println(F("ERROR_VOLUME_INIT"));
      break;
    case ERROR_ROOT_INIT:
      Serial.println(F("ERROR_ROOT_INIT"));
      break;
    case ERROR_FILE_OPEN:
      Serial.println(F("ERROR_FILE_OPEN"));
      break;
    default:
      Serial.println(F("UNKNOWN ERROR"));
      break;
  }
}
