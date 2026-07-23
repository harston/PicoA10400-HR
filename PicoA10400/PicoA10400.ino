/*
//            PICO A10400 an Atari 2600+7800 MultiCART by Andrea Ottaviani
// Atari 2600 / Atari 7800   multicart based on Raspberry Pico board -

// v. 0.1 2024-10-03 : Initial version for Pi Pico 
//
// 
//  More info on https://github.com/aotta/ 
*/

#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "string.h"
#include "rom.h"
//#include "menu7800.h"

// include for Flash files
#include "SPI.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
#include "supercharger_bios.h"
Adafruit_FlashTransport_RP2040 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;
FatFile root;
FatFile file;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
// Check if flash is formatted
bool fs_formatted;
// Set to true when PC write to flash
bool fs_changed;

#define a7800    0   // 1 for 7800 menu, 0 for 2600

// Pico pin usage definitions

#define A0_PIN    0
#define A1_PIN    1
#define A2_PIN    2
#define A3_PIN    3
#define A4_PIN    4
#define A5_PIN    5
#define A6_PIN    6
#define A7_PIN    7
#define A8_PIN    8
#define A9_PIN    9
#define A10_PIN  10
#define A11_PIN  11
#define A12_PIN  12
#define A13_PIN  13
#define A14_PIN  14
#define A15_PIN  15
#define D0_PIN   16
#define D1_PIN   17
#define D2_PIN   18
#define D3_PIN   19
#define D4_PIN   20
#define D5_PIN   21
#define D6_PIN   22
#define D7_PIN   23
#define HALT_PIN 24
#define RW_PIN   26
#define CLK_PIN  27
#define IRQ_PIN  28

// Pico pin usage masks
#define A0_PIN_MASK     0x00000001L //gpio 0
#define A1_PIN_MASK     0x00000002L
#define A2_PIN_MASK     0x00000004L
#define A3_PIN_MASK     0x00000008L
#define A4_PIN_MASK     0x00000010L
#define A5_PIN_MASK     0x00000020L
#define A6_PIN_MASK     0x00000040L
#define A7_PIN_MASK     0x00000080L
#define A8_PIN_MASK     0x00000100L
#define A9_PIN_MASK     0x00000200L
#define A10_PIN_MASK    0x00000400L
#define A11_PIN_MASK    0x00000800L
#define A12_PIN_MASK    0x00001000L  //  
#define A13_PIN_MASK    0x00002000L  //  
#define A14_PIN_MASK    0x00004000L  //  
#define A15_PIN_MASK    0x00008000L  //  
#define D0_PIN_MASK     0x00010000L
#define D1_PIN_MASK     0x00020000L
#define D2_PIN_MASK     0x00040000L
#define D3_PIN_MASK     0x00080000L
#define D4_PIN_MASK     0x00100000L  // gpio 20
#define D5_PIN_MASK     0x00200000L
#define D6_PIN_MASK     0x00400000L
#define D7_PIN_MASK     0x00800000L //gpio 23
#define HALT_PIN_MASK   0x01000000L //gpio 24
#define LED_PIN_MASK    0x02000000L //gpio 25
#define RW_PIN_MASK     0x04000000L //gpio 26
#define CLK_PIN_MASK    0x08000000L //gpio 27
#define IRQ_PIN_MASK    0x10000000L //gpio 28


// Aggregate Pico pin usage masks
#define ALL_GPIO_MASK  	0x1FFFFFFFL
#define BUS_PIN_MASK    0x0000FFFFL
#define BUS_H_PIN_MASK  0x0000E000L
#define DATA_PIN_MASK   0x00FF0000L
#define STATUS_PIN_MASK 0x1D000000L
#define READ_PIN_MASK    (A14_PIN_MASK | A15_PIN_MASK | RW_PIN_MASK)  //gpio 27

#define ALWAYS_IN_MASK  (BUS_PIN_MASK | STATUS_PIN_MASK)
#define ALWAYS_OUT_MASK (DATA_PIN_MASK)

#define SET_DATA_MODE_OUT   gpio_set_dir_out_masked(DATA_PIN_MASK)
#define SET_DATA_MODE_IN    gpio_set_dir_in_masked(DATA_PIN_MASK)

#define SET_LED_ON    	gpio_init(25);gpio_set_dir(25,GPIO_OUT);gpio_put(25,true);
#define SET_LED_OFF    	gpio_init(25);gpio_set_dir(25,GPIO_OUT);gpio_put(25,false);



#define CART_TYPE_NONE	0
#define CART_TYPE_2K	1
#define CART_TYPE_4K	2
#define CART_TYPE_F8	3	// 8k
#define CART_TYPE_F6	4	// 16k
#define CART_TYPE_F4	5	// 32k
#define CART_TYPE_F8SC	6	// 8k+ram
#define CART_TYPE_F6SC	7	// 16k+ram
#define CART_TYPE_F4SC	8	// 32k+ram
#define CART_TYPE_FE	9	// 8k
#define CART_TYPE_3F	10	// varies (examples 8k)
#define CART_TYPE_3E	11	// varies (only example 32k)
#define CART_TYPE_3EX   12
#define CART_TYPE_E0	13	// 8k
#define CART_TYPE_0840	14	// 8k
#define CART_TYPE_CV	15	// 2k+ram
#define CART_TYPE_EF	16	// 64k
#define CART_TYPE_EFSC	17	// 64k+ram
#define CART_TYPE_F0	18	// 64k
#define CART_TYPE_FA	19	// 12k
#define CART_TYPE_E7	20	// 16k+ram
#define CART_TYPE_DPC	21	// 8k+DPC(2k)
#define CART_TYPE_AR	22  // Arcadia Supercharger (variable size)
#define CART_TYPE_BF	23  // BF
#define CART_TYPE_BFSC	24  // BFSC
#define CART_TYPE_ACE	25  // ARM Custom Executable
#define CART_TYPE_PP    26  // Pink Panther Prototype
#define CART_TYPE_DF    27  // DF
#define CART_TYPE_DFSC  28  // DFSC
#define CART_TYPE_3EP	29	// 3E+ 1-64K + 32K ram
#define CART_TYPE_4KSC  30  // 4k+SC
#define CART_TYPE_FA2	31	// 28k
#define CART_TYPE_A78	32	// Atari 7800
#define CART_TYPE_NORMALA78	33	// Atari 7800
#define CART_TYPE_ABSOLUTE	34	// Atari 7800
#define CART_TYPE_ACTIVISION	35	// Atari 7800
#define CART_TYPE_SUPERCART_EF	36	// Atari 7800 supercart EX-FIX
#define CART_TYPE_SUPERCART_ROM	37	// Atari 7800
#define CART_TYPE_SUPERCART_RAM	38	// Atari 7800
#define CART_TYPE_SUPERCART_LARGE	39	// Atari 7800
#define CART_TYPE_SUPERCART	40	// Atari 7800 supercart bs

#define CCM_RAM ((uint8_t*)0x10000000)
#define CCM_SIZE (64 * 1024)

#define RAM_BANKS 48
#define CCM_BANKS 32

#define MAX_RAM_BANK (RAM_BANKS - 1)
#define MAX_CCM_BANK (MAX_RAM_BANK + CCM_BANKS)

typedef struct __attribute__((packed)) {
	uint8_t entry_lo;
	uint8_t entry_hi;
	uint8_t control_word;
	uint8_t block_count;
	uint8_t checksum;
	uint8_t multiload_id;
	uint8_t progress_bar_speed_lo;
	uint8_t progress_bar_speed_hi;
	uint8_t padding[8];
	uint8_t block_location[48];
	uint8_t block_checksum[48];
} LoadHeader;

typedef struct __attribute__((packed)) {
	uint8_t magic_number[8]; // Always ascii "ACE-2600"
	uint8_t driver_name[16]; // emulators care about this
	uint32_t driver_version; // emulators care about this
	uint32_t rom_size;		 // size of ROM to be copied to flash, 448KB max
	uint32_t rom_checksum;	 // used to verify if flash already contains valid image
	uint32_t entry_point;	 // where to begin executing
} ACEFileHeader;

typedef struct {
	const char *ext;
	int cart_type;
} EXT_TO_CART_TYPE_MAP;

EXT_TO_CART_TYPE_MAP ext_to_cart_type_map[] = {
	{"ROM", CART_TYPE_NONE},
	{"BIN", CART_TYPE_NONE},
	{"A26", CART_TYPE_NONE},
	{"2K", CART_TYPE_2K},
	{"4K", CART_TYPE_4K},
	{"F8", CART_TYPE_F8},
	{"F6", CART_TYPE_F6},
	{"F4", CART_TYPE_F4},
	{"F8S", CART_TYPE_F8SC},
	{"F6S", CART_TYPE_F6SC},
	{"F4S", CART_TYPE_F4SC},
	{"FE", CART_TYPE_FE},
	{"3F", CART_TYPE_3F},
	{"3E", CART_TYPE_3E},
	{"3EX", CART_TYPE_3E},
	{"3EP", CART_TYPE_3EP},
	{"E0", CART_TYPE_E0},
	{"084", CART_TYPE_0840},
	{"CV", CART_TYPE_CV},
	{"EF", CART_TYPE_EF},
	{"EFS", CART_TYPE_EFSC},
	{"F0", CART_TYPE_F0},
	{"FA", CART_TYPE_FA},
	{"E7", CART_TYPE_E7},
	{"DPC", CART_TYPE_DPC},
	{"AR", CART_TYPE_AR},
	{"BF", CART_TYPE_BF},
	{"BFS", CART_TYPE_BFSC},
	{"ACE", CART_TYPE_ACE},
	{"WD", CART_TYPE_PP},
	{"DF", CART_TYPE_DF},
	{"DFS", CART_TYPE_DFSC},
	{"4KSC", CART_TYPE_4KSC},
  {"FA2", CART_TYPE_FA2},
  {"A78", CART_TYPE_A78},
	{0,0}
};

//#define rom_table_SIZE			128  // kilobytes
unsigned int cart_size_bytes;

//unsigned char rom_table[32*1024];
char menu_status[16];
uint8_t AR_ROM[8448*4];
char filelist[85*48]; // 85 name of 48 chars max lenght
char direntry_isdir[85]; // 1 if filelist[n] is a directory, 0 if a regular file (".." counts as 0: no highlight, not sorted)
char direntry_toobig[85]; // 1 if filelist[n] is a file larger than rom_table: loaded truncated, shown red in the menu
#define MENU_FOOTER_TEXT "AOTTAv01 HR2" // 12 chars: the menu kernel renders exactly 12 per row
// Colour of oversized-ROM names. The kernel reads this at runtime from menu_status[12],
// so changing it needs no ROM patch - just this line. $66 was picked by sweeping all 16
// hues on the actual PAL TV: hue 6 is the red family here, and luminance 6 keeps it
// saturated (luminance A washed out to near-white). More saturated: $64. Brighter: $68.
#define OVERSIZED_COLOUR 0x66

// Marquee: the highlighted entry scrolls when its name is longer than the 12 columns
// a row can show. Only that row moves - moving the cursor away restores the plain
// beginning of the name. The cursor position is not visible to us on its own; the
// menu kernel reports it once per frame by reading CART_CMD_CURSOR_n + row.
#define MARQUEE_STEP_MS 280 // time per one-character step
#define MARQUEE_HOLD    5   // steps held still at each end, so both ends stay readable
volatile int cursor_row=0;  // row the Atari is highlighting
int menu_count=0;           // entries currently in menu_ram (guards the row index)
int marquee_row=-1;         // row being scrolled, -1 = none
int marquee_tick=0;
uint32_t marquee_last=0;
uint8_t ram_table[32*1024];
char path[128];
 char filetoopen[200]; // must hold path[128] + filename (up to 47 chars) + terminator; was 50, which overflowed with long names or subdirectories
 
char menu_ram[1024];	// < NUM_DIR_ITEMS * 12 (85 max)
char isfor7800=0;
char is16k=0;
int romsize;
int lastpos;
volatile u_int8_t bank_type=1;
volatile u_int8_t new_bank_type=1;
volatile int romLen=0;
u_int8_t gamechoosen=0;
volatile u_int8_t newgame=0;
volatile u_int8_t rootdir=0;
volatile uint32_t addrc;
volatile uint32_t retaddr;
volatile u_int8_t cart_to_emulate;
  
////////////////////////////////////////////////////////////////////////////////////
//                     REBOOT
////////////////////////////////////////////////////////////////////////////////////
void doReboot() {
  rp2040.reboot();
}
////////////////////////////////////////////////////////////////////////////////////
//                     EXIT CARTRIDGE
////////////////////////////////////////////////////////////////////////////////////

void exit_cartridge(uint32_t addr, uint32_t addr_prev){
         
  gpio_put_masked(DATA_PIN_MASK,0xEA<<D0_PIN);    // (NOP) or data for SWCHB
	SET_DATA_MODE_OUT;
	while ((gpio_get_all()&BUS_PIN_MASK) == addr);

	addr = gpio_get_all()&BUS_PIN_MASK;
	 gpio_put_masked(DATA_PIN_MASK,0x00<<D0_PIN); // (BRK)
  while ((gpio_get_all()&BUS_PIN_MASK) == addr);
}
////////////////////////////////////////////////////////////////////////////////////
//                    reboot CARTRIDGE
////////////////////////////////////////////////////////////////////////////////////

void reboot_cartridge(uint32_t addr, uint32_t addr_prev){
retry:
  while(!(addr=gpio_get_all()&BUS_PIN_MASK) & 0x1000);
  gpio_put_masked(DATA_PIN_MASK,0x6c<<D0_PIN);    // (NOP) or data for SWCHB
	SET_DATA_MODE_OUT;
	while ((gpio_get_all()&BUS_PIN_MASK) == addr);
  
  addr = gpio_get_all()&BUS_PIN_MASK;
	 gpio_put_masked(DATA_PIN_MASK,0xfc<<D0_PIN); // (BRK)
 
  while ((gpio_get_all()&BUS_PIN_MASK) == addr);
   
   gpio_put_masked(DATA_PIN_MASK,0xff<<D0_PIN); // (BRK)
  while ((gpio_get_all()&BUS_PIN_MASK) == addr);
  //if ((gpio_get_all()&BUS_PIN_MASK) != 0xfffc) goto retry;
}

////////////////////////////////////////////////////////////////////////////
// Activision bankswitch - the only two games are Double Dragon and Rampage.
// 128K image, 8 x 16K banks:
//   $4000-$7FFF  bank 6
//   $8000-$9FFF  one 8K half of bank 7
//   $A000-$DFFF  switchable bank; selected by a write at or above $E000, the
//                bank number coming from A2-A0 of that address
//   $E000-$FFFF  the other 8K half of bank 7
//
// A13 IS INVERTED relative to the map that MAME's a78_rom_act_device::read_40xx
// describes: inside every window the two 8K halves trade places. That is not a
// guess - it was read out of the game images and then confirmed on hardware
// (full trail in not_working_roms3/experiment_activision/README.md):
//   * $E000-$FFFF must serve file 0x1C000. Only then do the $FF80-$FFF7
//     signature block and the $FFF8/$FFF9 bytes land where the 7800 BIOS looks
//     for them; with 0x1E000 the console reads 00/FF there, decides no cartridge
//     is present and starts its built-in game.
//   * Double Dragon's reset vector $FF74 does JMP $448D, and $448D holds real
//     startup code - "LDA #$00 / STA $FF80 / JMP $DC00", i.e. select bank 0 and
//     jump into it - only at file 0x1A48D, which is bank 6 with A13 flipped.
//     Straight bank 6 gives 0x1848D, all zeros.
//   * That JMP $DC00 in turn needs bank 0 with A13 flipped (file 0x03C00, a
//     table of JMPs); without the flip it lands on 0x01C00, again all zeros.
// The call in setup1() used to be commented out with "doesn't work"; with this
// mapping Double Dragon (PAL) runs correctly on real hardware.
#define ACT_BANK7_AT_E000 0x1C000   // file half served at $E000-$FFFF
#define ACT_BANK7_AT_8000 0x1E000   // file half served at $8000-$9FFF
#define ACT_A13           0x2000    // A13 flip applied inside the 16K windows

// Shaped like emulate_supercart_ef(): one gpio_get_all() per pass, bit tests
// instead of range compares, and the data lines left driven (rom_in_use) rather
// than a SET_DATA_MODE_OUT / wait / SET_DATA_MODE_IN dance on every access. The
// original loop did all three the slow way and could not keep up with MARIA once
// a game started pulling graphics - the picture died the moment gameplay began.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_activision()) {
      uint32_t bank=0, addr=0, rawaddr=0;
      uint8_t rom_in_use=1;

      while (1) {
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        if (addr & A15_PIN_MASK) {
            if (addr & A14_PIN_MASK) {
                if (addr & A13_PIN_MASK) {                 // $E000-$FFFF: fixed half of bank 7
                    sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x1fff) + ACT_BANK7_AT_E000] << D0_PIN;
                    rawaddr = gpio_get_all() & READ_PIN_MASK;
                    if (rawaddr == READ_PIN_MASK) {
                        if (!rom_in_use) {
                            SET_DATA_MODE_OUT;
                            rom_in_use = 1;
                        }
                    } else {
                        // Bank select - but only after a second look confirms it.
                        // Anything that makes the read pattern fail to match,
                        // above all the address simply changing between the two
                        // samples, would otherwise be taken for a write and move
                        // the bank at random. Not theory: without this re-check
                        // both games banked away from their own code mid-frame
                        // (Double Dragon went black, Rampage showed only noise).
                        rawaddr = gpio_get_all();
                        if ((rawaddr & (RW_PIN_MASK | A15_PIN_MASK | A14_PIN_MASK | A13_PIN_MASK))
                              == (A15_PIN_MASK | A14_PIN_MASK | A13_PIN_MASK)) {
                            SET_DATA_MODE_IN;
                            rom_in_use = 0;
                            bank = (rawaddr & 7) * 0x4000;
                        }
                    }
                } else {                                   // $C000-$DFFF: switchable bank
                    sio_hw->gpio_out = (uint32_t)rom_table[((addr & 0x3fff) ^ ACT_A13) + bank] << D0_PIN;
                    rawaddr = gpio_get_all() & READ_PIN_MASK;
                    if (rawaddr == READ_PIN_MASK) {
                        if (!rom_in_use) {
                            SET_DATA_MODE_OUT;
                            rom_in_use = 1;
                        }
                    }
                }
            } else {
                if (addr & A13_PIN_MASK) {                 // $A000-$BFFF: switchable bank
                    sio_hw->gpio_out = (uint32_t)rom_table[((addr & 0x3fff) ^ ACT_A13) + bank] << D0_PIN;
                } else {                                   // $8000-$9FFF: fixed half of bank 7
                    sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x1fff) + ACT_BANK7_AT_8000] << D0_PIN;
                }
                rawaddr = gpio_get_all() & READ_PIN_MASK;
                if (rawaddr == (RW_PIN_MASK | A15_PIN_MASK)) {
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                }
            }
        } else {
            if (addr & A14_PIN_MASK) {                     // $4000-$7FFF: bank 6
                sio_hw->gpio_out = (uint32_t)rom_table[((addr & 0x3fff) ^ ACT_A13) + 0x18000] << D0_PIN;
                rawaddr = gpio_get_all() & (RW_PIN_MASK | A14_PIN_MASK);
                if (rawaddr == (RW_PIN_MASK | A14_PIN_MASK)) {
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                } else {
                    if (rom_in_use) {
                        SET_DATA_MODE_IN;
                        rom_in_use = 0;
                    }
                }
            } else {
                if (rom_in_use) {
                    SET_DATA_MODE_IN;
                    rom_in_use = 0;
                }
            }
        }
      }
}

void __time_critical_func(emulate_supercart_ef()) {
      uint32_t bank=0, addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      
      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (addr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                gpio_put_masked(DATA_PIN_MASK, rom_table[addr + 0x10000] << D0_PIN);
                rawaddr = gpio_get_all() & READ_PIN_MASK;
	          if (rawaddr == READ_PIN_MASK) {
                    // Read cycle
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                }
            } else {
                // Set the data on the bus for active bank
                gpio_put_masked(DATA_PIN_MASK, rom_table[(addr & 0x3fff) + bank] << D0_PIN);
                // Check for RW
                rawaddr = gpio_get_all() & READ_PIN_MASK;
	          if (rawaddr == (RW_PIN_MASK | A15_PIN_MASK)) {  // READ ROM
                    // Read cycle
                    if (!rom_in_use) {
                       SET_DATA_MODE_OUT;
                       rom_in_use = 1;
                    }
                } else {  // Write cycle to ROM
                   // rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    // Check for bankswitch
                    if (rawaddr == A15_PIN_MASK) {
                        // Bankswitching write
                        SET_DATA_MODE_IN;
                        // Check for 0x01
                        rawaddr = gpio_get_all();
                        bank=((rawaddr >> D0_PIN) & 0xf)*0x4000;
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            // EXFIX - bank 6 is in 0x4000
            if (addr & 0x4000) {
                gpio_put_masked(DATA_PIN_MASK, rom_table[(addr & 0x3fff) + 0x18000] << D0_PIN);
                rawaddr = gpio_get_all() & (RW_PIN_MASK | A14_PIN_MASK);
	        if (rawaddr == (RW_PIN_MASK | A14_PIN_MASK)) {
                    // Read cycle
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                } else {
                    if (rom_in_use) {
                        SET_DATA_MODE_IN;
                        rom_in_use = 0;
                    }
                }
            } else {
                if (rom_in_use) {
                    SET_DATA_MODE_IN;
                    rom_in_use = 0;
                }
            }
        }
      }
    }

void __time_critical_func(emulate_supercart_ram()) {
      uint32_t bank=0;
      uint32_t addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      
      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (addr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                gpio_put_masked(DATA_PIN_MASK, rom_table[(addr & 0x7fff) + romLen - 0x8000 ] << D0_PIN);
                rawaddr = gpio_get_all() & READ_PIN_MASK;
	          if (rawaddr == READ_PIN_MASK) {
                    // Read cycle
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                }
            } else {
                // Set the data on the bus for active bank
                //gpio_put_masked(DATA_PIN_MASK, rom_table[(addr & 0x3fff) + bank] << D0_PIN);
                gpio_put_masked(DATA_PIN_MASK, rom_table[(addr & 0x7fff) + bank] << D0_PIN);
                // Check for RW
                rawaddr = gpio_get_all() & READ_PIN_MASK;
	          if (rawaddr == (RW_PIN_MASK | A15_PIN_MASK)) {  // READ ROM
                    // Read cycle
                    if (!rom_in_use) {
                       SET_DATA_MODE_OUT;
                       rom_in_use = 1;
                    }
                } else {  // Write cycle to ROM
                   // rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    // Check for bankswitch
                    if (rawaddr == A15_PIN_MASK) {
                        // Bankswitching write
                        SET_DATA_MODE_IN;
                        // Check for 0x01
                        rawaddr = gpio_get_all();
                        bank=((rawaddr >> D0_PIN) & 0xf)*0x4000;
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            rawaddr=gpio_get_all();
            // EXram - 16k is in 0x4000
            if (rawaddr & 0x4000) {
                addr= rawaddr & 0x3fff;
                gpio_put_masked(DATA_PIN_MASK, ram_table[addr] << D0_PIN);
                rawaddr = gpio_get_all() & (RW_PIN_MASK | A14_PIN_MASK);
	        if (rawaddr == (RW_PIN_MASK | A14_PIN_MASK)) {
                    // Read cycle
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                } else {
                  if (rawaddr == A14_PIN_MASK) {
                     // Write cycle
                        SET_DATA_MODE_IN;
                        rawaddr = gpio_get_all();
                        ram_table[rawaddr & 0x3fff] = (rawaddr >> D0_PIN) & 0xff;
                        rom_in_use = 0;
                  } else {
                    if (rom_in_use) {
                        SET_DATA_MODE_IN;
                        rom_in_use = 0;
                    }
                  }
                }
            } else {
                if (rom_in_use) {
                    SET_DATA_MODE_IN;
                    rom_in_use = 0;
                }
            }
        }
      }
}

// Extracted from the inline switch case it used to be (see patches/PicoA10400_0.09.txt).
// Pure extraction + speed: behaviour is unchanged from before, only where the code lives
// and how fast it answers the bus changed. Compiled at -O2 rather than the sketch
// default -Os: at -Os the SDK's static-inline gpio_put_masked() gets outlined into a
// copy in FLASH, called through a RAM veneer on every single bus response (~25 cycles
// plus XIP-cache-miss jitter - see PicoA10400_tune/README.md for the measured evidence).
// Each data output below is a single SIO store instead, safe here because in 7800
// modes the only output pins are D0-D7, so the other OUT-latch bits are don't-cares.
// Verify any future change to this function with PicoA10400_tune/tools/check_hotpath.sh
// - it must keep reporting zero calls leaving RAM.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_large()) {
      // bank is a byte OFFSET into rom_table for the $8000-$BFFF window (not a bank
      // number): bank=0 means file bank 0 - the same 16KB already visible at
      // $4000-$7FFF - is what a real 9-bank SuperGame cart shows at $8000 before its
      // first bank-select write (MAME's a78 sg9 device: device_reset() { m_bank=0; }).
      uint32_t bank=0, addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;

      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (addr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                sio_hw->gpio_out = (uint32_t)rom_table[addr + 0x14000] << D0_PIN;
                rawaddr = gpio_get_all() & READ_PIN_MASK;
	          if (rawaddr == READ_PIN_MASK) {
                    // Read cycle
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                }
            } else {
                // Set the data on the bus for active bank
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + bank] << D0_PIN;
                // Check for RW
                rawaddr = gpio_get_all() & READ_PIN_MASK;
	          if (rawaddr == (RW_PIN_MASK | A15_PIN_MASK)) {  // READ ROM
                    // Read cycle
                    if (!rom_in_use) {
                       SET_DATA_MODE_OUT;
                       rom_in_use = 1;
                    }
                } else {  // Write cycle to ROM
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    // Check for bankswitch
                    if (rawaddr == A15_PIN_MASK) {
                        // Bankswitching write
                        SET_DATA_MODE_IN;
                        rawaddr = gpio_get_all();
                        // Mask 7, not 0xF: MAME computes bank_mask=7 for a 9-bank (144KB)
                        // image and wraps the written value against it. With 0xF a stray
                        // write of 8..15 would select "file banks 9..16", i.e. read past
                        // the end of rom_table into unrelated RAM - Alien Brigade writes
                        // values it loaded from memory here, not only the immediates
                        // 2..5 seen in its startup code, so out-of-range values cannot be
                        // ruled out. +1: file bank 0 is already shown at $4000.
                        bank = (((rawaddr >> D0_PIN) & 0x7) + 1) * 0x4000;
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            // EXROM - first 16k at 0x4000
            if (addr & 0x4000) {
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) ] << D0_PIN;
                rawaddr = gpio_get_all() & (RW_PIN_MASK | A14_PIN_MASK);
	        if (rawaddr == (RW_PIN_MASK | A14_PIN_MASK)) {
                    // Read cycle
                    if (!rom_in_use) {
                        SET_DATA_MODE_OUT;
                        rom_in_use = 1;
                    }
                } else {
                    if (rom_in_use) {
                        SET_DATA_MODE_IN;
                        rom_in_use = 0;
                    }
                }
            } else {
                if (rom_in_use) {
                    SET_DATA_MODE_IN;
                    rom_in_use = 0;
                }
            }
        }
      }
}

void __time_critical_func(emulate_supercharger_cartridge())  {
  uint8_t* buffer=rom_table;
  unsigned int image_size;
 	uint8_t *ram = buffer;
	uint8_t *rom = ram + 0x1800;
	uint8_t *multiload_map = rom + 0x0800;
	uint8_t *multiload_buffer = multiload_map + 0x0100;
	uint32_t addr = 0, addr_prev = 0, addr_prev2 = 0, last_address = 0;
  uint8_t data_prev = 0, data = 0;
	uint8_t *bank0 = ram, *bank1 = rom;
	uint32_t transition_count = 0;
	bool write_ram_enabled = false;
	uint8_t data_hold = 0;
	uint32_t multiload_count;
	uint8_t value_out;

  image_size=romLen;
  multiload_count = image_size / 8448;
  
  memset(ram, 0, 0x1800);
	setup_rom(rom);
  
  setup_multiload_map(multiload_map, multiload_count);

  while (1) {
		while (((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev) || (addr != addr_prev2))
		{
			addr_prev2 = addr_prev;
			addr_prev = addr;
		}
  
			if (addr & 0x1000){
			if (write_ram_enabled && transition_count == 5 && (addr < 0x1800 || bank1 != rom))
				value_out = data_hold;
			else
				value_out = addr < 0x1800 ? bank0[addr & 0x07ff] : bank1[addr & 0x07ff];

			gpio_put_masked(DATA_PIN_MASK,value_out<<D0_PIN);	
		
			SET_DATA_MODE_OUT;

			if (addr == 0x1ff9 && bank1 == rom && last_address <= 0xff) {
				SET_DATA_MODE_IN;

				while ((gpio_get_all()&BUS_PIN_MASK) == addr) { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }

				load_multiload(ram, rom, multiload_map[data_prev & 0xff], multiload_buffer);

			}
			else if ((addr & 0x0f00) == 0 && (transition_count > 5 || !write_ram_enabled)) {
				data_hold = (uint8_t) addr; // & 0xff;
				transition_count = 0;
			}
			else if (addr == 0x1ff8) {
				transition_count = 6;
				write_ram_enabled = data_hold & 0x02;
				switch ((data_hold & 0x1c) >> 2) {
					case 4:
					case 0:
						bank0 = ram + 2048 * 2;
						bank1 = rom;
						break;
					case 1:
						bank0 = ram;
						bank1 = rom;
						break;
					case 2:
						bank0 = ram + 2048 * 2;
						bank1 = ram;
						break;
					case 3:
						bank0 = ram;
						bank1 = ram + 2048 * 2;
						break;
					case 5:
						bank0 = ram + 2048;
						bank1 = rom;
						break;
					case 6:
						bank0 = ram + 2048 * 2;
						bank1 = ram + 2048;
						break;
					case 7:
						bank0 = ram + 2048;
						bank1 = ram + 2048 * 2;
						break;
					default:
						break;
				}
			}
			else if (write_ram_enabled && transition_count == 5) {
				if (addr < 0x1800)
					bank0[addr & 0x07ff] = data_hold;
				else if (bank1 != rom)
					bank1[addr & 0x07ff] = data_hold;
			}
		}

		if (transition_count < 6) transition_count++;

		last_address = addr;
		while ((gpio_get_all()&BUS_PIN_MASK) == addr);
		SET_DATA_MODE_IN;
	}
 }


////////////////////////////////////////////////////////////////////////////////////
//                     HANDLE BUS
////////////////////////////////////////////////////////////////////////////////////
#pragma GCC push_options
#pragma GCC optimize ("O3")

void __time_critical_func(setup1()) {   //HandleBUS()
	
  u_int8_t data, data_prev;
  uint32_t bank;
  uint8_t bankswitch;
  uint8_t rom_in_use;
 	uint32_t addr, addr_prev = 0, addr_prev2 = 0, rawaddr=0;
  int lastAccessWasFE = 0;
  unsigned char *bankPtr;
  uint8_t *fixedPtr;
  unsigned char *ram1Ptr;
  unsigned char *ram2Ptr;

  u_int16_t lowBS, highBS=0x1ff9;
  int isSC=0, cartPages, ram_mode;
  unsigned char curBanks[4];
  //------------------------------------------------------------------
  // atari->cart comms addresses
  //------------------------------------------------------------------
 
#define CART_CMD_SEL_ITEM_n	0x1E00
#define CART_CMD_CURSOR_n	0x1E80	// kernel reports the highlighted row here, once per frame
#define CART_CMD_ROOT_DIR	0x1EF0
#define CART_CMD_START_CART	0x1EFF
#define CART_STATUS_BYTES	0x1FE0	// 16 bytes of status

//	multicore_lockout_victim_init();	
    gpio_init_mask(ALL_GPIO_MASK);
    gpio_set_dir_in_masked(ALWAYS_IN_MASK);

// We require the menu to do a write to $1FF4 to unlock the comms area.
// This is because the 7800 bios accesses this area on console startup, and we wish to ignore these
// spurious reads until it has started the cartridge in 2600 mode.
//Serial.println("menu");
bool comms_enabled = false;

newgame=0; // poi rimetti 1
//exit_cartridge(0,0);

    //comms_enabled=true; // poi via?
 //  set_menu_status_byte(0);
start:
  while (newgame==0)
	{
		while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
   	// got a stable address
		if (addr & 0x1000) 	{ // A12 high
    	if (comms_enabled) 	{	// normal mode, once the cartridge code has done its init.
				// on a 7800, we know we are in 2600 mode now.
	      addrc= addr & 0x1fff;
        if ((addrc >= 0x1E00)&&(addrc<0x1F00)){
            if ((addrc >= CART_CMD_CURSOR_n) && (addrc < CART_CMD_ROOT_DIR)) {
              // Position report, not a command. It arrives every frame, so it must never
              // land in retaddr: the kernel resumes browsing about one frame after a
              // selection, and the next report would overwrite a command the main loop
              // had not polled yet - which silently swallowed selections such as "..".
              cursor_row = addrc - CART_CMD_CURSOR_n;
            } else {
              retaddr=addrc;	// atari 2600 has sent a command
              if (addr==CART_CMD_START_CART) newgame=1; //goto ballout;
            }
        } else if ((addrc >= 0x1800) && (addrc < 0x1C00)) {
				    	gpio_put_masked(DATA_PIN_MASK,menu_ram[addr&0x3FF]<<D0_PIN);
          } else if ((addr & 0x1FF0) == CART_STATUS_BYTES) {
					gpio_put_masked(DATA_PIN_MASK,menu_status[addr&0x0F]<<D0_PIN);
				} else {
      		gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0xFFF]<<D0_PIN);	
        }
      	SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				SET_DATA_MODE_IN;  
      } else {	// prior to an access to $1FF4, we might be running on a 7800 with the CPU at
				// ~1.8MHz so we've got less time than usual - keep this short.
				gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0xFFF]<<D0_PIN);	
		    SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				SET_DATA_MODE_IN;

				if (addr == 0x1FF4) {
          comms_enabled = true;
        }
      }
  	}
  }
  ballout:
     delay(12); // 16 or 12 don't remove!!!
  
  if (cart_to_emulate>=35) {
   vreg_set_voltage(VREG_VOLTAGE_1_30);
   int ret=set_sys_clock_khz(400000, true); // settled in compiler IDE as 250mhz overclocked
  }
  if (cart_to_emulate<=32) {
   gpio_set_dir_out_masked(BUS_H_PIN_MASK);
  }
  reboot_cartridge(addr,addr_prev);
  
  //exit_cartridge(addr,addr_prev);
 
  switch (cart_to_emulate) {

     case CART_TYPE_ACTIVISION:
      emulate_activision();
        break;

     case CART_TYPE_SUPERCART_RAM: 
      emulate_supercart_ram();
        break;
      
    case CART_TYPE_SUPERCART:
      // Plain SuperGame carts serve file bank 6 at $4000-$7FFF, i.e. exactly the
      // emulate_supercart_ef mapping (confirmed against MAME's a78 SG device, see
      // patches/PicoA10400_0.10.txt). This used to fall through into
      // emulate_supercart_ram() first - a dead call, since that function never
      // returns, so the EF path below was unreachable and every plain-SG cart
      // silently got RAM at $4000 instead of the ROM data it expects there.
     case CART_TYPE_SUPERCART_EF:
      // Continually check address lines and put associated data on bus.
      emulate_supercart_ef();
        break;
    
    case CART_TYPE_NORMALA78:
      if (romLen==1024*16) { //16k
	      while (1) {  
		      while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			      addr_prev = addr;
   	      // got a stable address
		      if (addr & 0x4000) 	{ // A14 
				    gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0x3FFF]<<D0_PIN);	
		        SET_DATA_MODE_OUT;
				    // wait for address bus to change
				    while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				    SET_DATA_MODE_IN;
          }
        } 
      } else if  (romLen==0x8000) { // 32k   
     	  while (1)   {
	        while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			      addr_prev = addr;
   	      // got a stable address
      	  if (addr & 0x8000)	{ // A15 high
            gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0x7fff]<<D0_PIN);	
		        SET_DATA_MODE_OUT;
				    // wait for address bus to change
				    while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				    SET_DATA_MODE_IN;
          }
        }
      } else if  (romLen==0xc000) { // 48k   
     	  while (1)   {
	        while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			      addr_prev = addr;
   	      // got a stable address
      	  if (addr & 0x8000)	{ // A15 high
            gpio_put_masked(DATA_PIN_MASK,rom_table[(addr)-0x4000]<<D0_PIN);	
		        SET_DATA_MODE_OUT;
				    // wait for address bus to change
				    while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				    SET_DATA_MODE_IN;
          } else {
    	      if ((addr & 0x4000))	{ // A14 high
              gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0x7fff-0x4000]<<D0_PIN);	
		          SET_DATA_MODE_OUT;
				      // wait for address bus to change
				      while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				      SET_DATA_MODE_IN;
            }
          }
        }
      } 
    break;
 
    case CART_TYPE_ABSOLUTE:
      // Continually check address lines and put associated data on bus.
      bank=0x4000;
      while (1) {       // Get address
        addr = gpio_get_all()&BUS_PIN_MASK;
        if (addr == 0x8000) {  // TO CHECK
            // Bankswitching write
           // Check for 0x01
          SET_DATA_MODE_IN;
          data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN;
          if (data == 0x01) {
            bank = 0;   // Switch to flying mode
          } else {
            if (data == 0x02) {
              bank = 0x4000; // Switch to title page
            }
          }
        } else if (addr & 0x8000) {        // Check for A15
            gpio_put_masked(DATA_PIN_MASK, rom_table[addr] << D0_PIN);
            SET_DATA_MODE_OUT;
           	while ((gpio_get_all()&BUS_PIN_MASK) == addr);     
            SET_DATA_MODE_IN;
            // Check for RW
          } else {    // Check for A14
            if (addr & 0x4000) {
                // Set the data on the bus
                gpio_put_masked(DATA_PIN_MASK, rom_table[(addr & 0x3fff) + bank] << D0_PIN);
                SET_DATA_MODE_OUT;
             		while ((gpio_get_all()&BUS_PIN_MASK) == addr);     
                SET_DATA_MODE_IN;
            }
          }
        }
       break;
    case CART_TYPE_2K:
    	while (1)  {
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
				addr_prev = addr;
		  // got a stable address
		  if (addr & 0x1000)
		    { // A12 high
				gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0x7FF]<<D0_PIN);	
		    SET_DATA_MODE_OUT;
			  // wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
			  SET_DATA_MODE_IN;
		    }
	    }
      break;
    case CART_TYPE_FE:
    bankPtr = &rom_table[0];
  	lastAccessWasFE = 0;
    addr=0;addr_prev=0;data=0;data_prev=0;
  	while (1) {
		while ((addr =gpio_get_all()&BUS_PIN_MASK) != addr_prev)
			addr_prev = addr;
		// got a stable address
		if (!(addr & 0x1000))
		{	// A12 low, read last data on the bus before the address lines change
			while ((gpio_get_all()&BUS_PIN_MASK) == addr) { data_prev = data; data =(gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
		}
		else
		{ // A12 high
			data = bankPtr[addr&0xFFF];
     	gpio_put_masked(DATA_PIN_MASK,data<<D0_PIN);	
			SET_DATA_MODE_OUT;
			// wait for address bus to change
			while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
			SET_DATA_MODE_IN;
		}
		// end of cycle
		if (lastAccessWasFE)
		{	// bank-switch - check the 5th bit of the data bus
			if (data & 0x20)
				bankPtr = &rom_table[0];
			else
				bankPtr = &rom_table[4 * 1024];
		}
		lastAccessWasFE = (addr == 0x01FE);
	}
      break;
    case CART_TYPE_4K:
      while (1) {
		    while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			  addr_prev = addr;
   	    // got a stable address
		    if (addr & 0x1000) 	{ // A12 high
				  gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0xFFF]<<D0_PIN);	
		      SET_DATA_MODE_OUT;
				  // wait for address bus to change
				  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				  SET_DATA_MODE_IN;
        }
      }
       break;
   case CART_TYPE_F6: // FxSC (0x1FF6, 0x1FF9, 0); lowbs, highbs,issc
      lowBS=0x1ff6; highBS=0x1ff9;isSC=0;
      bankPtr = &rom_table[0];
      while (1)  {
		while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		if (addr & 0x1000) 	  { // A12 high
			if (addr >= lowBS && addr <= highBS)	// bank-switch
				bankPtr = &rom_table[(addr-lowBS)*4*1024];
				// normal rom access
     	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
				SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
				SET_DATA_MODE_IN;
	  	}
  	}
      break;
    case CART_TYPE_F4: // FxSC (0x1FF4, 0x1FFb, 0); lowbs, highbs,issc
      lowBS=0x1ff4; highBS=0x1ffb; isSC=0;
      bankPtr = &rom_table[0];
      while (1) {
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		  if (addr & 0x1000)   { // A12 high
			  if (addr >= lowBS && addr <= highBS)	// bank-switch
				  bankPtr = &rom_table[(addr-lowBS)*4*1024];
				// normal rom access
     	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
				SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
				SET_DATA_MODE_IN;
	  	}
  	}
      break;
    case CART_TYPE_F8: // FxSC (0x1FF8, 0x1FF9, 0); lowbs, highbs,issc
      lowBS=0x1ff8; highBS=0x1ff9; isSC=0;
      bankPtr = &rom_table[0];
      while (1) {
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		  if (addr & 0x1000)   { // A12 high
			  if (addr >= lowBS && addr <= highBS)	// bank-switch
				  bankPtr = &rom_table[(addr-lowBS)*4*1024];
				// normal rom access
     	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
				SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
				SET_DATA_MODE_IN;
	  	}
  	}
      break;
    case CART_TYPE_F8SC: // FxSC (0x1FF8, 0x1FF9, 1); lowbs, highbs,issc
      lowBS=0x1ff8; highBS=0x1ff9; isSC=1;
      data=0;data_prev=0;
      bankPtr = &rom_table[0];
      while (1) {
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		  if (addr & 0x1000)   { // A12 high
			  if ((addr >= lowBS) && (addr <= highBS))	// bank-switch
				  bankPtr = &rom_table[(addr-lowBS)*4*1024];
        if (isSC && ((addr & 0x1F00) == 0x1000))
			    {	// SC RAM access
				  if (addr & 0x0080)
				  {	// a read from cartridge ram
       	    gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0x7F]<<D0_PIN);	
					  SET_DATA_MODE_OUT;
					  // wait for address bus to change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
					  SET_DATA_MODE_IN;
				  } else {	// a write to cartridge ram
					  // read last data on the bus before the address lines change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) 
            { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
					  ram_table[addr&0x7F] = data_prev;
				  }
			  } else { 				// normal rom access
     	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
				SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
				SET_DATA_MODE_IN;
	  	}
     }
  	}
      break;
       case CART_TYPE_F6SC: // FxSC (0x1FF6, 0x1FF9, 1); lowbs, highbs,issc
      lowBS=0x1ff6; highBS=0x1ff9; isSC=1;
      data=0;data_prev=0;
      bankPtr = &rom_table[0];
      while (1) {
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		  if (addr & 0x1000)   { // A12 high
			  if ((addr >= lowBS) && (addr <= highBS))	// bank-switch
				  bankPtr = &rom_table[(addr-lowBS)*4*1024];
        if (isSC && ((addr & 0x1F00) == 0x1000))
			    {	// SC RAM access
				  if (addr & 0x0080)
				  {	// a read from cartridge ram
       	    gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0x7F]<<D0_PIN);	
					  SET_DATA_MODE_OUT;
					  // wait for address bus to change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
					  SET_DATA_MODE_IN;
				  } else {	// a write to cartridge ram
					  // read last data on the bus before the address lines change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) 
            { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
					  ram_table[addr&0x7F] = data_prev;
				  }
			  } else { 				// normal rom access
     	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
				SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
				SET_DATA_MODE_IN;
	  	}
     }
  	}
      break;
    case CART_TYPE_F4SC: // FxSC (0x1FF4, 0x1FFB, 1); lowbs, highbs,issc
      lowBS=0x1ff4; highBS=0x1ffb; isSC=1;
      data=0;data_prev=0;
      bankPtr = &rom_table[0];
      while (1) {
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		  if (addr & 0x1000)   { // A12 high
			  if ((addr >= lowBS) && (addr <= highBS))	// bank-switch
				  bankPtr = &rom_table[(addr-lowBS)*4*1024];
        if (isSC && ((addr & 0x1F00) == 0x1000))
			    {	// SC RAM access
				  if (addr & 0x0080)
				  {	// a read from cartridge ram
       	    gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0x7F]<<D0_PIN);	
					  SET_DATA_MODE_OUT;
					  // wait for address bus to change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
					  SET_DATA_MODE_IN;
				  } else {	// a write to cartridge ram
					  // read last data on the bus before the address lines change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) 
            { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
					  ram_table[addr&0x7F] = data_prev;
				  }
			  } else { 				// normal rom access
     	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
				SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
				SET_DATA_MODE_IN;
	  	}
     }
  	}
      break;
  case CART_TYPE_E0: // FxSC (0x1FF4, 0x1FFB, 1); lowbs, highbs,issc
    curBanks[0] = 0;curBanks[1] = 0;curBanks[2] = 0;curBanks[3] = 7;
	  while (1) 	{
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			  addr_prev = addr;
		// got a stable address
		  if (addr & 0x1000) 	{ // A12 high
			  if (addr >= 0x1FE0 && addr <= 0x1FF7)
			  {	// bank-switching addresses
				  if (addr <= 0x1FE7)	// switch 1st bank
					  curBanks[0] = addr-0x1FE0;
				  else if (addr >= 0x1FE8 && addr <= 0x1FEF)	// switch 2nd bank
					  curBanks[1] = addr-0x1FE8;
				  else if (addr >= 0x1FF0)	// switch 3rd bank
					  curBanks[2] = addr-0x1FF0;
			  }
			  // fetch data from the correct bank
			  int target = (addr & 0xC00) >> 10;
		    gpio_put_masked(DATA_PIN_MASK,rom_table[curBanks[target]*1024 + (addr&0x3FF)]<<D0_PIN);	
        SET_DATA_MODE_OUT;
			  // wait for address bus to change
			  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
			  SET_DATA_MODE_IN;
		  }
	  }
      break;
  case CART_TYPE_E7: // FxSC (0x1FF4, 0x1FFB, 1); lowbs, highbs,issc
    addr=0;addr_prev = 0;
	  data = 0; data_prev = 0;
	  bankPtr = &rom_table[0];
	  fixedPtr = &rom_table[(8-1)*2048];
	  ram1Ptr = &ram_table[0];
	  ram2Ptr = &ram_table[1024];
	  ram_mode = 0;
	 
	while (1) {
		while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		// got a stable address
		if (addr & 0x1000)
		{ // A12 high
			if (addr & 0x0800)
			{	// higher 2k cartridge ROM area
				if ((addr & 0x0E00) == 0x0800)
				{	// 256 byte RAM access
					if (addr & 0x0100)
					{	// 1900-19FF is the read port
				    gpio_put_masked(DATA_PIN_MASK,ram1Ptr[addr&0xFF]<<D0_PIN);	
						SET_DATA_MODE_OUT;
						// wait for address bus to change
						while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
						SET_DATA_MODE_IN;
					} else {	// 1800-18FF is the write port
						while ((gpio_get_all()&BUS_PIN_MASK) == addr) { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
						ram1Ptr[addr&0xFF] = data_prev;
					}
				} else {	// fixed ROM bank access
					// check bankswitching addresses
					if (addr >= 0x1FE0 && addr <= 0x1FE7)
					{
						if (addr == 0x1FE7) ram_mode = 1;
						else 	{
							bankPtr = &rom_table[(addr - 0x1FE0)*2048];
							ram_mode = 0;
						}
					} else if (addr >= 0x1FE8 && addr <= 0x1FEB)
						ram1Ptr = &ram_table[(addr - 0x1FE8)*256];

				  gpio_put_masked(DATA_PIN_MASK,fixedPtr[addr&0x7FF]<<D0_PIN);	
				
					SET_DATA_MODE_OUT;
					// wait for address bus to change
					while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
					SET_DATA_MODE_IN;
				}
			} else {	// lower 2k cartridge ROM area
				if (ram_mode)
				{	// 1K RAM access
					if (addr & 0x400)
					{	// 1400-17FF is the read port
				    gpio_put_masked(DATA_PIN_MASK,ram2Ptr[addr&0x3FF]<<D0_PIN);	
						SET_DATA_MODE_OUT;
						// wait for address bus to change
						while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
						SET_DATA_MODE_IN;
					}
					else
					{	// 1000-13FF is the write port
						while ((gpio_get_all()&BUS_PIN_MASK) == addr) { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
						ram2Ptr[addr&0x3FF] = data_prev;
					}
				}
				else
				{	// selected ROM bank access
					gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0x7FF]<<D0_PIN);	
					SET_DATA_MODE_OUT;
					// wait for address bus to change
					while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
					SET_DATA_MODE_IN;
				}
			}
		}
	}
      break;
     case CART_TYPE_EF: // FxSC (0x1FE0, 0x1FEF, 0); lowbs, highbs,issc
    lowBS=0x1fe0; highBS=0x1fef; isSC=0;
      data=0;data_prev=0;
      bankPtr = &rom_table[0];
      while (1) {
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		  if (addr & 0x1000)   { // A12 high
			  if ((addr >= lowBS) && (addr <= highBS))	// bank-switch
				  bankPtr = &rom_table[(addr-lowBS)*4*1024];
        if (isSC && ((addr & 0x1F00) == 0x1000))
			    {	// SC RAM access
				  if (addr & 0x0080)
				  {	// a read from cartridge ram
       	    gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0x7F]<<D0_PIN);	
					  SET_DATA_MODE_OUT;
					  // wait for address bus to change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
					  SET_DATA_MODE_IN;
				  } else {	// a write to cartridge ram
					  // read last data on the bus before the address lines change
					  while ((gpio_get_all()&BUS_PIN_MASK) == addr) 
            { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
					  ram_table[addr&0x7F] = data_prev;
				  }
			  } else { 				// normal rom access
     	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
				SET_DATA_MODE_OUT;
				// wait for address bus to change
				while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
				SET_DATA_MODE_IN;
	  	}
     }
  	}
      break;
    case CART_TYPE_AR:  
         emulate_supercharger_cartridge();  
      break;
    case CART_TYPE_3F:  
  	  cartPages = romLen/2048;
	    addr=0; addr_prev = 0; addr_prev2 = 0;
	    data = 0; data_prev = 0;
	    bankPtr = &rom_table[0];
	    fixedPtr = &rom_table[(cartPages-1)*2048];
	    while (1) 	{
		    while (((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev) || (addr != addr_prev2))
		    {	// new more robust test for stable address (seems to be needed for 7800)
			    addr_prev2 = addr_prev;
			    addr_prev = addr;
		    }
		    // got a stable address
		    if (addr & 0x1000) { // A12 high
			    if (addr & 0x800) {
			    	  gpio_put_masked(DATA_PIN_MASK,fixedPtr[addr&0x7FF]<<D0_PIN);	
			    } else {
			    	  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0x7FF]<<D0_PIN);	
          }
			    SET_DATA_MODE_OUT;
			    // wait for address bus to change
			    while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
			    SET_DATA_MODE_IN;
		    } else {	// A12 low, read last data on the bus before the address lines change
			    while ((gpio_get_all()&BUS_PIN_MASK) == addr) { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
			    if (addr == 0x003F) {	// switch bank
				    int newPage = data_prev % cartPages; //data_prev>>8
				    bankPtr = &rom_table[newPage*2048];
			    }
			  }
		  }
      break;
    case CART_TYPE_FA:
      addr, addr_prev = 0, data = 0, data_prev = 0;
	    bankPtr = &rom_table[0];
	  while (1) 	{
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			  addr_prev = addr;
		    // got a stable address
		  if (addr & 0x1000)
		  { // A12 high
			  if ((addr >= 0x1FF8) && (addr <= 0x1FFA))	// bank-switch
				  bankPtr = &rom_table[(addr-0x1FF8)*4*1024];

			  if ((addr & 0x1F00) == 0x1100)
			  {	// a read from cartridge ram
				  gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0xFF]<<D0_PIN);	
				  SET_DATA_MODE_OUT;
				  // wait for address bus to change
				  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				  SET_DATA_MODE_IN;
			  }
			  else if ((addr & 0x1F00) == 0x1000)
			  {	// a write to cartridge ram
				  // read last data on the bus before the address lines change
				  while ((gpio_get_all()&BUS_PIN_MASK) == addr) { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
				  ram_table[addr&0xFF] = data_prev;
			  }
			  else
			  {	// normal rom access
				  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
          SET_DATA_MODE_OUT;
				  // wait for address bus to change
				  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				  SET_DATA_MODE_IN;
			  }
		  }
	  }
      break;
    case CART_TYPE_FA2:
      addr, addr_prev = 0, data = 0, data_prev = 0;
	    bankPtr = &rom_table[0];

	  while (1) 	{
		  while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			  addr_prev = addr;
		    // got a stable address
		  if (addr & 0x1000)
		  { // A12 high
			  if ((addr >= 0x1FF5) && (addr <= 0x1FFB))	// bank-switch
				  bankPtr = &rom_table[(addr-0x1FF5)*4*1024];

			  if ((addr & 0x1F00) == 0x1100)
			  {	// a read from cartridge ram
				  gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0xFF]<<D0_PIN);	
				  SET_DATA_MODE_OUT;
				  // wait for address bus to change
				  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				  SET_DATA_MODE_IN;
			  }
			  else if ((addr & 0x1F00) == 0x1000)
			  {	// a write to cartridge ram
				  // read last data on the bus before the address lines change
				  while ((gpio_get_all()&BUS_PIN_MASK) == addr) { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
				  ram_table[addr&0xFF] = data_prev;
			  }
			  else
			  {	// normal rom access
				  gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);	
          SET_DATA_MODE_OUT;
				  // wait for address bus to change
				  while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
				  SET_DATA_MODE_IN;
			  }
		  }
	  }
      break;
    case CART_TYPE_F0:
      	addr=0; addr_prev = 0;
	      bank = 0;
	while (1) 	{
		while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			addr_prev = addr;
		// got a stable address
		if (addr & 0x1000)
		{ // A12 high
			if (addr == 0x1FF0)
				bank = (bank + 1) % 16;
			// ROM access
			gpio_put_masked(DATA_PIN_MASK,rom_table[(bank * 4096)+(addr&0xFFF)]<<D0_PIN);	  
      SET_DATA_MODE_OUT;
			// wait for address bus to change
			while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
			SET_DATA_MODE_IN;
		}
	} 
     break;

      case CART_TYPE_SUPERCART_ROM: // same 9-bank shape as SUPERCART_LARGE; used to fall
                                     // through to default (no emulation at all, dead cart)
      case CART_TYPE_SUPERCART_LARGE:
      emulate_supercart_large();
        break;

    default:
        break;

 } // end switch
}
#pragma GCC pop_options

////////////////////////////////////////////////////////////////////////////////////
//                     Status Atari 
////////////////////////////////////////////////////////////////////////////////////    

void set_menu_status_msg(const char* message) {
	// 12, not 15: the kernel renders exactly 12 characters, and bytes [12..15] carry
	// out-of-band data (the oversized-file colour, the status byte). Copying 15 here
	// zero-padded the colour byte to $00 on every footer update, drawing flagged names
	// in black.
	strncpy(menu_status, message, 12);
}

void set_menu_status_byte(char status_byte) {
	menu_status[15] = status_byte;
}


/////////////////////////////////////////////////////////////////////////////////
// supercharger
/////////////////////////////////////////////////////////////////////////////////
static void setup_rom(uint8_t* rom) {
	memset(rom, 0, 0x0800);
	memcpy(rom, supercharger_bios_bin, supercharger_bios_bin_len);

	rom[0x07ff] = rom[0x07fd] = 0xf8;
	rom[0x07fe] = rom[0x07fc] = 0x07;

	//switch (tv_mode) {
	//	case TV_MODE_PAL:
			rom[0x07fa] = 0x03;
	//		break;
	//	case TV_MODE_PAL60:
	//		rom[0x07fa] = 0x02;
	//		break;
	//}
}

static void setup_multiload_map(uint8_t *multiload_map, uint32_t multiload_count) {
	LoadHeader header;
  uint8_t tmpheader[112];
	memset(multiload_map, 0, 0xff);
  
	for (uint32_t i = 0; i < multiload_count; i++) {
    int start=((i + 1) * 8448 - 256);
	  for (int j=0;j<112;j++) 	tmpheader[j]=AR_ROM[start+j];
    memcpy(&header,tmpheader,sizeof(LoadHeader));
		multiload_map[header.multiload_id] = i;
	}
}

static void read_multiload(uint8_t *buffer, uint8_t physical_index) {

  int start=physical_index * 8448;
	for (int i=0;i<8448;i++) {
     buffer[i]=AR_ROM[start+i];
   //  Serial.print(buffer[i],HEX);Serial.print(" ");
   //  if (i%256==0) Serial.println(" ");
  }
}

static void load_multiload(uint8_t *ram, uint8_t *rom, uint8_t physical_index, uint8_t *buffer) {

  read_multiload(buffer, physical_index);
  
  LoadHeader header;
  uint8_t tmpheader[112];
  for (int i=0;i<112;i++) tmpheader[i]=buffer[8192+i];

	memcpy(&header,tmpheader,sizeof(LoadHeader));
  //  Serial.println(header.block_count,HEX);
    
  for (uint8_t i = 0; i < header.block_count; i++) {
		uint8_t location = header.block_location[i];
		uint8_t bank = (location & 0x03) % 3;
		uint8_t base = (location & 0x1f) >> 2;
  //  Serial.println(header.block_location[i],HEX);
		memcpy(ram + bank * 2048 + base * 256, buffer + 256 * i, 256);
	}

	rom[0x7f0] = header.control_word;
	rom[0x7f1] = 0x9c;
	rom[0x7f2] = header.entry_lo;
	rom[0x7f3] = header.entry_hi;
}

/*************************************************************************
 * Cartridge Type Detection
 *************************************************************************/

int searchForBytes(unsigned char *bytes, int size, unsigned char *signature, int sigsize, int minhits)
{
	int count = 0;
	for(int i = 0; i < size - sigsize; ++i)
	{
		int matches = 0;
		for(int j = 0; j < sigsize; ++j)
		{
			if(bytes[i+j] == signature[j])
				++matches;
			else
				break;
		}
		if(matches == sigsize)
		{
			++count;
			i += sigsize;  // skip past this signature 'window' entirely
		}
		if(count >= minhits)
			break;
	}
  
  return (count >= minhits);
}

/* The following detection routines are modified from the Atari 2600 Emulator Stella
  (https://github.com/stella-emu) */

int isProbablySC(int size, unsigned char *bytes)
{
	int banks = size/4096;
	for (int i = 0; i < banks; i++)
	{
		for (int j = 0; j < 128; j++)
		{
			if (bytes[i*4096+j] != bytes[i*4096+j+128])
				return 0;
		}
	}
	return 1;
}

int isProbablyFE(int size, unsigned char *bytes)
{	// These signatures are attributed to the MESS project
	unsigned char signature[4][5] = {
		{ 0x20, 0x00, 0xD0, 0xC6, 0xC5 },  // JSR $D000; DEC $C5
		{ 0x20, 0xC3, 0xF8, 0xA5, 0x82 },  // JSR $F8C3; LDA $82
		{ 0xD0, 0xFB, 0x20, 0x73, 0xFE },  // BNE $FB; JSR $FE73
		{ 0x20, 0x00, 0xF0, 0x84, 0xD6 }   // JSR $F000; STY $D6
	};
	for (int i = 0; i < 4; ++i)
		if(searchForBytes(bytes, size, signature[i], 5, 1))
			return 1;

	return 0;
}

int isProbably3F(int size, unsigned char *bytes)
{	// 3F cart bankswitching is triggered by storing the bank number
	// in address 3F using 'STA $3F'
	// We expect it will be present at least 2 times, since there are
	// at least two banks
	unsigned char signature[] = { 0x85, 0x3F };  // STA $3F
	return searchForBytes(bytes, size, signature, 2, 2);
}

int isProbably3E(int size, unsigned char *bytes)
{	// 3E cart bankswitching is triggered by storing the bank number
	// in address 3E using 'STA $3E', commonly followed by an
	// immediate mode LDA
	unsigned char  signature[] = { 0x85, 0x3E, 0xA9, 0x00 };  // STA $3E; LDA #$00
	return searchForBytes(bytes, size, signature, 4, 1);
}

int isProbably3EPlus(int size, unsigned char *bytes)
{	// 3E+ cart is identified by key 'TJ3E' in the ROM
	unsigned char  signature[] = { 'T', 'J', '3', 'E' };
	return searchForBytes(bytes, size, signature, 4, 1);
}

int isProbablyE0(int size, unsigned char *bytes)
{	// E0 cart bankswitching is triggered by accessing addresses
	// $FE0 to $FF9 using absolute non-indexed addressing
	// These signatures are attributed to the MESS project
	unsigned char signature[8][3] = {
			{ 0x8D, 0xE0, 0x1F },  // STA $1FE0
			{ 0x8D, 0xE0, 0x5F },  // STA $5FE0
			{ 0x8D, 0xE9, 0xFF },  // STA $FFE9
			{ 0x0C, 0xE0, 0x1F },  // NOP $1FE0
			{ 0xAD, 0xE0, 0x1F },  // LDA $1FE0
			{ 0xAD, 0xE9, 0xFF },  // LDA $FFE9
			{ 0xAD, 0xED, 0xFF },  // LDA $FFED
			{ 0xAD, 0xF3, 0xBF }   // LDA $BFF3
		};
	for (int i = 0; i < 8; ++i)
		if(searchForBytes(bytes, size, signature[i], 3, 1))
			return 1;
	return 0;
}

int isProbably0840(int size, unsigned char *bytes)
{	// 0840 cart bankswitching is triggered by accessing addresses 0x0800
	// or 0x0840 at least twice
	unsigned char signature1[3][3] = {
			{ 0xAD, 0x00, 0x08 },  // LDA $0800
			{ 0xAD, 0x40, 0x08 },  // LDA $0840
			{ 0x2C, 0x00, 0x08 }   // BIT $0800
		};
	for (int i = 0; i < 3; ++i)
		if(searchForBytes(bytes, size, signature1[i], 3, 2))
			return 1;

	unsigned char signature2[2][4] = {
			{ 0x0C, 0x00, 0x08, 0x4C },  // NOP $0800; JMP ...
			{ 0x0C, 0xFF, 0x0F, 0x4C }   // NOP $0FFF; JMP ...
		};
	for (int i = 0; i < 2; ++i)
		if(searchForBytes(bytes, size, signature2[i], 4, 2))
			return 1;

	return 0;
}

int isProbablyCV(int size, unsigned char *bytes)
{ 	// CV RAM access occurs at addresses $f3ff and $f400
	// These signatures are attributed to the MESS project
	unsigned char signature[2][3] = {
			{ 0x9D, 0xFF, 0xF3 },  // STA $F3FF.X
			{ 0x99, 0x00, 0xF4 }   // STA $F400.Y
		};
  
  for (int i = 0; i < 2; ++i)
		if(searchForBytes(bytes, size, signature[i], 3, 1))
			return 1;
	return 0;
}

int isProbablyEF(int size, unsigned char *bytes)
{ 	// EF cart bankswitching switches banks by accessing addresses
	// 0xFE0 to 0xFEF, usually with either a NOP or LDA
	// It's likely that the code will switch to bank 0, so that's what is tested
	unsigned char signature[4][3] = {
			{ 0x0C, 0xE0, 0xFF },  // NOP $FFE0
			{ 0xAD, 0xE0, 0xFF },  // LDA $FFE0
			{ 0x0C, 0xE0, 0x1F },  // NOP $1FE0
			{ 0xAD, 0xE0, 0x1F }   // LDA $1FE0
		};
		
  for (int i = 0; i < 4; ++i)
		if(searchForBytes(bytes, size, signature[i], 3, 1))
			return 1;
	return 0;
}

int isProbablyE7(int size, unsigned char *bytes)
{ 	// These signatures are attributed to the MESS project
	unsigned char signature[7][3] = {
			{ 0xAD, 0xE2, 0xFF },  // LDA $FFE2
			{ 0xAD, 0xE5, 0xFF },  // LDA $FFE5
			{ 0xAD, 0xE5, 0x1F },  // LDA $1FE5
			{ 0xAD, 0xE7, 0x1F },  // LDA $1FE7
			{ 0x0C, 0xE7, 0x1F },  // NOP $1FE7
			{ 0x8D, 0xE7, 0xFF },  // STA $FFE7
			{ 0x8D, 0xE7, 0x1F }   // STA $1FE7
		};
	
  for (int i = 0; i < 7; ++i)
		if(searchForBytes(bytes, size, signature[i], 3, 1))
			return 1;
	return 0;
}

int isProbablyBF(unsigned char *tail)
{
 	return !memcmp(tail + 8, "BFBF", 4);
}

int isProbablyBFSC(unsigned char *tail)
{
	return !memcmp(tail + 8, "BFSC", 4);
}

int isProbablyDF(unsigned char *tail)
{
 
	return !memcmp(tail + 8, "DFBF", 4);
}

int isProbablyDFSC(unsigned char *tail)
{
	return !memcmp(tail + 8, "DFSC", 4);
}

int isProbably4KSC(unsigned char *bytes) {
	for (int i = 0; i < 256; i++)
		if (bytes[i] != bytes[0]) return 0;

	return bytes[0x0ffa] == 'S' && bytes[0x0ffb] == 'C';
}

//////////////////////////////////////////////////////////////////////////////////////
//                                    Identify Cartridge
//////////////////////////////////////////////////////////////////////////////////////
int identify_cartridge(char *filename)
{
	unsigned int image_size;
	int cart_type = CART_TYPE_NONE;
  int bytes_read;
  char A78_HEADER[0X80];
	Serial.print("Identify:");Serial.println(filename);
  
  if (!(file.open(filename))) Serial.println("Open error");
  int pos=0;
	// select type by file extension (last '.'); avoids fixed-size loop bound and out-of-bounds read
  char *dot = strrchr(filename, '.');
  if (dot != NULL) pos = dot - filename;
  char ext[4]={0,0,0,0};
  ext[0]=filename[pos+1];
  if (ext[0]>96) ext[0]=ext[0]-32;
  ext[1]=filename[pos+2];
  if (ext[1]>96) ext[1]=ext[1]-32;
  ext[2]=filename[pos+3];
  if (ext[2]>96) ext[2]=ext[2]-32;
  EXT_TO_CART_TYPE_MAP *p = ext_to_cart_type_map;
  char test[3];
	while (p->ext) {
    memcpy(test,p->ext,3);
    if ((test[0]==ext[0]) && (test[1]==ext[1]) && (test[2]==ext[2])) {
			cart_type = p->cart_type;
   		break;
		}
		p++;
	}
  Serial.print("File ext:");Serial.print(ext);Serial.println("-"); 
	image_size = file.fileSize();

  Serial.print("File size:");Serial.println(image_size);
  
	// Supercharger cartridges get special treatment, since we don't load the entire
	// file into the rom_table here
	if ((cart_type == CART_TYPE_NONE) && ((image_size % 8448) == 0))
		cart_type = CART_TYPE_AR;
	if (cart_type == CART_TYPE_AR) {
        if (image_size > sizeof(AR_ROM)) {
          Serial.print("ERROR: Supercharger image (");Serial.print(image_size);
          Serial.print(" bytes) exceeds AR_ROM capacity (");Serial.print(sizeof(AR_ROM));
          Serial.println(" bytes), truncating to prevent memory corruption");
          image_size = sizeof(AR_ROM);
        }
        for (int i=0;i<image_size;i++) {
        int readbyte=file.read();
        if (readbyte!=-1) {
          AR_ROM[i]=readbyte;
        }
      }
      romLen=image_size;
    goto found;
  }
	// otherwise, read the file into the cartridge buffer
    if (cart_type == CART_TYPE_A78) { //7800 header
          Serial.println("Loading header");
          isfor7800=1;
          for (int j=0;j<0x80;j++) A78_HEADER[j]=file.read();
    }
    if (image_size > sizeof(rom_table)) {
      // Truncate rather than refuse: the menu shows these entries in red, and loading
      // a partial image is the user's call. Truncation is what keeps the copy below
      // inside rom_table - writing past it lands directly in the TinyUSB descriptors.
      Serial.print("WARNING: ROM (");Serial.print(image_size);
      Serial.print(" bytes) exceeds rom_table capacity (");Serial.print(sizeof(rom_table));
      Serial.println(" bytes), truncating - the game will most likely misbehave");
      image_size = sizeof(rom_table);
    }
    for (int i=0;i<image_size;i++) {
      int readbyte=file.read();
      if (readbyte!=-1) {
        rom_table[i]=readbyte;
      //  Serial.print(rom_table[i]);
      } else {
        //Serial.print("Last byte red:");Serial.println(rom_table[i-1],HEX);
        //Serial.print("Eof at ");Serial.println(i,HEX);
        image_size=i;
        break;
      }
    }
	  bytes_read=image_size;
   romLen=image_size;
    //Serial.println("bytes read:");Serial.println(bytes_read,HEX);
   
    if (cart_type == CART_TYPE_A78) {
      isfor7800=1;
        Serial.println("reading A78 header");
        // image_size |= A78_HEADER[49] << 32;
        image_size = A78_HEADER[50] << 16;
        image_size |= A78_HEADER[51] << 8;
        image_size |= A78_HEADER[52];
        romLen=image_size;
        Serial.print("53:");Serial.println(A78_HEADER[53],DEC);
        Serial.print("54:");Serial.println(A78_HEADER[54],DEC);
        
        if(A78_HEADER[53] == 0) {
          A78_HEADER[54]=A78_HEADER[54]&0b10111110;
          if(image_size > (131072+0x80)) {
            cart_type = CART_TYPE_SUPERCART_LARGE;
          } else if(A78_HEADER[54] == 2 || A78_HEADER[54] == 3) {
              cart_type = CART_TYPE_SUPERCART;
            } else if(A78_HEADER[54] == 18 || A78_HEADER[54] == 19) {
              cart_type = CART_TYPE_SUPERCART_EF;
          } else if(A78_HEADER[54] == 4 || A78_HEADER[54] == 5 || A78_HEADER[54] == 6 || A78_HEADER[54] == 7) {
            cart_type = CART_TYPE_SUPERCART_RAM;
          } else if(A78_HEADER[54] == 8 || A78_HEADER[54] == 9 || A78_HEADER[54] == 10 || A78_HEADER[54] == 11) {
              cart_type = CART_TYPE_SUPERCART_ROM;
          } else {
            cart_type = CART_TYPE_NORMALA78;
          }
        } else { 
        if(A78_HEADER[53] == 1) {
          cart_type = CART_TYPE_ACTIVISION;
        } else if(A78_HEADER[53] == 2) {
          cart_type = CART_TYPE_ABSOLUTE;
          } else {
          cart_type = CART_TYPE_NORMALA78;
          }
        }
      Serial.println("7800 cart");
      goto close_exit;
    }
  // Serial.println(bytes_read);
	uint8_t tail[16];
	for (int i=0;i<16;i++) tail[i]=rom_table[bytes_read-16+i];

  if (cart_type != CART_TYPE_NONE) goto close_exit;
  //Serial.print("cart found->");Serial.println(cart_type);  
	
	// If we don't already know the type (from the file extension), then we
	// auto-detect the cart type - largely follows code in Stella's CartDetector.cpp
  //Serial.println("autodetect");
	if (is_ace_cartridge(bytes_read, rom_table))
	{
		cart_type = CART_TYPE_ACE;
	}
	else if (image_size <= 64 * 1024 && (image_size % 1024) == 0 && isProbably3EPlus(image_size, rom_table))
	{
    // Serial.println("check 3ep");
		cart_type = CART_TYPE_3EP;
	}
	else if (image_size == 2*1024)
	{
		Serial.println("2k");
    if (isProbablyCV(bytes_read, rom_table))
    	cart_type = CART_TYPE_CV;
		else
			cart_type = CART_TYPE_2K;
	}
	else if (image_size == 4*1024)
	{
  	cart_type = isProbably4KSC(rom_table) ? CART_TYPE_4KSC : CART_TYPE_4K;
	}
	else if (image_size == 8*1024)
	{
  	// First check for *potential* F8
		unsigned char  signature[] = { 0x8D, 0xF9, 0x1F };  // STA $1FF9
		int f8 = searchForBytes(rom_table, bytes_read, signature, 3, 2);

		if (isProbablySC(bytes_read, rom_table))
			cart_type = CART_TYPE_F8SC;
		else if (memcmp(rom_table, rom_table + 4096, 4096) == 0)
			cart_type = CART_TYPE_4K;
		else if (isProbablyE0(bytes_read, rom_table))
			cart_type = CART_TYPE_E0;
		else if (isProbably3E(bytes_read, rom_table))
			cart_type = CART_TYPE_3E;
		else if (isProbably3F(bytes_read, rom_table)) 
      cart_type = CART_TYPE_3F;
    else if (isProbablyFE(bytes_read, rom_table) && !f8)
			cart_type = CART_TYPE_FE;
		else if (isProbably0840(bytes_read, rom_table))
			cart_type = CART_TYPE_0840;
		else {
			cart_type = CART_TYPE_F8;
		}
	}
	else if (image_size == 8*1024 + 3) {
		cart_type = CART_TYPE_PP;
	}
	else if(image_size >= 10240 && image_size <= 10496)
	{  // ~10K - Pitfall II
		cart_type = CART_TYPE_DPC;
	}
	else if (image_size == 12*1024)
	{
		cart_type = CART_TYPE_FA;
	}
  	else if (image_size == 28*1024)
	{
		cart_type = CART_TYPE_FA2;
	}
	else if (image_size == 16*1024)
	{
		if (isProbablySC(bytes_read, rom_table))
			cart_type = CART_TYPE_F6SC;
		else if (isProbablyE7(bytes_read, rom_table))
			cart_type = CART_TYPE_E7;
		else if (isProbably3E(bytes_read, rom_table))
			cart_type = CART_TYPE_3E;
		else
			cart_type = CART_TYPE_F6;
	}
	else if (image_size == 32*1024)
	{
		if (isProbablySC(bytes_read, rom_table))
			cart_type = CART_TYPE_F4SC;
		else if (isProbably3E(bytes_read, rom_table))
			cart_type = CART_TYPE_3E;
		else if (isProbably3F(bytes_read, rom_table)) 
      cart_type = CART_TYPE_3F;
		else
			cart_type = CART_TYPE_F4;
	}
	else if (image_size == 64*1024)
	{
		if (isProbably3E(bytes_read, rom_table))
			cart_type = CART_TYPE_3E;
		else if (isProbably3F(bytes_read, rom_table)) 
      cart_type = CART_TYPE_3F;
		else if (isProbablyEF(bytes_read, rom_table))
		{
			if (isProbablySC(bytes_read, rom_table))
				cart_type = CART_TYPE_EFSC;
			else
				cart_type = CART_TYPE_EF;
		}
		else
			cart_type = CART_TYPE_F0;
	}
	else if (image_size == 128 * 1024) {
		if (isProbablyDF(tail))
			cart_type = CART_TYPE_DF;
		else if (isProbablyDFSC(tail))
			cart_type = CART_TYPE_DFSC;
	}
	else if (image_size == 256 * 1024)
	{
		if (isProbablyBF(tail))
			cart_type = CART_TYPE_BF;
		else if (isProbablyBFSC(tail))
			cart_type = CART_TYPE_BFSC;
	}

found:
	if (cart_type)
		cart_size_bytes = image_size;
   
close_exit:
  Serial.print("RomLen:");Serial.println(image_size,HEX);
  
  file.close();
	return cart_type;
}

static const unsigned char MagicNumber[] = "ACE-2600";

int is_ace_cartridge(unsigned int image_size, uint8_t *rom_table)
{
	if(image_size < sizeof(ACEFileHeader))
		return 0;

	ACEFileHeader * header = (ACEFileHeader *)rom_table;

	// Check magic number
	for(int i = 0; i < 8; i++)
	{
		if(MagicNumber[i] != header->magic_number[i])
			return 0;
	}
  
	return 1;
}

// Callback invoked when received READ10 command.
// Copy disk's data to rom_table (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb (uint32_t lba, void* rom_table, uint32_t bufsize)
{
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) rom_table, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in rom_table to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* rom_table, uint32_t bufsize)
{
 
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, rom_table, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  fs_changed = true;

}
// check if dir up
bool checkDirUp (char* fileto) {
  size_t len = strlen(fileto);
  return (len>=2) && (fileto[len-1]=='.') && (fileto[len-2]=='.');
}
// check if dir up
void DirUp() {
  int len = strlen(path);
	path[len]=0;
  len--;
 	if (len>0) {
		while (len && path[--len] != '/');
		path[len] = '/';
    path[len+1]=0;
	}
  if (len==0) {
    path[0]='/';
    path[1]=0;
  }
 	
}
////////////////////////////////////////////////////////////////////////////////////
//                     LOAD Game
////////////////////////////////////////////////////////////////////////////////////
void LoadGame(int numfile) { 
  String riga;
  
  int verified=0;
  
  Serial.print("load game n.:");Serial.print(numfile);
  Serial.print(" - ");Serial.println(filelist[numfile*48]);

  char filetoadd[48];
  for(int x=0;x<48;x++) filetoadd[x]=filelist[numfile*48+x];//   memcpy(filelist[contfile*48],filename,48);
  filetoadd[47]=0; // guarantee null-terminator regardless of source name length

  snprintf(filetoopen, sizeof(filetoopen), "%s%s", path, filetoadd); // bounds-checked; replaces unsafe strcat/strcat that overflowed filetoopen[50]
           

  Serial.print("FTO:");Serial.println(filetoopen);
  
  if (checkDirUp(filetoopen)) {
    DirUp();
    Serial.print("DirUp:");Serial.println(path);
    root.close();
    if (!root.open(path)) Serial.println("Error openin new path");     
    Serial.print(" new path:");root.printName(&Serial);Serial.println(" ");
  } else {
  if (!file.open(filetoopen)) {
    Serial.println("open error");
    while(1);
  }
  
  if (file.isDir()) {
    //Serial.println("is dir");
    memset(path,0,sizeof(path));
    snprintf(path, sizeof(path)-1, "%s", filetoopen); // bounds-checked (filetoopen can now be longer than path); leaves room for the trailing '/' appended below
    file.close();
    Serial.print("old path:");root.printName(&Serial);
    root.close();
    strcat(path,"/");
    Serial.print("opening path:");Serial.println(path);
    if (!root.open(path)) Serial.println("Error openin new path");
     
     Serial.print(" new path:");root.printName(&Serial);Serial.println(" ");
  //    AtariMenu(1);
 
  } else {
    Serial.println("start loading");
    file.close();

    cart_to_emulate=identify_cartridge(filetoopen);

    //delay(300);
     
        // 400 MHz
   
   //vreg_set_voltage(VREG_VOLTAGE_1_30);
   //int ret=set_sys_clock_khz(400000, true); // settled in compiler IDE as 250mhz overclocked
   //Serial.print("Ovrclk ret:");Serial.println(ret);
   // set_sys_clock_pll(1200000000, 4, 1);
   
    Serial.println("----------------------------------");
    Serial.print("Cart type:");Serial.println(cart_to_emulate);
    Serial.println("----------------------------------");
    Serial.println(" ");
 
    //while(retaddr!=CART_CMD_START_CART);


    set_menu_status_byte(1);
    delay(100);
    newgame=1;  

  }
 }
}

void printram() {
  for (int j=0;j<85;j++) {
    for (int i=0;i<12;i++) {
    Serial.print(menu_ram[i+j*12]);
  }
  Serial.print(" ");
  Serial.println(j);
  }
}
void printfilelist() {
  Serial.println("----------------filelist-----------");
  for (int j=0;j<8;j++) {
    Serial.print(filelist[j*48]);
  Serial.print(" ");Serial.println(j);
  }
}

////////////////////////////////////////////////////////////////////////////////////
//                     MENU ATARI
////////////////////////////////////////////////////////////////////////////////////

// case-insensitive compare of two null-terminated (or space-padded) filelist entries
int compareNamesCI(const char* a, const char* b) {
  for (int i=0;i<47;i++) {
    char ca=a[i], cb=b[i];
    if ((ca>96)&&(ca<123)) ca-=32;
    if ((cb>96)&&(cb<123)) cb-=32;
    if (ca!=cb) return (unsigned char)ca-(unsigned char)cb;
    if (ca==0) break;
  }
  return 0;
}

// sort filelist[start..start+count) in place: directories first, alphabetical within each group;
// direntry_isdir is kept in sync with the swaps
void sortFileList(int start, int count) {
  for (int i=start;i<start+count-1;i++) {
    int best=i;
    for (int j=i+1;j<start+count;j++) {
      bool better;
      if (direntry_isdir[j]!=direntry_isdir[best]) {
        better = direntry_isdir[j]>direntry_isdir[best]; // directories before files
      } else {
        better = compareNamesCI(&filelist[j*48], &filelist[best*48])<0;
      }
      if (better) best=j;
    }
    if (best!=i) {
      char tmp[48];
      memcpy(tmp,&filelist[i*48],48);
      memcpy(&filelist[i*48],&filelist[best*48],48);
      memcpy(&filelist[best*48],tmp,48);
      char t=direntry_isdir[i]; direntry_isdir[i]=direntry_isdir[best]; direntry_isdir[best]=t;
      t=direntry_toobig[i]; direntry_toobig[i]=direntry_toobig[best]; direntry_toobig[best]=t;
    }
  }
}

// Render one listing entry into the 12 bytes the Atari reads for that row.
// off skips characters of the name, which is how the highlighted row scrolls. The high
// bit of a character is never drawn (the glyph renderer masks with AND #$7F), so it
// carries the colour flags - and those belong to fixed screen positions, not to the
// characters, hence they are applied after the shift.
void renderEntry(int idx, int off) {
  for (int i=0;i<12;i++) {
    char c = (off+i < 48) ? filelist[idx*48+off+i] : 32;
    if ((c>96) && (c<123)) c=c-32; // the font has no lowercase letters
    if (c==0) c=32;
    if ((i==0) && direntry_isdir[idx]) c=c|0x80;
    if ((i==1) && direntry_toobig[idx]) c=c|0x80;
    menu_ram[idx*12+i]=c;
  }
}

void AtariMenu(int tipo) { // 1=start,2=next page, 3=prev page, 4=dir up
  int contfile=0;
  char filename[48];
 
  memset(filename,0,sizeof(filename));
  switch (tipo) {
    case 1: { // root dir
      //memset(menu_ram,32,1023);
      contfile=0;
      memset(filelist,0,sizeof(filelist));
      memset(direntry_isdir,0,sizeof(direntry_isdir));
      memset(direntry_toobig,0,sizeof(direntry_toobig));
      root.rewind();
      // Serial.println(" Menu-1:");

      // root.printName(&Serial);Serial.println(" ");

      int firstentry=0; // index where real (sortable) entries start: 1 if ".." was added, 0 otherwise
        if ((!root.isRoot())&&(contfile==0)) {
       //    Serial.println("subdir, adding ..");
           memset(filename,0,sizeof(filename));
           memcpy(filename,"..",2); // no trailing padding: filetoadd is read as a C-string (checkDirUp needs it to end in "..")
           for(int x=0;x<48;x++) filelist[contfile*48+x]=filename[x];//   memcpy(filelist[contfile*48],filename,48);
        contfile++;
        firstentry=1;
      }

      // pass 1: collect names + type, skipping hidden entries; no rendering yet.
      // Directories are collected in their own sweep FIRST. menu_ram is what the Atari
      // reads at $1800-$1BFF, so the listing can never hold more than 1024/12 = 85
      // entries. That cut-off used to fall wherever the filesystem happened to return
      // entries, and sorting only ran afterwards - so a subdirectory returned after 85
      // files was dropped before sorting could lift it to the top, leaving it invisible
      // and the whole subtree unreachable. Files now fill only the room left over.
      int dirs_found=0, files_found=0;
      for (int wantdir=1; wantdir>=0; wantdir--) {
        root.rewind();
        while (file.openNext(&root, O_RDONLY) ) {
          bool isdir = file.isDir();
          if (file.isHidden() || (isdir?1:0)!=wantdir) { // wrong kind for this sweep
            file.close();
            continue;
          }
          if (isdir) dirs_found++; else files_found++;

          if (contfile<=84) { // bounds check: filelist/menu_ram hold at most 85 entries (0..84)
            memset(filename,32,sizeof(filename));
            file.getName(filename, 48); // may fail/truncate silently for names >47 chars
            filename[sizeof(filename)-1]=0; // force null-terminator so later strcat can't run past this buffer
            for(int x=0;x<48;x++) filelist[contfile*48+x]=filename[x];//   memcpy(filelist[contfile*48],filename,48);
            direntry_isdir[contfile] = isdir ? 1 : 0;
            // A .a78 file starts with a 128-byte header that identify_cartridge() consumes
            // before copying, so only the payload has to fit in rom_table. Without this a
            // 7800 ROM of exactly 144KB (file size 147584) would be flagged despite loading
            // whole. This mirrors what the loader really does: bytes are lost only when
            // fileSize-header exceeds the buffer.
            uint32_t payload = file.fileSize();
            char *ext = strrchr(filename, '.');
            if (ext && (ext[1]=='a'||ext[1]=='A') && ext[2]=='7' && ext[3]=='8' && payload>=0x80)
              payload -= 0x80;
            direntry_toobig[contfile] = (!isdir && payload > sizeof(rom_table)) ? 1 : 0;
            contfile++;
          }
          file.close();
        }
      }

      // pass 2: sort real entries (".." excluded) - directories first, alphabetical within each group
      sortFileList(firstentry, contfile-firstentry);

      // The 85-entry ceiling is a hard limit of the cart<->Atari protocol, so a directory
      // with more entries is shown partially. Say so in the footer instead of silently
      // hiding files - padded to 12 chars because unset bytes render as underscores.
      {
        int shown = contfile-firstentry, found = dirs_found+files_found;
        char msg[20];
        if (found > shown) {
          char num[20];
          snprintf(num, sizeof(num), "%d OF %d", shown, found);
          snprintf(msg, sizeof(msg), "%-12.12s", num);
          Serial.print("WARNING: directory holds ");Serial.print(found);
          Serial.print(" entries, only ");Serial.print(shown);Serial.println(" fit in the menu");
        } else {
          snprintf(msg, sizeof(msg), "%-12.12s", MENU_FOOTER_TEXT);
        }
        set_menu_status_msg(msg);
      }

      // pass 3: render every entry from its start (see renderEntry)
      for (int idx=0; idx<contfile; idx++) renderEntry(idx, 0);
      menu_count = contfile;
      marquee_row = -1; // listing rebuilt: nothing is being scrolled yet

      //Serial.print("stop byte a:");Serial.println(contfile);
      menu_ram[(contfile)*12]=0;

      Serial.print("tot file:");Serial.println(contfile-1);

      break;
    }
    case 2:
    //  Serial.println(" Menu-2:");
      //printram();
      LoadGame(gamechoosen); // gc
      break;
   }
}
////////////////////////////////////////////////////////////////////////////////////
//                     SETUP
////////////////////////////////////////////////////////////////////////////////////

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_15); // set to 1_15 or 1_20 if you experience some glitches
  //delay(10); // to stabilize voltage
  set_sys_clock_khz(250000, true); // settled in compiler IDE as 250mhz overclocked
 //   gpio_init_mask(ALL_GPIO_MASK);
  //pinMode(A0_PIN,INPUT);
  pinMode(RW_PIN,INPUT);
  bool carton=false;

  while (to_ms_since_boot(get_absolute_time()) < 200) // era 200
    {
      if (gpio_get(A0_PIN)==1)
        carton=true;
    }
 flash.begin();

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Pico2600", "External Flash", "1.0");

  // Set callback
   usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size() / 512, 512);

  // MSC is ready for read/write
   usb_msc.setUnitReady(true);

   usb_msc.begin();

  // Init file system on the flash
  fs_formatted = fatfs.begin(&flash);

  Serial.begin(115200);
  memset(ram_table,0,sizeof(ram_table));
  while ((!Serial)&&(to_ms_since_boot(get_absolute_time())) < 200);   // wait for native usb
  
  if ( !fs_formatted )
  {
    Serial.println("Failed to init files system, flash may not be formatted");
  }
if (!carton) {
  Serial.println("Connected to PC");
  Serial.print("JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: "); Serial.print(flash.size() / 1024); Serial.println(" KB");

  fs_changed = true; // to print contents initially
    
  } else {
  Serial.println("connected to Atari");
  }

}
////////////////////////////////////////////////////////////////////////////////////
//                     MAIN LOOP
////////////////////////////////////////////////////////////////////////////////////    
  
void loop() 
{
  u_int8_t gc=0;
  
  bool menuletto=false;
  Serial.println("Opening root");
  strcpy(path,"/");
    if ( !root.open(path) ) {
      Serial.println("open root failed");
      while(1);
    }

    Serial.println("Flash contents:");

    // Open next file in root.
    // Warning, openNext starts at the current directory position
    // so a rewind of the directory may be required.
    
     set_menu_status_msg(MENU_FOOTER_TEXT);
     menu_status[12] = OVERSIZED_COLOUR; // read by the menu kernel for oversized entries
   	 set_menu_status_byte(0);

  int i=0;
  int cmd_exec=0; // start with read files
  // Initialize GPIO pins
  Serial.println("waiting commands..");
 
  while (1) {
  if (retaddr>=CART_CMD_SEL_ITEM_n) {
    if (retaddr==CART_CMD_ROOT_DIR) {
      retaddr=0;
      AtariMenu(1);
      menuletto=1;
    } else if (menuletto) {
      gamechoosen = retaddr-CART_CMD_SEL_ITEM_n;
      Serial.print("Selected:");Serial.println(gamechoosen);
      AtariMenu(2);
      if (newgame==0){
          AtariMenu(1);     
          delay(280);
      }
         retaddr=0;
      //delay(100);
      } 
      retaddr=0;
    }

   // Scroll the highlighted entry when its name does not fit in 12 columns.
   if (menuletto && cursor_row>=0 && cursor_row<menu_count) {
     if (cursor_row != marquee_row) {
       if (marquee_row>=0 && marquee_row<menu_count) renderEntry(marquee_row, 0); // put the old row back
       marquee_row = cursor_row;
       marquee_tick = 0;
       marquee_last = millis();
       renderEntry(marquee_row, 0);
     } else if (millis()-marquee_last > MARQUEE_STEP_MS) {
       marquee_last = millis();
       int len = strlen(&filelist[marquee_row*48]);
       if (len > 12) {
         int maxoff = len-12;
         marquee_tick = (marquee_tick+1) % (maxoff + 2*MARQUEE_HOLD);
         int off = marquee_tick < MARQUEE_HOLD          ? 0
                 : marquee_tick < MARQUEE_HOLD + maxoff ? marquee_tick-MARQUEE_HOLD
                                                        : maxoff;
         renderEntry(marquee_row, off);
       }
     }
   }

   if (cmd_exec!=0) {
    Serial.print("GC:");Serial.println(gamechoosen);
    delay(500);
   }
  }
   
 }

////////////////////////////////////////////////////////////////////////////////////
//                     MAIN LOOP
////////////////////////////////////////////////////////////////////////////////////    
  
void loop1()
{
     
}

