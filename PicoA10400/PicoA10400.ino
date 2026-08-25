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

#include "ym2151.h"  // YM2151 (OPM) FM synthesis, same audio pin. Included
                     // BEFORE pokey.h, because pokey_window_service() hands
                     // the $04xx window over to it for a YM cart.
#include "pokey.h"   // minimal POKEY audio on GPIO29 (cart pin 18) - see
                     // pokey_feasibility/. Included here, after the pin masks,
                     // because pokey_capture_write() uses them.

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
#define CART_TYPE_MRAM	41	// Atari 7800 flat ROM + mRAM ("masked RAM") @$4000
#define CART_TYPE_VERSA	42	// Atari 7800 VersaBoard: SuperGame + 2x16K banked RAM

// CCM_RAM/CCM_SIZE/RAM_BANKS/CCM_BANKS/MAX_RAM_BANK/MAX_CCM_BANK removed
// (OPTIMIZATION.md 2.6): UnoCart (STM32) relic, unused anywhere in this
// file. $10000000 is XIP FLASH on an RP2040, not fast RAM - CCM_RAM was
// not just dead, it was a live landmine for any future UnoCart port that
// reached for it by habit.

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

// const (OPTIMIZATION.md 2.4): read-only lookup table, only ever walked
// in identify_cartridge() - moves it from RAM (.data) to flash (.rodata).
const EXT_TO_CART_TYPE_MAP ext_to_cart_type_map[] = {
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
// AR_ROM overlaid onto the unused tail of rom_table (OPTIMIZATION.md 2.1),
// instead of a separate 33792-byte array. Safe because the two never
// overlap in a live game: the AR file-load path (identify_cartridge())
// never touches rom_table at all - "we don't load the entire file into
// the rom_table here", it goes straight to AR_ROM and jumps to "found" -
// and emulate_supercharger_cartridge() never touches rom_table past
// offset 0x40FF (ram/rom/multiload_map/multiload_buffer; read_multiload()
// fills exactly rom_table[0x2100..0x40FF], the top of its working set).
// That leaves 96KB of margin between the emulator's high-water mark and
// this overlay's start. AR_ROM is a macro, not an array, from here down -
// every place that used to size the array now reads AR_ROM_SIZE, since
// measuring the macro directly would size the pointer expression
// (4 bytes), not the intended capacity.
#define AR_ROM_SIZE (8448*4)
#define AR_ROM (rom_table + sizeof(rom_table) - AR_ROM_SIZE)
// bugs/b01: file names were capped at 48 bytes (47 usable + terminator). SdFat's
// getName8() correctly REFUSES to write past the buffer it's given (FsUtf::cpToMb
// returns null when it runs out of room) - that is not a library bug. The bug was
// entirely ours: real ROM names in ROMS/ run up to 61 chars, so any name >=48 chars
// hit that refusal, and SdFat's own failure path then zeroed byte 0 of the (already
// partially-written) buffer instead of the byte it had actually reached - producing
// a name that starts with '\0' followed by real characters 1..46. That corrupted
// sort order (a leading '\0' sorts before everything), marquee scrolling (strlen()
// of a string starting with '\0' is 0), and loading (LoadGame() rebuilds the path
// from this same corrupted string, so it silently re-opens the current directory
// instead of the file). Fix: give the buffer enough room that real names never hit
// SdFat's failure path in the first place. 80 covers the longest name currently in
// ROMS/ (61) with margin; raise it if a longer name ever needs it. Verified via the
// L<len>C<code> footer diagnostic (see bugs/b01/worklog.md) on real hardware.
#define MAX_NAME_LEN 80
char filelist[85*MAX_NAME_LEN]; // 85 entries, MAX_NAME_LEN bytes each (incl. terminator)
char direntry_isdir[85]; // 1 if filelist[n] is a directory, 0 if a regular file (".." counts as 0: no highlight, not sorted)
char direntry_toobig[85]; // 1 if filelist[n] is a file larger than rom_table: loaded truncated, shown red in the menu
#define MENU_FOOTER_TEXT "AOTTAv01 HR7" // 12 chars: the menu kernel renders exactly 12 per row
// Colour of oversized-ROM names. The kernel reads this at runtime from menu_status[12],
// so changing it needs no ROM patch - just this line. $66 was picked by sweeping all 16
// hues on the actual PAL TV: hue 6 is the red family here, and luminance 6 keeps it
// saturated (luminance A washed out to near-white). More saturated: $64. Brighter: $68.
#define OVERSIZED_COLOUR 0x66

// Core-1 clock used while a 7800 cart type is being emulated (cart_to_emulate>=35).
// Everything else - menu, USB, 2600 types - keeps the 250MHz/1.15V set in setup().
//
// not_working_roms4 "Krok 13/17": the 400000 this used to be is ~3x past the RP2040's
// 133MHz spec, and the measured timing budget says it buys nothing FOR BUS TIMING.
// Response path in emulate_supercart_ram() is 18 instructions (objdump), and MARIA's
// tightest cartridge access interval is ~279ns (maria.cpp cost model at 7.159MHz):
//     400MHz -> 45ns typical / 107ns worst case   (~6x margin)
//     250MHz -> 72ns typical / 172ns worst case   (~3.9x margin)
// Both fit comfortably, and both are faster than the 150-250ns mask ROMs MARIA was
// designed against. So dropping from 400 to 250 was a pure stability experiment: if
// artifacts dropped at 250MHz, the overclock was CAUSING them rather than preventing
// them - which is also what upstream's own "set to 1_15 or 1_20 if you experience
// some glitches" comment in setup() hints at.
//
// 2026-08-24: briefly raised to 300000 on the theory that YM2151 synthesis on core 0
// was compute-bound. That was a guess, and it was WRONG - the real cause was a
// truncated timestamp in ym2151.h's pacing loop, which made it free-run instead of
// waiting (see the YM_PERIOD_US_FRAC comment there). The 300MHz build also came back
// from hardware with NO audio at all, i.e. it regressed something rather than fixing
// anything - unsurprising, since unlike every previous clock change here this one is
// an ACTUAL PLL reconfiguration performed on core 1 while core 0 is already running,
// not the no-op re-set of an already-current value. Reverted, and it stays reverted
// unless a measurement (ym2151.h YM_REPORT_RATE) actually shows core 0 short of
// cycles at 250MHz.
#define EMU_CLOCK_KHZ 250000

// YM_CLOCK_KHZ - the core clock used while a YM2151 cart plays - is defined in
// ym2151.h, next to the sample rate that motivates it. It has to live there
// because that header is included long before this point and its own benchmark
// reports against it.


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
 char filetoopen[256]; // must hold path[128] + filename (up to MAX_NAME_LEN-1 chars) + terminator; was 50, which overflowed with long names or subdirectories
 
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
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
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

// v0.13 (P1): same treatment 0.09 gave emulate_supercart_large and 0.12 gave
// emulate_activision. At -Os GCC outlines the masked-GPIO-write helper into a
// copy in FLASH called through a RAM veneer on every bus response (~25 cycles
// + XIP-miss jitter, see PicoA10400_tune/README.md). -O2 plus the direct SIO
// store below keep the whole response path in RAM - verify with check_hotpath.sh.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_ef()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      uint32_t bank=0, addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      // Bank-number mask derived from the real ROM size - see the identical
      // comment in emulate_supercart_ram() for the full reasoning and sources.
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));

      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (addr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                sio_hw->gpio_out = (uint32_t)rom_table[addr + 0x10000] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + bank] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                        // Krok 20: end-of-cycle capture, proven on CART_TYPE_SUPERCART_RAM
                        // in Krok 19. The 6502 does not drive the data lines until the
                        // second half of a write cycle, and this loop polls, so it spots
                        // the write at a random phase - sampling here reads the bus before
                        // the CPU has driven it roughly half the time. Keep the last value
                        // seen while the address was still valid. Bounded at 64 turns
                        // (~2x one 6502 cycle at 250MHz): an unbounded wait hung the cart
                        // in Krok 18. Note the 2600 paths in setup1() have always used
                        // this shape ("while (addr unchanged) { data_prev = data; ... }");
                        // only the 7800 SuperGame paths were missing it.
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        bank=((last >> D0_PIN) & sc_bank_mask)*0x4000;  // was & 0xf - see the mask comment above
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            // EXFIX - bank 6 is in 0x4000
            if (addr & 0x4000) {
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + 0x18000] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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

// v0.13 (P1): same treatment 0.09 gave emulate_supercart_large and 0.12 gave
// emulate_activision. At -Os GCC outlines the masked-GPIO-write helper into a
// copy in FLASH called through a RAM veneer on every bus response (~25 cycles
// + XIP-miss jitter, see PicoA10400_tune/README.md). -O2 plus the direct SIO
// store below keep the whole response path in RAM - verify with check_hotpath.sh.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_ram()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      uint32_t bank=0;
      // v0.13 (P1): romLen is volatile, so the fixed-bank index below was
      // re-read from memory on every pass. Constant for the whole game - hoist.
      const uint32_t fixed_base = (uint32_t)romLen - 0x8000;
      uint32_t addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      // not_working_roms4 "Przypadek 3": the bank number latched on a $8000-$BFFF
      // write used to be masked with a fixed "& 0xf", i.e. 16 banks, no matter how
      // big the cart actually is. Both reference implementations bound it to the
      // real ROM instead:
      //   MAME  a78_slot.cpp:74-77 + rom.cpp:388 - m_bank = data & m_bank_mask,
      //         where m_bank_mask = (nbanks odd) ? nbanks-2 : nbanks-1;
      //   ProSystem/JS7800 Cartridge.js:729 - the write is IGNORED entirely unless
      //         cartridge_GetBank(data) < size/16384.
      // Every SUPERCART_RAM cart in this library is 128KB = 8 banks, so the old
      // mask let a bankswitch write select banks 8-15, which do not exist: bank 8
      // reads whatever the previously loaded game left in rom_table (it is never
      // cleared between loads), banks 9-15 index past the 144KB array altogether -
      // straight into the TinyUSB descriptors, per CLAUDE.md. That matters even for
      // a well-behaved ROM, because this firmware samples the data bus on a write at
      // a moment of its own choosing rather than on the CPU's write strobe: a single
      // misread D3 turns a legal "select bank 5" into "show 16KB of garbage until the
      // next bankswitch" - transient corruption that appears and disappears, which is
      // exactly the reported symptom. Pico2A10400 already hardcoded "& 0x07" here;
      // deriving the mask keeps both boards correct for 4/8/9/16-bank carts alike.
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));
      
      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (addr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + fixed_base] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                //sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + bank] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + bank] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                        // not_working_roms4 "Krok 19": a 6502 puts address and R/W up at
                        // the start of a cycle but does not drive the data lines until its
                        // second half, so grabbing the byte here - a few instructions after
                        // spotting the write - reads the bus before the CPU has driven it.
                        // Because this loop polls, it catches the write at a random phase,
                        // so the byte is sometimes good and sometimes garbage: intermittent
                        // wrong-bank corruption, i.e. 16KB of the wrong graphics until the
                        // next bankswitch. Take instead the LAST sample seen while the
                        // address was still valid (= end of the write cycle, data settled).
                        // BOUNDED on purpose: Krok 18 used an unbounded wait here and the
                        // cart hung (yellow screen). One 6502 cycle at 250MHz is ~140 Pico
                        // cycles and this spin is ~5 cycles per turn, so ~28 turns covers a
                        // whole bus cycle; 64 gives 2x headroom while capping the time we
                        // can ever stop serving the bus at well under 1.5us.
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        bank=((last >> D0_PIN) & sc_bank_mask)*0x4000;  // was & 0xf
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            rawaddr=gpio_get_all();
            // EXram - 16k is in 0x4000
            if (rawaddr & 0x4000) {
                addr= rawaddr & 0x3fff;
                sio_hw->gpio_out = (uint32_t)ram_table[addr] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                        // Krok 19: same end-of-cycle capture as the bank register above.
                        // This path corrupts whatever the game just stored in on-cart RAM,
                        // which for these carts is graphics - so a byte sampled before the
                        // 6502 drives it shows up directly on screen. 'addr' has already
                        // been narrowed to a 14-bit offset here, so compare a full-width
                        // address captured now.
                        uint32_t wlast = gpio_get_all(), wcur;
                        uint32_t waddr = wlast & BUS_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS_PIN_MASK) != waddr) break;
                            wlast = wcur;
                        }
                        ram_table[waddr & 0x3fff] = (wlast >> D0_PIN) & 0xff;
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
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
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
                        // Krok 20: end-of-cycle capture - see emulate_supercart_ef() above
                        // for the full reasoning. Directly relevant here: the comment below
                        // notes Alien Brigade writes values it loaded from memory, so a
                        // half-driven bus sampled too early is exactly how a legal bank
                        // number turns into a wrong one.
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        rawaddr = last;
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

// Hardened + reordered (patch 0.26). This was the ONLY emulate_* function left
// at the sketch's default -Os, still driving the bus through gpio_put_masked() -
// the pair patch 0.09 replaced everywhere else, because gpio_put_masked()
// compiles to a call into a RAM-resident helper (~25 cycles plus XIP-cache-miss
// jitter) instead of one inlined store, on EVERY bus response. 0.13 scoped
// Supercharger out of that pass ("needs a variant preserving A13-A15") and it was
// never revisited; AR was also never hardware-tested until now.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercharger_cartridge())  {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
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

  // NOTE: memset(ram)/setup_rom()/setup_multiload_map() used to run HERE. They
  // now run in setup_supercharger(), called BEFORE reboot_cartridge() - see the
  // call site in setup1() for why that ordering is mandatory.

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

			// Masked toggle, copied from PlusCart-Pico's RP2040 DATA_OUT
			// (ORIG/PlusCart-Pico/include/cartridge_io.h:30) - the closest reference
			// implementation available, same chip. Stays entirely in RAM (no flash
			// veneer) yet touches ONLY D0-D7, which is exactly the "preserving variant"
			// patch 0.13 said this function needed: "in 2600 mode A13-A15
			// (BUS_H_PIN_MASK) are also outputs, so the bare SIO store used here would
			// clobber them".
			sio_hw->gpio_togl = (sio_hw->gpio_out ^ ((uint32_t)value_out << D0_PIN)) & DATA_PIN_MASK;
		
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

// DPC - Activision's custom chip, the one in "Pitfall II - Lost Caverns". Not a
// bankswitching scheme: a co-processor with 8 "data fetchers" (address counters
// that walk a 2KB display-data area backwards), per-fetcher top/bottom/flag
// registers, an LFSR random-number generator, and three of the fetchers doubling
// as a music generator.
//
// Ported from UnoCart's cartridge_dpc.c, which is the same class of device as us
// (a microcontroller bit-banging the 2600 bus), cross-checked against Stella's
// CartDPC.cxx for the authoritative behaviour. The two agree on everything that
// matters here, including the amplitude table {00,04,05,09,06,0A,0B,0F}
// (CartDPC.cxx:183) and the display data living at offset 8192 of the image
// (CartDPC.cxx:64) - so rom_table[0..8191] is the two 4KB banks and
// rom_table[8192..10239] is the display data.
//
// NO AUDIO HARDWARE IS INVOLVED. The DPC does not make sound; on a read of
// $1005-$1007 it returns an AMPLITUDE, and the game writes that to the TIA's
// volume register itself. So this works on both boards, unlike the 7800 POKEY.
//
// Two deliberate differences from UnoCart:
//
//  1. No copy of the image. UnoCart memcpy()s the whole cart into fast CCM RAM
//     because on an STM32 it would otherwise be read from flash. rom_table is
//     already in SRAM here, so the copy would cost 10KB for nothing.
//
//  2. The music clock is read from the hardware timer instead of being polled.
//     UnoCart hangs an UPDATE_MUSIC_COUNTER macro off SysTick in EVERY branch of
//     the bus loop; the DPC's oscillator is just free-running (~20kHz - Stella's
//     default DPC pitch, AudioSettings.hxx:62 - and user-tunable there, so the
//     exact figure is not critical), which means the tick count can be derived
//     from elapsed time whenever it is actually needed. timer_hw->timerawl is a
//     free-running microsecond counter, so /50 gives 20kHz ticks. That takes the
//     music update out of the hot path entirely. The counter is only ever used
//     modulo a top value, so starting from boot rather than from game start just
//     shifts the phase.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_dpc_cartridge()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency

  // NOT static const: that would land in .rodata in FLASH and be read from the
  // bus loop through the XIP cache - the exact defect patches 0.09/0.13 removed.
  // Written out element by element so it is unambiguously on the stack, in SRAM.
  uint8_t soundAmplitudes[8];
  soundAmplitudes[0] = 0x00; soundAmplitudes[1] = 0x04;
  soundAmplitudes[2] = 0x05; soundAmplitudes[3] = 0x09;
  soundAmplitudes[4] = 0x06; soundAmplitudes[5] = 0x0a;
  soundAmplitudes[6] = 0x0b; soundAmplitudes[7] = 0x0f;

  // Zeroed by hand rather than with "= {0}": across four arrays that is 40 bytes,
  // and GCC decides a call to memset is cheaper - which puts a FLASH call in a
  // function that must not have one. Cold path (runs once, before the loop), but
  // a check_hotpath.sh FAIL we learn to ignore is worse than none at all.
  uint8_t  DpcTops[8], DpcBottoms[8], DpcFlags[8];
  uint16_t DpcCounters[8];
  DpcTops[0]=0; DpcTops[1]=0; DpcTops[2]=0; DpcTops[3]=0;
  DpcTops[4]=0; DpcTops[5]=0; DpcTops[6]=0; DpcTops[7]=0;
  DpcBottoms[0]=0; DpcBottoms[1]=0; DpcBottoms[2]=0; DpcBottoms[3]=0;
  DpcBottoms[4]=0; DpcBottoms[5]=0; DpcBottoms[6]=0; DpcBottoms[7]=0;
  DpcFlags[0]=0; DpcFlags[1]=0; DpcFlags[2]=0; DpcFlags[3]=0;
  DpcFlags[4]=0; DpcFlags[5]=0; DpcFlags[6]=0; DpcFlags[7]=0;
  DpcCounters[0]=0; DpcCounters[1]=0; DpcCounters[2]=0; DpcCounters[3]=0;
  DpcCounters[4]=0; DpcCounters[5]=0; DpcCounters[6]=0; DpcCounters[7]=0;

  uint32_t DpcRandom = 1;               // LFSR seed; must be non-zero
  uint32_t dpctop_music = 0, dpcbottom_music = 0;
  uint8_t  music_flags = 0, music_modes = 0;
  uint8_t  prev_rom = 0, prev_rom2 = 0;
  uint32_t addr, addr_prev = 0xFFFF, data = 0, data_prev = 0;
  uint8_t *bankPtr = &rom_table[0];
  uint8_t *DpcDisplayPtr = &rom_table[8*1024];

  // Music oscillator state. Kept as a per-voice phase advanced by elapsed time,
  // rather than one counter taken modulo each voice's top: a variable "%" on a
  // Cortex-M0+ is a call to __aeabi_uidivmod, which lives in FLASH and is reached
  // through a RAM veneer - the defect patches 0.09/0.13 removed and the reason
  // check_hotpath.sh exists. Subtraction in a BOUNDED loop costs less than the
  // call would and never leaves RAM. It is also closer to Stella, which likewise
  // keeps a counter per voice (CartDPC.cxx updateMusicModeDataFetchers).
  uint32_t music_last_us = timer_hw->timerawl;
  uint32_t music_acc = 0;               // microseconds not yet turned into ticks
  uint32_t mphase[3];                   // each < its voice's top
  mphase[0]=0; mphase[1]=0; mphase[2]=0;

  while (1) {
    while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
      addr_prev = addr;

    if (addr & 0x1000) {                // A12 high: the cartridge is selected
      if (addr < 0x1040) {              // ---- DPC register read ----
        uint32_t index    = addr & 0x07;
        uint32_t function = (addr >> 3) & 0x07;
        uint8_t  result   = 0;

        switch (function) {
          case 0x00:
            if (index < 4) {            // random number
              DpcRandom ^= DpcRandom << 3;
              DpcRandom ^= DpcRandom >> 5;
              result = (uint8_t)DpcRandom;
            } else {                    // music amplitude
              result = soundAmplitudes[music_modes & music_flags];
            }
            break;
          case 0x01:                    // display data
            result = DpcDisplayPtr[2047 - DpcCounters[index]];
            break;
          case 0x02:                    // display data AND'd with the flag
            result = DpcDisplayPtr[2047 - DpcCounters[index]] & DpcFlags[index];
            break;
          case 0x07:                    // flag register
            result = DpcFlags[index];
            break;
        }

        gpio_put_masked(DATA_PIN_MASK, (uint32_t)result << D0_PIN);
        SET_DATA_MODE_OUT;

        // Clock this data fetcher - but NOT the ones running in music mode,
        // which are driven by the oscillator instead. Done AFTER the byte is on
        // the bus, so none of it is in the response path.
        if (index < 5 || !(music_modes & (1 << (index - 5)))) {
          DpcCounters[index] = (DpcCounters[index] - 1) & 0x07FF;
          if ((DpcCounters[index] & 0x00FF) == DpcTops[index])
            DpcFlags[index] = 0xFF;
          else if ((DpcCounters[index] & 0x00FF) == DpcBottoms[index])
            DpcFlags[index] = 0x00;
        }

        while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
        SET_DATA_MODE_IN;
        addr_prev = 0xFFFF;
      }
      else if (addr < 0x1080) {         // ---- DPC register write ----
        uint32_t index    = addr & 0x07;
        uint32_t function = (addr >> 3) & 0x07;
        uint8_t  ctr      = DpcCounters[index] & 0xFF;

        // End-of-cycle capture: a 6502 drives the data lines only in the second
        // half of the cycle. Same shape as the proven F8SC path.
        while ((gpio_get_all()&BUS_PIN_MASK) == addr)
        { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK) >> D0_PIN; }
        addr_prev = 0xFFFF;

        uint8_t value = (uint8_t)data_prev;
        switch (function) {
          case 0x00:                    // top count
            DpcTops[index] = value;
            DpcFlags[index] = (ctr == value) ? 0xFF : 0x00;
            if (index >= 5)
              dpctop_music = (dpctop_music & ~(0xFFu << (8*(index-5))))
                           | ((uint32_t)value << (8*(index-5)));
            break;
          case 0x01:                    // bottom count
            DpcBottoms[index] = value;
            if (ctr == value) DpcFlags[index] = 0x00;
            if (index >= 5)
              dpcbottom_music = (dpcbottom_music & ~(0xFFu << (8*(index-5))))
                              | ((uint32_t)value << (8*(index-5)));
            break;
          case 0x02:                    // counter low
            DpcCounters[index] = (DpcCounters[index] & 0x0700) | value;
            if (value == DpcTops[index])         DpcFlags[index] = 0xFF;
            else if (value == DpcBottoms[index]) DpcFlags[index] = 0x00;
            break;
          case 0x03:                    // counter high (+ music mode bit)
            DpcCounters[index] = (((uint16_t)(value & 0x07)) << 8) | ctr;
            if (index >= 5)
              music_modes = (music_modes & ~(0x01 << (index-5)))
                          | ((value & 0x10) >> (9 - index));
            break;
          case 0x06:                    // reset the random number generator
            DpcRandom = 1;
            break;
        }
      }
      else {                            // ---- plain ROM, with F8 bankswitch ----
        if      (addr == 0x1FF8) bankPtr = &rom_table[0];
        else if (addr == 0x1FF9) bankPtr = &rom_table[4*1024];

        gpio_put_masked(DATA_PIN_MASK, (uint32_t)bankPtr[addr&0xFFF] << D0_PIN);
        SET_DATA_MODE_OUT;

        prev_rom2 = prev_rom;
        prev_rom = bankPtr[addr&0xFFF];

        while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
        SET_DATA_MODE_IN;
        addr_prev = 0xFFFF;
      }
    }
    // Below $1000 the console owns the bus. UnoCart uses this window to refresh
    // the music flags, recognising the moment by the two bytes the cart last
    // handed over: (prev_rom2 & 0xDC) == 0x84 matches the zero-page LDA/LDX/LDY
    // and STA/STX/STY opcodes, and prev_rom - the operand byte - equalling the
    // address now on the bus confirms this really is that instruction's zero-page
    // access rather than a coincidence. It is a heuristic, but it is the one that
    // ships in a working cartridge, and it costs nothing when it does not match.
    else if (((prev_rom2 & 0xDC) == 0x84) && prev_rom == addr) {
      // Advance the oscillator by however long it has been. 50us per tick is the
      // ~20kHz Stella uses by default. This branch is entered on nearly every
      // zero-page access, so the gap is normally a handful of microseconds and
      // both loops below run once or not at all; the bounds only matter if the
      // game goes a long time without one, and losing a few ticks of music is a
      // far better failure than stalling the bus.
      uint32_t now = timer_hw->timerawl;
      music_acc += now - music_last_us;          // unsigned: wraps correctly
      music_last_us = now;
      // Bounded at 16 ticks. That bound sets the worst case for the per-voice
      // loops below: each runs at most (16 / top) + 1 times, so 17 iterations for
      // top=1 and three voices is under 1us even then - about one 2600 bus cycle.
      // A bound of 64 would have allowed ~3us here, i.e. missing up to four bus
      // cycles, which is not worth it: this branch is entered on nearly every
      // zero-page access, thousands of times per frame, so ticks is realistically
      // 0 or 1 and the bound never binds.
      uint32_t ticks = 0;
      while (music_acc >= 50 && ticks < 16) { music_acc -= 50; ticks++; }
      if (music_acc >= 50) music_acc = 0;        // fell far behind - resync

      uint32_t tops    = dpctop_music;
      uint32_t bottoms = dpcbottom_music;
      music_flags = 0;
      for (uint32_t v = 0; v < 3; v++) {
        uint32_t top    = (tops    >> (8*v)) & 0xFF;
        uint32_t bottom = (bottoms >> (8*v)) & 0xFF;
        if (top) {
          uint32_t p = mphase[v] + ticks;
          while (p >= top) p -= top;             // bounded: ticks <= 64
          mphase[v] = p;
          if (p > bottom) music_flags |= (1 << v);
        } else {
          mphase[v] = 0;
        }
      }
      while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
      addr_prev = 0xFFFF;
    }
  }
}

// not_working_roms4: CART_TYPE_NORMALA78 and CART_TYPE_ABSOLUTE are just as
// real a 7800/MARIA game as CART_TYPE_SUPERCART_RAM etc. (same tight DMA
// response window), but - unlike every SuperGame/Activision/Supercharger path
// above - never got the 0.09-0.13 hardening, because those patches were
// scoped to specific bug reports (Alien Brigade, Ikari Warriors, Double
// Dragon, Rampage), not a full audit of every cart type. They ran inline
// inside setup1() instead, which IS built at -O3 (see the #pragma block
// below) - so the missing piece here is not optimization level, it's:
//   1. cpsid i - setup1() itself never disables core-1 IRQs, so every inline
//      case (including these two) was exposed the whole game, not just once
//      at startup.
//   2. gpio_put_masked() - even at -O3 this compiles to a call into a small
//      shared RAM-resident helper (verified with check_hotpath.sh: no flash
//      veneer here, unlike the pre-0.09 bug, but still a call+return on every
//      single bus response) instead of the single inlined SIO store the
//      functions above use.
// User-visible motivation: "7800 XMAS" (SUPERCART_RAM, hardened) showed
// visibly fewer in-game artifacts than CART_TYPE_NORMALA78 titles (2600 Maze
// Pac-Man, 3D Worldrunner) on the same hardware. Pure extraction otherwise:
// logic (including the bugs/b01 16k A14+A15 fix) is unchanged, only where it
// lives and how it answers the bus changed. Diagnostic - not yet confirmed
// on hardware to actually reduce artifacts; see not_working_roms4/README.md.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_normala78()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  uint32_t addr, addr_prev = 0;
  // not_working_roms4 "Krok 21": one general mapping instead of three hardcoded
  // size cases. Straight from MAME's a78_rom_device::read_40xx (rom.cpp:257-263),
  // which is the authority for every flat 7800 cart:
  //
  //     m_base_rom = 0x10000 - size;
  //     if (offset + 0x4000 < m_base_rom) return 0xff;      // open bus
  //     else return m_rom[offset + 0x4000 - m_base_rom];
  //
  // i.e. a flat cart is mapped to the TOP of the address space and stays silent
  // below its own start. The old code hardcoded that for exactly three sizes -
  // 16k/32k/48k - and any other size fell through the if/else chain and drove
  // nothing at all, which is a white/blank screen. In this library that silently
  // broke "7ix" (28KB) and "Bouncing Balls (Demo)" (8KB).
  //
  // Verified equivalent to the old code for all three sizes it did handle:
  //   16k: base=0xC000 -> answers $C000-$FFFF, index addr-0xC000 == addr&0x3FFF
  //   32k: base=0x8000 -> answers $8000-$FFFF, index addr-0x8000 == addr&0x7FFF
  //   48k: base=0x4000 -> answers $4000-$FFFF, index addr-0x4000, which for
  //        $4000-$7FFF equals the old (addr&0x7fff-0x4000) - that expression is
  //        really addr&0x3FFF, since 0x7fff-0x4000 binds first.
  // The 16k case keeps the bugs/b01 behaviour (silent in $4000-$BFFF, so a POKEY
  // at $4000 is not fought over) - it falls out of the same formula.
  //
  // The 0x4000 floor is a safety net, not part of MAME's formula: the cartridge
  // slot only decodes $4000-$FFFF, so a hypothetical >=64KB flat image must never
  // make this drive over TIA/RIOT/console RAM below $4000.
  const uint32_t base_rom = (romLen >= 0x10000) ? 0x4000 : (0x10000 - (uint32_t)romLen);
  const uint32_t lo = (base_rom < 0x4000) ? 0x4000 : base_rom;

  while (1) {
    while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
      addr_prev = addr;
    // got a stable address
    if (addr >= lo) {
      sio_hw->gpio_out = (uint32_t)rom_table[addr - base_rom] << D0_PIN;  // D0-D7 are the only outputs in 7800 modes
      SET_DATA_MODE_OUT;
      // wait for address bus to change
      while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
      SET_DATA_MODE_IN;
    }
  }
}

// POKEY-cart variant of the loop above. A SEPARATE function on purpose: the plain
// path took several hardware iterations to get right (see not_working_roms4/), and
// nothing here is worth risking a regression on the other ~76 NORMALA78 titles.
// The only addition is capturing writes to $4000-$400F into pokey_regs[]. The cart
// never DRIVES that range for a flat ROM (lo is $8000 for 32k, $C000 for 16k), so
// this cannot collide with the ROM window - it only listens.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_normala78_pokey()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  uint32_t addr, addr_prev = 0;
  const uint32_t base_rom = (romLen >= 0x10000) ? 0x4000 : (0x10000 - (uint32_t)romLen);
  const uint32_t lo = (base_rom < 0x4000) ? 0x4000 : base_rom;
  const uint32_t pkbase = (uint32_t)pokey_base;   // volatile: hoist out of the loop
  const uint32_t pkmask = (uint32_t)pokey_mask;
  const uint32_t ymon   = (uint32_t)ym_enabled;  // ditto - never read in the loop

  while (1) {
    while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
      addr_prev = addr;
    // got a stable address
    // ROM window is tested FIRST, so a cart whose ROM reaches down to $4000 (a 48k
    // flat image) can never have its data stolen by the POKEY window. No such cart
    // declares POKEY today; if one ever does, it silently gets no sound rather than
    // a broken picture, which is the right way round.
    if (addr >= lo) {
      sio_hw->gpio_out = (uint32_t)rom_table[addr - base_rom] << D0_PIN;
      SET_DATA_MODE_OUT;
      while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
      SET_DATA_MODE_IN;
    } else if ((addr & pkmask) == pkbase && ymon) {
      // YM2151 window ($0460/$0461). This one CANNOT be listen-only: 32 of the 45
      // YM carts sit in "BIT $0461 / BMI *-3" waiting for the BUSY bit to clear
      // and write nothing until they see it. ym_window_service_blocking() answers
      // in the same drive/wait/release shape this loop uses for ROM, so it does
      // not change how the loop behaves - see ym2151.h.
      ym_window_service_blocking(addr);
    } else if ((addr & pkmask) == pkbase) {      // POKEY register window
      // LISTEN ONLY - deliberately no read support on this path.
      // Reads were tried here and REGRESSED "3D Worldrunner Theme Melody (4000)",
      // which had worked with write-only capture. Driving the bus gated on R/W is
      // the same shape that hung the cart in Krok 18: one mis-read of R/W turns a
      // POKEY write into "drive against the CPU and then block". The SuperGame
      // variants can afford reads because theirs never block; this one cannot.
      // Nothing was lost by reverting: Ballblazer and 3D Worldrunner both work
      // write-only, and reads were only added to chase 7800 XMAS, which turned out
      // not to touch a POKEY register at all (0 references in 128KB).
      {
        // End-of-cycle capture, same proven shape as version 0.15: a 6502 does not
        // drive the data lines until the second half of the cycle. Bounded at 64.
        uint32_t last = gpio_get_all(), cur;
        for (uint32_t g = 0; g < 64; g++) {
          cur = gpio_get_all();
          if ((cur & BUS_PIN_MASK) != addr) break;
          last = cur;
        }
        pokey_regs[addr & 0x0F] = (uint8_t)((last >> D0_PIN) & 0xFF);
      }
    }
  }
}

// mRAM ("masked RAM") - MAME's A78_TYPE8, test7800's external/mram.go. A FLAT
// cart plus 16KB of on-cart RAM at $4000-$7FFF. MAME's rom.cpp describes the
// board as "no bankswitch + mRAM chip"; it is selected by bit 7 of the a78
// header's low byte, which OVERRIDES whatever bankswitch bits are also set
// (a78_slot.cpp:493) - "Turrican II Circular Scroll Test" (header 0x0082) has
// the SuperGame bit on and is still an mRAM cart.
//
// Two halves, each already proven on this hardware, joined:
//   * the ROM window is emulate_normala78()'s general mapping (version 0.16,
//     from MAME rom.cpp:257-263): a flat cart sits at the TOP of the address
//     space, base_rom = 0x10000 - size, and stays silent below its own start.
//   * the RAM window is emulate_supercart_ram()'s $4000 path, including the
//     Krok 19 end-of-cycle write capture.
//
// The one new element is the address mask. test7800 (mram.go):
//
//     if address < 0x8000 { address &= 0xfeff; ram[address-0x4000] }
//
// A8 is not connected, so $4100-$41FF is the same storage as $4000-$40FF - that
// is where the name comes from. For the $4000-$7FFF range this branch handles,
// (address & 0xfeff) - 0x4000 is exactly (address & 0x3eff), so the mask is
// folded into the index. Highest index reached is 0x3EFF, inside ram_table.
//
// Unlike emulate_normala78() this loop does NOT block while the address is
// stable: it must keep servicing the RAM window, and "blocking ROM path + R/W
// gating" is precisely the combination that regressed 3D Worldrunner in 0.18.
// Shaped like emulate_supercart_ram(), which never blocks.
//
// No _pokey variant on purpose: every mRAM file in this library has header
// 0x0080 or 0x0082, i.e. none declares a POKEY, and an untested variant would
// cost RAM on a board that is already 95% full.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_mram()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  const uint32_t base_rom = (romLen >= 0x10000) ? 0x4000 : (0x10000 - (uint32_t)romLen);
  const uint32_t lo = (base_rom < 0x4000) ? 0x4000 : base_rom;
  uint32_t addr = 0, rawaddr = 0;
  uint8_t rom_in_use = 1;

  while (1) {
    rawaddr = gpio_get_all();
    addr = rawaddr & BUS_PIN_MASK;
    if (addr >= lo) {                       // flat ROM at the top of the map
        sio_hw->gpio_out = (uint32_t)rom_table[addr - base_rom] << D0_PIN;  // D0-D7 are the only outputs in 7800 modes
        if ((gpio_get_all() & RW_PIN_MASK) == RW_PIN_MASK) {
            if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else if (rom_in_use) {
            SET_DATA_MODE_IN; rom_in_use = 0;
        }
    } else if (addr & A14_PIN_MASK) {       // $4000-$7FFF: the mRAM window
        sio_hw->gpio_out = (uint32_t)ram_table[addr & 0x3EFF] << D0_PIN;
        rawaddr = gpio_get_all() & (RW_PIN_MASK | A14_PIN_MASK);
        if (rawaddr == (RW_PIN_MASK | A14_PIN_MASK)) {
            if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else if (rawaddr == A14_PIN_MASK) {
            SET_DATA_MODE_IN;
            // Krok 19 end-of-cycle capture - see emulate_supercart_ram().
            uint32_t wlast = gpio_get_all(), wcur;
            uint32_t waddr = wlast & BUS_PIN_MASK;
            for (uint32_t g = 0; g < 64; g++) {
                wcur = gpio_get_all();
                if ((wcur & BUS_PIN_MASK) != waddr) break;
                wlast = wcur;
            }
            ram_table[waddr & 0x3EFF] = (wlast >> D0_PIN) & 0xff;
            rom_in_use = 0;
        } else if (rom_in_use) {
            SET_DATA_MODE_IN; rom_in_use = 0;
        }
    } else if (rom_in_use) {                // below $4000, or the gap under a 16K ROM
        SET_DATA_MODE_IN; rom_in_use = 0;
    }
  }
}

__attribute__((optimize("O2")))
void __time_critical_func(emulate_absolute()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  // Continually check address lines and put associated data on bus.
  uint32_t addr;
  uint32_t bank=0x4000;
  while (1) {       // Get address
    addr = gpio_get_all()&BUS_PIN_MASK;
    if (addr == 0x8000) {  // TO CHECK
        // Bankswitching write
       // Check for 0x01
      SET_DATA_MODE_IN;
      // Krok 20: end-of-cycle capture - see emulate_supercart_ef() for the reasoning.
      // No CART_TYPE_ABSOLUTE file exists in the current ROMS/ library, so this one
      // is corrected for consistency and cannot be verified on hardware yet.
      uint32_t alast = gpio_get_all(), acur;
      for (uint32_t g = 0; g < 64; g++) {
          acur = gpio_get_all();
          if ((acur & BUS_PIN_MASK) != addr) break;
          alast = acur;
      }
      uint8_t data = (alast & DATA_PIN_MASK)>>D0_PIN;
      if (data == 0x01) {
        bank = 0;   // Switch to flying mode
      } else {
        if (data == 0x02) {
          bank = 0x4000; // Switch to title page
        }
      }
    } else if (addr & 0x8000) {        // Check for A15
        sio_hw->gpio_out = (uint32_t)rom_table[addr] << D0_PIN;  // D0-D7 are the only outputs in 7800 modes
        SET_DATA_MODE_OUT;
       	while ((gpio_get_all()&BUS_PIN_MASK) == addr);
        SET_DATA_MODE_IN;
        // Check for RW
      } else {    // Check for A14
        if (addr & 0x4000) {
            // Set the data on the bus
            sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + bank] << D0_PIN;  // D0-D7 are the only outputs in 7800 modes
            SET_DATA_MODE_OUT;
         		while ((gpio_get_all()&BUS_PIN_MASK) == addr);
            SET_DATA_MODE_IN;
        }
      }
    }
}



// POKEY-cart variant of emulate_supercart_ef(). A SEPARATE function so that carts without
// POKEY keep byte-identical code to the version proven on hardware.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_ef_pokey()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      // Hoisted: pokey_base is volatile, and reading it from memory on every
      // pass through the cold branch is exactly what made Alien Brigade (a cart
      // with no POKEY at all) start glitching.
      const uint32_t pkbase = (uint32_t)pokey_base;
      const uint32_t pkmask = (uint32_t)pokey_mask;
      uint32_t bank=0, addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      // Bank-number mask derived from the real ROM size - see the identical
      // comment in emulate_supercart_ram() for the full reasoning and sources.
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));

      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (addr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                sio_hw->gpio_out = (uint32_t)rom_table[addr + 0x10000] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + bank] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                        // Krok 20: end-of-cycle capture, proven on CART_TYPE_SUPERCART_RAM
                        // in Krok 19. The 6502 does not drive the data lines until the
                        // second half of a write cycle, and this loop polls, so it spots
                        // the write at a random phase - sampling here reads the bus before
                        // the CPU has driven it roughly half the time. Keep the last value
                        // seen while the address was still valid. Bounded at 64 turns
                        // (~2x one 6502 cycle at 250MHz): an unbounded wait hung the cart
                        // in Krok 18. Note the 2600 paths in setup1() have always used
                        // this shape ("while (addr unchanged) { data_prev = data; ... }");
                        // only the 7800 SuperGame paths were missing it.
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        bank=((last >> D0_PIN) & sc_bank_mask)*0x4000;  // was & 0xf - see the mask comment above
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            // EXFIX - bank 6 is in 0x4000
            if (addr & 0x4000) {
              // POKEY @$4000 (byte54 bit0) replaces this window entirely. MAME:
              // a78_rom_sg_pokey_device::read_40xx returns m_pokey->read(offset & 0x0f)
              // for the whole $4000-$7FFF range and write_40xx sends writes there to
              // the chip - the bank-6 ROM below is the NON-POKEY SuperGame layout.
              // pkbase is a hoisted constant, so this costs one register compare and
              // only inside the POKEY variant; plain SuperGame carts never see it.
              if (pkbase == 0x4000) {
                pokey_window_service(addr, &rom_in_use);   // mirrors every 16 bytes
              } else {
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + 0x18000] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
              }   // end of the non-POKEY $4000-$7FFF branch
            } else {
                // $0000-$3FFF. The $0450 POKEY window lives here, below every
                // SuperGame window, so it cannot collide with the ROM/RAM paths.
                if ((addr & pkmask) == pkbase) {
                    pokey_window_service(addr, &rom_in_use);
                } else if (rom_in_use) {
                    SET_DATA_MODE_IN;
                    rom_in_use = 0;
                }
            }
        }
      }
    }

// POKEY-cart variant of emulate_supercart_ram(). A SEPARATE function so that carts without
// POKEY keep byte-identical code to the version proven on hardware.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_ram_pokey()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      // Hoisted: pokey_base is volatile, and reading it from memory on every
      // pass through the cold branch is exactly what made Alien Brigade (a cart
      // with no POKEY at all) start glitching.
      const uint32_t pkbase = (uint32_t)pokey_base;
      const uint32_t pkmask = (uint32_t)pokey_mask;
      uint32_t bank=0;
      // v0.13 (P1): romLen is volatile, so the fixed-bank index below was
      // re-read from memory on every pass. Constant for the whole game - hoist.
      const uint32_t fixed_base = (uint32_t)romLen - 0x8000;
      uint32_t addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      // not_working_roms4 "Przypadek 3": the bank number latched on a $8000-$BFFF
      // write used to be masked with a fixed "& 0xf", i.e. 16 banks, no matter how
      // big the cart actually is. Both reference implementations bound it to the
      // real ROM instead:
      //   MAME  a78_slot.cpp:74-77 + rom.cpp:388 - m_bank = data & m_bank_mask,
      //         where m_bank_mask = (nbanks odd) ? nbanks-2 : nbanks-1;
      //   ProSystem/JS7800 Cartridge.js:729 - the write is IGNORED entirely unless
      //         cartridge_GetBank(data) < size/16384.
      // Every SUPERCART_RAM cart in this library is 128KB = 8 banks, so the old
      // mask let a bankswitch write select banks 8-15, which do not exist: bank 8
      // reads whatever the previously loaded game left in rom_table (it is never
      // cleared between loads), banks 9-15 index past the 144KB array altogether -
      // straight into the TinyUSB descriptors, per CLAUDE.md. That matters even for
      // a well-behaved ROM, because this firmware samples the data bus on a write at
      // a moment of its own choosing rather than on the CPU's write strobe: a single
      // misread D3 turns a legal "select bank 5" into "show 16KB of garbage until the
      // next bankswitch" - transient corruption that appears and disappears, which is
      // exactly the reported symptom. Pico2A10400 already hardcoded "& 0x07" here;
      // deriving the mask keeps both boards correct for 4/8/9/16-bank carts alike.
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));
      
      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (addr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + fixed_base] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                //sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + bank] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + bank] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                        // not_working_roms4 "Krok 19": a 6502 puts address and R/W up at
                        // the start of a cycle but does not drive the data lines until its
                        // second half, so grabbing the byte here - a few instructions after
                        // spotting the write - reads the bus before the CPU has driven it.
                        // Because this loop polls, it catches the write at a random phase,
                        // so the byte is sometimes good and sometimes garbage: intermittent
                        // wrong-bank corruption, i.e. 16KB of the wrong graphics until the
                        // next bankswitch. Take instead the LAST sample seen while the
                        // address was still valid (= end of the write cycle, data settled).
                        // BOUNDED on purpose: Krok 18 used an unbounded wait here and the
                        // cart hung (yellow screen). One 6502 cycle at 250MHz is ~140 Pico
                        // cycles and this spin is ~5 cycles per turn, so ~28 turns covers a
                        // whole bus cycle; 64 gives 2x headroom while capping the time we
                        // can ever stop serving the bus at well under 1.5us.
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        bank=((last >> D0_PIN) & sc_bank_mask)*0x4000;  // was & 0xf
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            rawaddr=gpio_get_all();
            // EXram - 16k is in 0x4000
            if (rawaddr & 0x4000) {
                addr= rawaddr & 0x3fff;
                sio_hw->gpio_out = (uint32_t)ram_table[addr] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                        // Krok 19: same end-of-cycle capture as the bank register above.
                        // This path corrupts whatever the game just stored in on-cart RAM,
                        // which for these carts is graphics - so a byte sampled before the
                        // 6502 drives it shows up directly on screen. 'addr' has already
                        // been narrowed to a 14-bit offset here, so compare a full-width
                        // address captured now.
                        uint32_t wlast = gpio_get_all(), wcur;
                        uint32_t waddr = wlast & BUS_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS_PIN_MASK) != waddr) break;
                            wlast = wcur;
                        }
                        ram_table[waddr & 0x3fff] = (wlast >> D0_PIN) & 0xff;
                        rom_in_use = 0;
                  } else {
                    if (rom_in_use) {
                        SET_DATA_MODE_IN;
                        rom_in_use = 0;
                    }
                  }
                }
            } else {
                // $0000-$3FFF. The $0450 POKEY window lives here, below every
                // SuperGame window, so it cannot collide with the ROM/RAM paths.
                if ((addr & pkmask) == pkbase) {
                    pokey_window_service(addr, &rom_in_use);
                } else if (rom_in_use) {
                    SET_DATA_MODE_IN;
                    rom_in_use = 0;
                }
            }
        }
      }
}

// VersaBoard (CPUWIZ homebrew board) - MAME cpuwiz.cpp:83-104. This is
// emulate_supercart_ram() with ONE addition: the 16KB RAM window at $4000-$7FFF
// is banked, and the second bank is selected by bit 5 of the same write that
// selects the ROM bank:
//
//     write_40xx: offset < 0x4000 -> m_ram[offset + m_ram_bank * 0x4000];
//                 offset < 0x8000 -> m_bank     = (data & 0x0f) & m_bank_mask;
//                                    m_ram_bank = BIT(data, 5);
//     read_40xx:  offset < 0x4000 -> m_ram[offset + m_ram_bank * 0x4000];
//                 offset < 0x8000 -> m_rom[(offset & 0x3fff) + m_bank * 0x4000];
//                 else            -> m_rom[(offset & 0x3fff) + m_bank_mask * 0x4000];
//
// (MAME's `offset` is relative to $4000, so its "offset < 0x4000" is our
// $4000-$7FFF RAM window and its "< 0x8000" is our $8000-$BFFF bank window.)
//
// ram_table is 32KB - exactly the 2 x 16KB this board has, so no allocation
// change is needed. The "& 0x0f" is MAME's; for the 128KB carts this type
// actually has it is already implied by sc_bank_mask (7), but keeping it
// explicit stays correct for the 256KB/16-bank configurations the board allows.
//
// MegaCart+ is the same board with a 5-bit bank mask and up to 512KB of ROM;
// it is deliberately NOT implemented, because 512KB cannot fit rom_table's
// 144KB and no file in this library needs it (MAME picks MegaCart over
// VersaBoard only when the payload exceeds 256KB - a78_slot.cpp:424).
__attribute__((optimize("O2")))
void __time_critical_func(emulate_versa()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      uint32_t bank=0, ram_bank=0;
      const uint32_t fixed_base = (uint32_t)romLen - 0x8000;
      uint32_t addr=0, rawaddr=0;
      uint8_t rom_in_use=1;
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));

      while (1) {
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        if (addr & A15_PIN_MASK) {
            if (addr & A14_PIN_MASK) {
                // $C000-$FFFF: fixed last bank
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + fixed_base] << D0_PIN;
                rawaddr = gpio_get_all() & READ_PIN_MASK;
                if (rawaddr == READ_PIN_MASK) {
                    if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
                }
            } else {
                // $8000-$BFFF: switchable bank, and the bank/RAM-bank register
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + bank] << D0_PIN;
                rawaddr = gpio_get_all() & READ_PIN_MASK;
                if (rawaddr == (RW_PIN_MASK | A15_PIN_MASK)) {
                    if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
                } else {
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    if (rawaddr == A15_PIN_MASK) {
                        SET_DATA_MODE_IN;
                        // Krok 19 end-of-cycle capture - see emulate_supercart_ram().
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        uint32_t d = (last >> D0_PIN) & 0xff;
                        bank     = (d & 0x0f & sc_bank_mask) * 0x4000;
                        ram_bank = (d & 0x20) ? 0x4000 : 0;   // BIT(data, 5)
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            rawaddr = gpio_get_all();
            if (rawaddr & 0x4000) {
                // $4000-$7FFF: banked on-cart RAM
                addr = rawaddr & 0x3fff;
                sio_hw->gpio_out = (uint32_t)ram_table[addr + ram_bank] << D0_PIN;
                rawaddr = gpio_get_all() & (RW_PIN_MASK | A14_PIN_MASK);
                if (rawaddr == (RW_PIN_MASK | A14_PIN_MASK)) {
                    if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
                } else {
                  if (rawaddr == A14_PIN_MASK) {
                        SET_DATA_MODE_IN;
                        uint32_t wlast = gpio_get_all(), wcur;
                        uint32_t waddr = wlast & BUS_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS_PIN_MASK) != waddr) break;
                            wlast = wcur;
                        }
                        ram_table[(waddr & 0x3fff) + ram_bank] = (wlast >> D0_PIN) & 0xff;
                        rom_in_use = 0;
                  } else {
                    if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
                  }
                }
            } else {
                if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
            }
        }
      }
}

// POKEY-cart variant of emulate_versa(). A SEPARATE function for the same reason
// as every other _pokey variant (see version 0.18): pokey_base is volatile, so a
// shared helper re-reads it from memory on every pass through the cold branch -
// the branch every console-RAM, TIA and MARIA access takes - which is what made
// "Alien Brigade" glitch. MAME gives this board its own device, a78_versapokey
// (cpuwiz.cpp:41), because a few VersaBoard demos combine banked RAM with a POKEY
// at $0450 for XBoarD/XM compatibility. In this library that is exactly one file:
// "Mario Bros (Ice Stress Test)", header 0x0062.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_versa_pokey()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      uint32_t bank=0, ram_bank=0;
      const uint32_t fixed_base = (uint32_t)romLen - 0x8000;
      uint32_t addr=0, rawaddr=0;
      uint8_t rom_in_use=1;
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));
      // Hoisted: see emulate_supercart_ram_pokey().
      const uint32_t pkbase = (uint32_t)pokey_base;
      const uint32_t pkmask = (uint32_t)pokey_mask;

      while (1) {
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        if (addr & A15_PIN_MASK) {
            if (addr & A14_PIN_MASK) {
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + fixed_base] << D0_PIN;
                rawaddr = gpio_get_all() & READ_PIN_MASK;
                if (rawaddr == READ_PIN_MASK) {
                    if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
                }
            } else {
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x7fff) + bank] << D0_PIN;
                rawaddr = gpio_get_all() & READ_PIN_MASK;
                if (rawaddr == (RW_PIN_MASK | A15_PIN_MASK)) {
                    if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
                } else {
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    if (rawaddr == A15_PIN_MASK) {
                        SET_DATA_MODE_IN;
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        uint32_t d = (last >> D0_PIN) & 0xff;
                        bank     = (d & 0x0f & sc_bank_mask) * 0x4000;
                        ram_bank = (d & 0x20) ? 0x4000 : 0;   // BIT(data, 5)
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            rawaddr = gpio_get_all();
            if (rawaddr & 0x4000) {
                addr = rawaddr & 0x3fff;
                sio_hw->gpio_out = (uint32_t)ram_table[addr + ram_bank] << D0_PIN;
                rawaddr = gpio_get_all() & (RW_PIN_MASK | A14_PIN_MASK);
                if (rawaddr == (RW_PIN_MASK | A14_PIN_MASK)) {
                    if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
                } else {
                  if (rawaddr == A14_PIN_MASK) {
                        SET_DATA_MODE_IN;
                        uint32_t wlast = gpio_get_all(), wcur;
                        uint32_t waddr = wlast & BUS_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS_PIN_MASK) != waddr) break;
                            wlast = wcur;
                        }
                        ram_table[(waddr & 0x3fff) + ram_bank] = (wlast >> D0_PIN) & 0xff;
                        rom_in_use = 0;
                  } else {
                    if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
                  }
                }
            } else {
                // $0000-$3FFF. The $0450 POKEY window lives here, below every
                // VersaBoard window, so it cannot collide with the ROM/RAM paths.
                if ((addr & pkmask) == pkbase) {
                    pokey_window_service(addr, &rom_in_use);
                } else if (rom_in_use) {
                    SET_DATA_MODE_IN; rom_in_use = 0;
                }
            }
        }
      }
}

// POKEY-cart variant of emulate_supercart_large(). A SEPARATE function so that carts without
// POKEY keep byte-identical code to the version proven on hardware.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_large_pokey()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      // Hoisted: pokey_base is volatile, and reading it from memory on every
      // pass through the cold branch is exactly what made Alien Brigade (a cart
      // with no POKEY at all) start glitching.
      const uint32_t pkbase = (uint32_t)pokey_base;
      const uint32_t pkmask = (uint32_t)pokey_mask;
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
                        // Krok 20: end-of-cycle capture - see emulate_supercart_ef() above
                        // for the full reasoning. Directly relevant here: the comment below
                        // notes Alien Brigade writes values it loaded from memory, so a
                        // half-driven bus sampled too early is exactly how a legal bank
                        // number turns into a wrong one.
                        uint32_t last = gpio_get_all(), cur;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS_PIN_MASK) != addr) break;
                            last = cur;
                        }
                        rawaddr = last;
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
                // $0000-$3FFF. The $0450 POKEY window lives here, below every
                // SuperGame window, so it cannot collide with the ROM/RAM paths.
                if ((addr & pkmask) == pkbase) {
                    pokey_window_service(addr, &rom_in_use);
                } else if (rom_in_use) {
                    SET_DATA_MODE_IN;
                    rom_in_use = 0;
                }
            }
        }
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
  
  // A YM2151 cart needs the higher clock whatever its mapper happens to be, so
  // this test is on ym_enabled and NOT nested inside the one below.
  // 2026-08-24: it was nested, and "cart_to_emulate>=35" silently excluded
  // CART_TYPE_NORMALA78 (33) - which is what 29 of the 45 YM carts in this
  // library are, "OutRun - Last Wave" and "UN Squadron" among them. Those two
  // were reported still warbling after the clock was supposedly raised; they had
  // in fact been getting neither the clock NOR the 1.30V, and YM_LOG.TXT still
  // reading clk_sys=250000 is what gave it away.
  if (ym_enabled) {
   vreg_set_voltage(VREG_VOLTAGE_1_30);
   int ret=set_sys_clock_khz(YM_CLOCK_KHZ, true);
  } else if (cart_to_emulate>=35) {
   vreg_set_voltage(VREG_VOLTAGE_1_30);
   int ret=set_sys_clock_khz(EMU_CLOCK_KHZ, true);
  }
  if (cart_to_emulate<=32) {
   // Raise the core voltage for 2600 emulation too. setup() runs the whole chip
   // at 250MHz on VREG_VOLTAGE_1_15 - an ~88% overclock over the RP2040's nominal
   // 133MHz, at a voltage its own comment flags as marginal ("set to 1_15 or 1_20
   // if you experience some glitches"). The 7800 dispatch above already bumps to
   // 1_30 before emulating; the 2600 dispatch never did. Voltage only: the clock
   // is already 250MHz from setup(), so set_sys_clock_khz() is not repeated.
   vreg_set_voltage(VREG_VOLTAGE_1_30);
   delay(2);   // let the regulator settle before the bus loop starts
   gpio_set_dir_out_masked(BUS_H_PIN_MASK);
  }
  // Clock and voltage are now whatever this cart is going to run at, so core 0
  // may start synthesising. See emu_clock_ready in ym2151.h.
  emu_clock_ready = 1;

  // AR/Supercharger is the ONE cart type whose "ROM" is not already sitting in
  // rom_table by this point. Every other type had its full image loaded by
  // identify_cartridge() before we got here, so the reset vector at $1FFC/$1FFD
  // is valid. AR's cartridge ROM is the 311-byte mini-BIOS, and it was being
  // installed by setup_rom() INSIDE emulate_supercharger_cartridge() - i.e.
  // AFTER the reboot below. Since reboot_cartridge() feeds the 6502 a
  // JMP ($FFFC), the CPU was reading its reset vector out of whatever the
  // previous game or the menu kernel had left at that offset. All FOUR
  // reference implementations in ORIG/ get this right - UnoCart-2600,
  // DirtyHairy-UnoCart-2600, United-Carts-of-Atari and PlusCart-Pico all call
  // reboot AFTER setup_rom()/setup_multiload_map(). Restore that ordering.
  if (cart_to_emulate == CART_TYPE_AR) setup_supercharger();

  reboot_cartridge(addr,addr_prev);
  
  //exit_cartridge(addr,addr_prev);
 
  switch (cart_to_emulate) {

     case CART_TYPE_ACTIVISION:
      emulate_activision();
        break;

     case CART_TYPE_SUPERCART_RAM: 
      if (pokey_enabled) emulate_supercart_ram_pokey(); else emulate_supercart_ram();
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
      if (pokey_enabled) emulate_supercart_ef_pokey();
      else               emulate_supercart_ef();
        break;
    
    case CART_TYPE_NORMALA78:
      // POKEY carts get the listening variant; everything else keeps the plain,
      // hardware-proven loop untouched.
      if (pokey_enabled) emulate_normala78_pokey();
      else               emulate_normala78();
    break;

    case CART_TYPE_ABSOLUTE:
      emulate_absolute();
       break;

    case CART_TYPE_MRAM:
      emulate_mram();
    break;

    case CART_TYPE_VERSA:
      // Only "Mario Bros (Ice Stress Test)" (header 0x0062) takes the _pokey path
      // in this library; the other four VersaBoard files declare no POKEY.
      if (pokey_enabled) emulate_versa_pokey();
      else               emulate_versa();
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

    // 4K + SuperChip RAM. Detected since forever (isProbably4KSC(), reached only
    // for a 4096-byte image) but until now it fell through the switch and the
    // cart answered nothing. This is CART_TYPE_4K above with the SuperChip block
    // from CART_TYPE_F8SC dropped in - 128 bytes of RAM mirrored as write port
    // $1000-$107F and read port $1080-$10FF - and no bankswitching, because a 4K
    // cart has exactly one bank. Both halves are hardware-proven; only the
    // combination is new.
    //
    // NOTE: this library contains no real 4KSC GAME to verify it with. All 112
    // matching files are single 4KB banks of one batari Basic multikernel
    // framework (their first 256 bytes are 0xFF padding, which is what satisfies
    // the "256 identical bytes" half of isProbably4KSC, and their reset vectors
    // point into a bankswitch trampoline). See TEST_ROMS_2/README.md.
    case CART_TYPE_4KSC:
      data=0; data_prev=0;
      while (1) {
		    while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
			  addr_prev = addr;
   	    // got a stable address
		    if (addr & 0x1000) 	{ // A12 high
          if ((addr & 0x1F00) == 0x1000) {	// SC RAM access
            if (addr & 0x0080) {	// a read from cartridge ram
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
          } else {	// normal rom access
            gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0xFFF]<<D0_PIN);
            SET_DATA_MODE_OUT;
            // wait for address bus to change
            while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
            SET_DATA_MODE_IN;
          }
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
    case CART_TYPE_DPC:
         emulate_dpc_cartridge();
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
      if (pokey_enabled) emulate_supercart_large_pokey(); else emulate_supercart_large();
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

// Supercharger: install the mini-BIOS and build the multiload map. Split out of
// emulate_supercharger_cartridge() so it can run BEFORE reboot_cartridge(),
// which is the whole point - see the call site in setup1().
static void setup_supercharger(void) {
  uint8_t *ram = rom_table;
  uint8_t *rom = ram + 0x1800;
  uint8_t *multiload_map = rom + 0x0800;

  memset(ram, 0, 0x1800);
  setup_rom(rom);
  setup_multiload_map(multiload_map, (uint32_t)romLen / 8448);
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
  // Cleared here, not in the a78 branch: a 2600 file inspected after a 7800 one
  // must not inherit the previous cart's aux chip. (pokey_enabled has always had
  // that flaw; it is harmless in practice because a game never hands core 1 back,
  // so only the LAST identify before newgame can matter - but there is no reason
  // to add a second one.)
  ym_enabled = 0;
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
  const EXT_TO_CART_TYPE_MAP *p = ext_to_cart_type_map;
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
        if (image_size > AR_ROM_SIZE) {
          Serial.print("ERROR: Supercharger image (");Serial.print(image_size);
          Serial.print(" bytes) exceeds AR_ROM capacity (");Serial.print(AR_ROM_SIZE);
          Serial.println(" bytes), truncating to prevent memory corruption");
          image_size = AR_ROM_SIZE;
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
        
        // POKEY @$4000 is byte54 bit0 - and the mask on the next line CLEARS it,
        // so it has to be taken first. (bit6 = POKEY @$0450 is not handled yet.)
        // POKEY placement, from the 16-bit header field (byte53 high, byte54 low -
        // the order MAME, ProSystem and test7800 all agree on):
        //   bit0  -> $4000    bit6  -> $0450    bit10 -> $0440    bit15 -> $0800
        {
          uint16_t head_lo = A78_HEADER[54];

          // MAME's validate_header() (a78_slot.cpp:150-185) DISABLES POKEY@$4000
          // when the same header also puts RAM / bank 0 / bank 6 / banked RAM at
          // $4000 - the two cannot share the window, and the ROM/RAM is what the
          // game actually needs there. Without this we would hand such a cart a
          // POKEY instead of its data and break it outright. Real cases exist:
          // "Donkey Kong PK" (byte54=0x0B) and "Pit Fighter (Proto Alt 1)" (0x13)
          // in the Trebors library.
          uint8_t conflict = head_lo & 0x3d;
          if (conflict == 0x05 || conflict == 0x09 ||
              conflict == 0x11 || conflict == 0x21) {
            Serial.println("POKEY@4000 conflicts with $4000 RAM/ROM - disabling");
            head_lo &= (uint16_t)~0x01;
          }

          // Only ONE POKEY is emulated. Carts declaring two (the "Dual POKEY
          // 440 450" and "800 810" demos) get the first match and therefore half
          // their music - better than nothing, and they still run.
          pokey_mask = 0xFFF0;                      // 16-byte window by default
          if      (head_lo & 0x0001)     { pokey_enabled = 1; pokey_base = 0x4000; }
          else if (head_lo & 0x0040)     { pokey_enabled = 1; pokey_base = 0x0450; }
          else if (head_lo & 0x0400)     { pokey_enabled = 1; pokey_base = 0x0440; }
          else if (A78_HEADER[53] & 0x80){ pokey_enabled = 1; pokey_base = 0x0800;
                                           pokey_mask = 0xF800; }  // $0800-$0FFF
          else                           { pokey_enabled = 0; pokey_base = 0xFFFF; }
        }
        for (int i=0;i<16;i++) pokey_regs[i]=0;
        Serial.print("POKEY base:");Serial.println(pokey_base,HEX);

        // YM2151 (OPM) at $0460/$0461 - byte53 bit 3, "ym2151 at $460/$461" in
        // both MAME (xm.cpp) and ProSystem/JS7800 (Cartridge.js:443). 45 files in
        // this library declare it, among them 1942, Wonder Boy, Pac-Man Collection
        // 40th Anniversary and Block'Em Sock'Em.
        //
        // The chip belongs to Atari's XM expansion module, and 37 of the 45 write
        // the XM's enable register at $0470 before using it. We deliberately do NOT
        // decode $0470: a cartridge that carries its own YM2151 has no such
        // register, so the chip is simply always present. That serves both groups -
        // the $0470 write becomes a harmless store into open bus.
        //
        // Reusing pokey_base/pokey_mask is not a hack, it is the whole point: every
        // emulate_*_pokey() loop already tests one aux-chip window in the
        // $0000-$3FFF branch, and pokey_window_service() forwards to ym2151.h when
        // ym_enabled is set. Only one aux chip is emulated either way; the single
        // header here that declares both (Pit Fighter prototype, 0x2813) has its
        // POKEY disabled by the $4000 conflict rule above in any case.
        ym_enabled = (A78_HEADER[53] & 0x08) ? 1 : 0;
        if (ym_enabled) {
          pokey_enabled = 1;          // selects the listening emulate_* variant
          pokey_base    = 0x0460;
          pokey_mask    = 0xFFFE;     // exactly two bytes, as MAME's XM decodes
          Serial.println("YM2151 @ $0460");
#if YM_REPORT_RATE
          // Measure core 0's FM throughput HERE, where writing a file is safe -
          // no game is running yet and this function is already doing file I/O.
          // See ym2151.h for why it cannot be done from inside ym_run().
          ym_benchmark_and_log();
#endif
        }

        // MAME picks the bankswitch scheme from (byte53<<8 | byte54) & 0xe02e and
        // only afterwards overrides it, and only for byte53 EXACTLY 0x01
        // (Activision) or 0x02 (Absolute) - a78_slot.cpp:409-490. Bits 2 and 3 of
        // byte53, POKEY at $0440 and YM2151 at $0460, are outside that mask: they
        // say what ELSE is on the board, not how it banks.
        //
        // Gating this whole chain on "byte53 == 0" therefore dropped every cart
        // that declares one of those chips straight through to the flat-ROM path.
        // Measured over the 2345 headers in ROMS/7800, this change plus the
        // byte54==8 fix below moves 22 files and no others:
        //   16  YM2151 carts onto their real board - Wonder Boy (SuperGame+RAM),
        //       Pac-Man Collection 40th x2 (SuperGame), Block'Em Sock'Em
        //       (9-bank), and 12 that are 256KB-1MB and stay unplayable for the
        //       unrelated reason that rom_table holds 144KB
        //    4  flat 48KB carts off the 9-bank SuperCart path (byte54==8)
        //    2  dual-POKEY LZSS demos onto SuperGame+RAM, which is what MAME
        //       gives header 0x0446
        // The Activision/Absolute tests stay on the RAW byte, so headers like
        // 0x05 (Impossible Mission [f1]) keep landing where they land today.
        uint8_t map53 = A78_HEADER[53] & (uint8_t)~0x0C;
        if(A78_HEADER[53] == 1) {
          cart_type = CART_TYPE_ACTIVISION;
        } else if(A78_HEADER[53] == 2) {
          cart_type = CART_TYPE_ABSOLUTE;
        } else if(map53 == 0) {
          // Keep the raw low byte: the mask on the next line clears bit 0 and
          // bit 6, and bit 6 (POKEY @$0450) is what selects the _pokey variant
          // for a VersaBoard. Bits 7 and 5, which pick the mapper itself, do
          // survive the mask - reading them from raw54 is for clarity, not need.
          uint8_t raw54 = A78_HEADER[54];
          A78_HEADER[54]=A78_HEADER[54]&0b10111110;
          // Order is MAME's. In a78_slot.cpp the mRAM bit is an OVERRIDE applied
          // AFTER the bankswitch switch (:493), so it beats the SuperGame and
          // VersaBoard bits: "Turrican II Circular Scroll Test" (header 0x0082)
          // has the SuperGame bit set and is still an mRAM cart.
          if(raw54 & 0x80) {
            cart_type = CART_TYPE_MRAM;
          } else if((raw54 & 0x2e) == 0x22 || (raw54 & 0x2e) == 0x26) {
            // a78_slot.cpp:422 - switch (mapper & 0xe02e), cases 0x0022/0x0026.
            // byte53 is 0 in this branch, so the 0xe000 half of that mask is
            // always clear and testing the low byte against 0x2e is the same
            // test. MAME picks MegaCart instead when the payload exceeds 256KB
            // (:424); that variant is deliberately not implemented, since 512KB
            // cannot fit rom_table and no file in this library needs it.
            cart_type = CART_TYPE_VERSA;
          } else if(image_size > (131072+0x80)) {
            cart_type = CART_TYPE_SUPERCART_LARGE;
          } else if(A78_HEADER[54] == 2 || A78_HEADER[54] == 3) {
              cart_type = CART_TYPE_SUPERCART;
            } else if(A78_HEADER[54] == 18 || A78_HEADER[54] == 19) {
              cart_type = CART_TYPE_SUPERCART_EF;
          } else if(A78_HEADER[54] == 4 || A78_HEADER[54] == 5 || A78_HEADER[54] == 6 || A78_HEADER[54] == 7) {
            cart_type = CART_TYPE_SUPERCART_RAM;
          } else if(A78_HEADER[54] == 10 || A78_HEADER[54] == 11) {
              // MAME's switch has a case for 0x000a (A78_TYPEA - SuperGame with
              // bank 6 at $4000, Alien Brigade/Crossbow) but NONE for 0x0008, so
              // that value leaves m_type at its initial A78_TYPE0: a plain flat
              // cart. Byte54 bit 3 on its own means "ROM at $4000", which for a
              // non-bankswitched board is simply a 48K image - and every one of
              // the 21 files in ROMS/7800 that masks to 0x08 is exactly 49152
              // bytes. Serving them as a 9-bank SuperCart, which is what this
              // branch used to do, cannot have worked.
              cart_type = CART_TYPE_SUPERCART_ROM;
          } else {
            cart_type = CART_TYPE_NORMALA78;
          }
        } else {
          cart_type = CART_TYPE_NORMALA78;
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
  Serial.print(" - ");Serial.println(filelist[numfile*MAX_NAME_LEN]);

  char filetoadd[MAX_NAME_LEN];
  for(int x=0;x<MAX_NAME_LEN;x++) filetoadd[x]=filelist[numfile*MAX_NAME_LEN+x];
  filetoadd[MAX_NAME_LEN-1]=0; // guarantee null-terminator regardless of source name length

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
    Serial.print(filelist[j*MAX_NAME_LEN]);
  Serial.print(" ");Serial.println(j);
  }
}

////////////////////////////////////////////////////////////////////////////////////
//                     MENU ATARI
////////////////////////////////////////////////////////////////////////////////////

// case-insensitive compare of two null-terminated (or space-padded) filelist entries
int compareNamesCI(const char* a, const char* b) {
  for (int i=0;i<MAX_NAME_LEN-1;i++) {
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
        better = compareNamesCI(&filelist[j*MAX_NAME_LEN], &filelist[best*MAX_NAME_LEN])<0;
      }
      if (better) best=j;
    }
    if (best!=i) {
      char tmp[MAX_NAME_LEN];
      memcpy(tmp,&filelist[i*MAX_NAME_LEN],MAX_NAME_LEN);
      memcpy(&filelist[i*MAX_NAME_LEN],&filelist[best*MAX_NAME_LEN],MAX_NAME_LEN);
      memcpy(&filelist[best*MAX_NAME_LEN],tmp,MAX_NAME_LEN);
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
    char c = (off+i < MAX_NAME_LEN) ? filelist[idx*MAX_NAME_LEN+off+i] : 32;
    if ((c>96) && (c<123)) c=c-32; // the font has no lowercase letters
    if (c==0) c=32;
    if ((i==0) && direntry_isdir[idx]) c=c|0x80;
    if ((i==1) && direntry_toobig[idx]) c=c|0x80;
    menu_ram[idx*12+i]=c;
  }
}

void AtariMenu(int tipo) { // 1=start,2=next page, 3=prev page, 4=dir up
  int contfile=0;
  char filename[MAX_NAME_LEN];
 
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
           for(int x=0;x<MAX_NAME_LEN;x++) filelist[contfile*MAX_NAME_LEN+x]=filename[x];
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
            // MAX_NAME_LEN-1 usable chars + terminator: see the bugs/b01 note by the
            // filelist declaration for why this must stay >= the longest real ROM name.
            file.getName(filename, MAX_NAME_LEN);
            filename[sizeof(filename)-1]=0; // force null-terminator so later strcat can't run past this buffer
            for(int x=0;x<MAX_NAME_LEN;x++) filelist[contfile*MAX_NAME_LEN+x]=filename[x];
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
   // Once ANY cart is running, core 0 has nothing useful left to do - the menu is
   // gone and every emulate_* function on core 1 is an infinite loop that never
   // returns, whatever the cart type. The block below this point renders the menu
   // marquee (renderEntry(): reads filelist[], writes menu_ram[], both in SRAM) and
   // handles menu commands - real work only while the ATARI is running the menu
   // kernel, but still executed every pass here regardless, because menuletto never
   // resets to false once the menu has been browsed (which it always has, before any
   // game can be selected). For a POKEY cart this was already worked around by
   // handing core 0 to audio synthesis, which incidentally also stops it touching
   // that SRAM - but the fix only ever covered POKEY carts, not "any game is
   // running". This is the same class of core0/core1 SRAM contention the POKEY
   // carve-out exists to avoid.
   if (newgame) {
     if (ym_enabled)    ym_run();      // never returns
     if (pokey_enabled) pokey_run();   // never returns
     continue;                          // no cart type ever hands control back here
   }

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
       int len = strlen(&filelist[marquee_row*MAX_NAME_LEN]);
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

