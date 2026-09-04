// Order matters: including these first forces the "SdFat - Adafruit Fork" package
// (required by Adafruit_SPIFlash) to be picked over the RP2040 core's bundled plain SdFat.
#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <RingBuf.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>

#include <Adafruit_TinyUSB.h>

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
#define D0_PIN   15
#define D1_PIN   16
#define D2_PIN   17
#define D3_PIN   18
#define D4_PIN   19
#define D5_PIN   20
#define D6_PIN   21
#define D7_PIN   22
#define LED_PIN  25 //gpio 25
#define A15_PIN  26 // to manage
#define RW_PIN   27
#define CLK_PIN  28

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
#define D0_PIN_MASK     0x00008000L //gpio 15
#define D1_PIN_MASK     0x00010000L
#define D2_PIN_MASK     0x00020000L
#define D3_PIN_MASK     0x00040000L
#define D4_PIN_MASK     0x00080000L  // gpio 19
#define D5_PIN_MASK     0x00100000L
#define D6_PIN_MASK     0x00200000L
#define D7_PIN_MASK     0x00400000L //gpio 22
#define filler          0x00800000L //gpio 23
#define filler1         0x01000000L //gpio 24
#define LED_PIN_MASK    0x02000000L //gpio 25
#define A15_PIN_MASK    0x04000000L //gpio 26
#define RW_PIN_MASK     0x08000000L //gpio 27
#define CLK_PIN_MASK    0x10000000L //gpio 28


// Aggregate Pico pin usage masks
#define ALL_GPIO_MASK  	0x1FFFFFFFL
#define BUS_PIN_MASK    0x00007FFFL
#define BUS15_PIN_MASK  0x04007FFFL 
#define BUS_H_PIN_MASK  (A13_PIN_MASK | A14_PIN_MASK | A15_PIN_MASK ) // to check
#define DATA_PIN_MASK   0x007F8000L
#define STATUS_PIN_MASK 0x18000000L
#define READ_PIN_MASK    (A14_PIN_MASK | A15_PIN_MASK | RW_PIN_MASK)  //gpio 27

#define ALWAYS_IN_MASK  (BUS_PIN_MASK | STATUS_PIN_MASK)
#define ALWAYS_OUT_MASK (DATA_PIN_MASK)

#define SET_DATA_MODE_OUT   gpio_set_dir_out_masked(DATA_PIN_MASK)
#define SET_DATA_MODE_IN    gpio_set_dir_in_masked(DATA_PIN_MASK)

// --- Bankset (7800): HALT ---------------------------------------------------
// BANKSET_HAS_HALT says whether the console's HALT line physically reaches this
// board. It is the ONE thing the two sketches cannot share, and it is a board
// fact, not a preference:
//
//   PicoA10400  - the purple RP2040 clone exposes GPIO23/24/25, and the shipped
//                 gerbers route the cartridge connector's Halt pin (U2 pad 2) to
//                 header pin 14, which on that clone is GPIO24. Verified in
//                 GERBER-PicoA10400.zip: the Halt net has a pad AND traces, and
//                 pin 14 sits between pin 13 (unrouted = GP25/LED) and pin 15
//                 (/D7 = GPIO23), which pins the numbering down.
//   Pico2A10400 - a genuine Pico 2 does not break GPIO23/24/25 out at all, and
//                 GERBER-Pico2A10400.zip leaves the Halt net as a single
//                 unconnected pad on the connector. There is no free header GPIO
//                 left on that board either (0-14 address, 15-22 data, 26-28
//                 A15/RW/CLK), so this is not a firmware choice to make.
//
// With BANKSET_HAS_HALT 0 the Bankset loops still run and still map the cart
// correctly - they just always serve Sally's half, so a Bankset game boots and
// plays with the wrong graphics instead of not booting at all. That is strictly
// better than the flat mapping these files get today, and it is all the Pico 2
// hardware allows.
// Overridable from the compile line so a single build can ask "is the HALT
// line usable at all?": build.sh target PicoA10400-BSSA passes
// -DBANKSET_HAS_HALT=0 and installs <Sketch>.ino_BSSA.uf2, which maps every
// Bankset board correctly but always serves Sally's half.
#ifndef BANKSET_HAS_HALT
#define BANKSET_HAS_HALT 0
#endif

// /HALT is asserted LOW - Maria pulls it down to take the bus, the way RDY
// stops a 6502 - so HALT low selects Maria's half. NOT yet confirmed on our own
// hardware, so it is overridable from the compile line: build.sh targets
// PicoA10400-BSHI / Pico2A10400-BSHI pass -DBANKSET_HALT_ACTIVE_LOW=0 and
// install the result as <Sketch>.ino_BSHI.uf2, next to the production build.
// A wrong guess here is not subtle - the reset vector itself comes out of the
// wrong half - so one hardware test settles it.
#ifndef BANKSET_HALT_ACTIVE_LOW
#define BANKSET_HALT_ACTIVE_LOW 1
#endif

#include "ym2151.h"  // YM2151 (OPM) FM synthesis, same audio pin. Included
                     // BEFORE pokey.h, because pokey_window_service() hands
                     // the $04xx window over to it for a YM cart.
#include "pokey.h"   // minimal POKEY audio - see the header for why this board
                     // has no audio line routed to the Pico at all.


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
// UA Ltd. 8K bankswitching. Numbered ABOVE the 7800 block only because that
// block was numbered first - UA and UASW are 2600 types and must be dispatched
// as such. Every numeric test on the cart type (the clock/voltage branches in
// setup1()) therefore names them explicitly; see is2600 there.
#define CART_TYPE_UA	43	// 8k, hotspots BELOW $1000 ($0220 / $0240)
#define CART_TYPE_UASW	44	// 8k UA with the two banks swapped (Stella "UASW")
// Bankset (7800): one file, two complete images - the first half for Sally,
// the second for Maria, selected by the console's HALT line. Four boards, the
// same four MAME's a78_slot.cpp switch(mapper & 0xe02e) picks between; the
// POKEY/YM variants are not separate types here because pokey_base already
// carries that.
#define CART_TYPE_BANKSET	45	// flat 2x32K / 2x48K / 2x52K
#define CART_TYPE_BANKSET_RAM	46	// flat + 2x16K banked RAM at $4000
#define CART_TYPE_BANKSET_SG	47	// SuperGame, 2x(64K..256K)
#define CART_TYPE_BANKSET_SG_RAM	48	// SuperGame + 2x16K banked RAM at $4000
// FC - Amiga's "Power Play Arcade Video Game Album" board. A 2600 type sitting
// above the 7800 block for the same reason UA does, so it has to be named in
// the is2600 test in setup1() as well. Unlike every other 2600 mapper here the
// bank number arrives on the DATA bus, not in the address.
#define CART_TYPE_FC	49	// 8K/16K/32K, bank latched from the data bus

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
	{"UA", CART_TYPE_UA},
	{"UAS", CART_TYPE_UASW},
  {"FA2", CART_TYPE_FA2},
  {"FC", CART_TYPE_FC},
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
// SdFat's failure path in the first place. Verified via the L<len>C<code> footer
// diagnostic (see bugs/b01/worklog.md) on real hardware.
//
// 80 was picked when ROMS/ held nothing longer than 61 characters. That bound came
// back: measured over the whole 71288-file library, 8960 names (12.6%) are 80 bytes
// or longer, and every one of them was listed corrupted. 128 brings that down to 936
// files (1.3%, 266 distinct titles) for 4080 bytes of SRAM - the knee of the curve:
// up to 128 a repaired file costs well under a byte, past it the price per file goes
// 3.6 and then 21 bytes (204 would cover the whole library - the longest name in it
// is 203 - but costs 10616 bytes). Names that still do not fit are no longer a silent
// corruption: getName()'s return value is checked, and such an entry is drawn as
// MENU_LONGNAME_TEXT in OVERSIZED_COLOUR. Raise this if those 936 ever matter.
#define MAX_NAME_LEN 128
char filelist[85*MAX_NAME_LEN]; // 85 entries, MAX_NAME_LEN bytes each (incl. terminator)
char direntry_isdir[85]; // 1 if filelist[n] is a directory, 0 if a regular file (".." counts as 0: no highlight, not sorted)
char direntry_toobig[85]; // 1 if filelist[n] cannot be used as listed: file larger than rom_table (loaded truncated), or a name too long to read; shown red in the menu
#define MENU_FOOTER_TEXT "AOTTAv01 HR8" // 12 chars: the menu kernel renders exactly 12 per row
// Stand-in name for a directory entry whose real name does not fit in MAX_NAME_LEN.
// 12 chars, uppercase only: the row shows exactly 12 and the font has no lowercase.
#define MENU_LONGNAME_TEXT "NAME TOO LNG"
// Colour of oversized-ROM names. The kernel reads this at runtime from menu_status[12],
// so changing it needs no ROM patch - just this line. $66 was picked by sweeping all 16
// hues on the actual PAL TV: hue 6 is the red family here, and luminance 6 keeps it
// saturated (luminance A washed out to near-white). More saturated: $64. Brighter: $68.
#define OVERSIZED_COLOUR 0x66

// Core-1 clock used while a 7800 cart type is being emulated (cart_to_emulate>=33 on
// this board). Everything else - menu, USB, 2600 types - keeps the 250MHz set in
// setup(). Kept in sync with PicoA10400.ino, where the full reasoning lives:
// the measured response path is ~18 instructions against MARIA's ~279ns tightest
// cartridge access, so 250MHz still leaves ~3.9x margin while 400000 was ~2.7x past
// the RP2350's 150MHz spec. Single-variable change - the 1.25V below is unchanged.
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
// Deepest directory in ROMS/ needs 120 bytes including the trailing '/', so 128 still
// fits with margin; filetoopen is sized FROM these two so it can never be the shorter
// one again (path <= MAX_PATH_LEN-1 chars + name <= MAX_NAME_LEN-1 chars + terminator).
#define MAX_PATH_LEN 128
char path[MAX_PATH_LEN];
 char filetoopen[MAX_PATH_LEN+MAX_NAME_LEN]; // was 50, which overflowed with long names or subdirectories; a too-short buffer here truncates the path and the file silently fails to open
 
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
        if (rawaddr & A15_PIN_MASK) {
            addr = addr | 0x8000;
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
      
      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (rawaddr & A15_PIN_MASK) {
            addr=addr | 0x8000;
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
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    // Check for bankswitch
                    if (rawaddr == A15_PIN_MASK) {
                        // Bankswitching write
                        SET_DATA_MODE_IN;
                        // Check for 0x01
                        // Krok 20: end-of-cycle capture - see emulate_supercart_ram()
                        // for the reasoning. Bounded at 64 turns; A15 is gpio 26 here so
                        // the comparison uses BUS15_PIN_MASK.
                        {
                          uint32_t last = gpio_get_all(), cur;
                          uint32_t full = last & BUS15_PIN_MASK;
                          for (uint32_t g = 0; g < 64; g++) {
                              cur = gpio_get_all();
                              if ((cur & BUS15_PIN_MASK) != full) break;
                              last = cur;
                          }
                          rawaddr = last;
                        }
                        switch ((rawaddr >> D0_PIN) & 0xff) {
                        case 0:
                            bank = 0;
                            break;
                        case 1:
                            bank = 0x4000;
                            break;
                        case 2:
                            bank = 0x8000;
                            break;
                        case 3:
                            bank = 0xc000;
                            break;
                        case 4:
                            bank = 0x10000;
                            break;
                        case 5:
                            bank = 0x14000;
                            break;
                        case 6:
                            bank = 0x18000;
                            break;
                        case 7:
                            bank = 0x1c000;
                            break;
                        default:
                            break;
                        }
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            // EXFIX - bank 6 is in 0x4000
            if (addr & 0x4000) {
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + 0x18000] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
                rawaddr = gpio_get_all() & (READ_PIN_MASK );
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
      // Bank-number mask derived from the real ROM size, mirroring MAME
      // (a78_slot.cpp:74-77 + rom.cpp:388: data & m_bank_mask, where
      // m_bank_mask = (nbanks odd) ? nbanks-2 : nbanks-1) and ProSystem/JS7800
      // (Cartridge.js:729, which instead drops any write with bank >= size/16384).
      // For the 128KB/8-bank carts this type actually sees, this is exactly the
      // "& 0x07" that used to be hardcoded here; deriving it keeps PicoA10400 and
      // Pico2A10400 identical and stays correct for 4/9/16-bank carts too.
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));

      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (rawaddr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                sio_hw->gpio_out = (uint32_t)rom_table[addr + fixed_base] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    // Check for bankswitch
                    if (rawaddr == A15_PIN_MASK) {
                        // Bankswitching write
                        SET_DATA_MODE_IN;
                        // Krok 14b BUG: rawaddr still held the RW/A15-masked value from
                        // the test above, so inside this branch it is exactly A15_PIN_MASK
                        // and every data bit in it is 0 - ((0x04000000 >> 15) & 0x07) is a
                        // constant 0. A bankswitch write always selected bank 0, so no
                        // SUPERCART_RAM cart could ever leave its first 16KB bank on this
                        // board. Krok 19: and the byte must come from the END of the write
                        // cycle - a 6502 does not drive the data lines until the second
                        // half of the cycle. Bounded (see PicoA10400.ino for the cycle
                        // arithmetic); an unbounded wait here hung the cart in Krok 18.
                        // A15 is gpio 26 on this board and sits OUTSIDE BUS_PIN_MASK, so
                        // end-of-cycle detection must compare BUS15_PIN_MASK.
                        uint32_t last = gpio_get_all(), cur;
                        uint32_t full = last & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS15_PIN_MASK) != full) break;
                            last = cur;
                        }
                        bank = ((last >> D0_PIN) & sc_bank_mask) * 0x4000;
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
                        // Krok 19: end-of-cycle capture, same reasoning as the bank
                        // register above. 'addr' is already a 14-bit offset here, so keep
                        // a full-width address for the comparison.
                        uint32_t wlast = gpio_get_all(), wcur;
                        uint32_t wfull = wlast & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS15_PIN_MASK) != wfull) break;
                            wlast = wcur;
                        }
                        ram_table[wlast & 0x3fff] = (wlast >> D0_PIN) & 0xff;
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

// Extracted from the inline switch case it used to be (see patches/Pico2A10400_0.09.txt).
// Pure extraction + speed: behaviour is unchanged from before, only where the code lives
// and how fast it answers the bus changed. Compiled at -O2 rather than the sketch
// default -Os: at -Os the SDK's static-inline gpio_put_masked() gets outlined into a
// copy in FLASH, called through a RAM veneer on every single bus response (~25 cycles
// plus XIP-cache-miss jitter - see PicoA10400_tune/README.md for the measured evidence).
// Each data output below is a single SIO store instead, safe here because in 7800
// modes the only output pins are D0-D7, so the other OUT-latch bits are don't-cares.
// Note the addr=addr|0x8000 step below: on this board A15 is wired to GPIO26, outside
// BUS_PIN_MASK's contiguous bit field, so it has to be folded back into a normal 16-bit
// address by hand wherever the code needs one (unlike PicoA10400, where A15 already
// sits at bit 15 of BUS_PIN_MASK) - this line is NOT redundant, do not remove it.
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
        if (rawaddr & A15_PIN_MASK) {
          addr=addr | 0x8000;
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
                        // Krok 20: end-of-cycle capture, proven on CART_TYPE_SUPERCART_RAM
                        // in Krok 19 - the 6502 drives the data lines only in the second
                        // half of a write cycle, and this polling loop catches the write at
                        // a random phase. Bounded at 64 turns; an unbounded wait hung the
                        // cart in Krok 18. A15 is gpio 26 here, outside BUS_PIN_MASK, so
                        // compare BUS15_PIN_MASK.
                        {
                          uint32_t last = gpio_get_all(), cur;
                          uint32_t full = last & BUS15_PIN_MASK;
                          for (uint32_t g = 0; g < 64; g++) {
                              cur = gpio_get_all();
                              if ((cur & BUS15_PIN_MASK) != full) break;
                              last = cur;
                          }
                          rawaddr = last;
                        }
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
// response window), but never got the "P2" treatment already given to
// emulate_activision()/emulate_supercart_*() on this board: core-1 IRQs left
// on, and (for the 32k path here) a SET_DATA_MODE_OUT / wait / SET_DATA_MODE_IN
// dance on every single access instead of the rom_in_use-cached direction
// those functions use. They ran inline inside setup1() instead - which is
// itself built at -O3 (see the #pragma block below), so this is purely the
// same gap noted on those other functions: no cpsid i, and gpio_put_masked()
// (a call+return per bus response, confirmed call-only via check_hotpath.sh -
// no flash veneer) instead of a single inlined SIO store.
// User-visible motivation: "7800 XMAS" (SUPERCART_RAM, already P2-hardened)
// showed visibly fewer in-game artifacts than CART_TYPE_NORMALA78 titles
// (2600 Maze Pac-Man, 3D Worldrunner) on the same hardware. Pure extraction
// otherwise: logic - including the bugs/b01 16k A14+A15 fix, the 32k path's
// per-access direction toggling, and the 48k/ABSOLUTE paths' rom_in_use
// caching and RW_PIN_MASK-based bankswitch-write detection - is unchanged,
// only where it lives and how it drives the bus changed. Diagnostic - not
// yet confirmed on hardware to actually reduce artifacts; see
// not_working_roms4/README.md.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_normala78()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  uint32_t addr=0, addr_prev=0, rawaddr=0;
  // not_working_roms4 "Krok 21" - one general mapping instead of three hardcoded
  // size cases, taken from MAME's a78_rom_device::read_40xx (rom.cpp:257-263):
  // a flat cart sits at the TOP of the address space (m_base_rom = 0x10000 - size)
  // and stays silent below its own start. The old code only handled 16k/32k/48k;
  // any other size fell through and drove nothing at all - a blank screen. In this
  // library that silently broke "7ix" (28KB) and "Bouncing Balls (Demo)" (8KB).
  // See PicoA10400.ino for the equivalence proof against the three old cases.
  //
  // Board difference (NOT a copy of PicoA10400.ino): here BUS_PIN_MASK covers only
  // A0-A14 and A15 is gpio 26, so the full 16-bit address has to be reassembled
  // before it can be compared against a $0000-$FFFF boundary, and the wait for the
  // cycle to end compares BUS15_PIN_MASK.
  const uint32_t base_rom = (romLen >= 0x10000) ? 0x4000 : (0x10000 - (uint32_t)romLen);
  const uint32_t lo = (base_rom < 0x4000) ? 0x4000 : base_rom;

  while (1) {
    // wait for a stable full address (A0-A14 plus A15 on gpio 26)
    while ((rawaddr = (gpio_get_all()&BUS15_PIN_MASK)) != addr_prev)
      addr_prev = rawaddr;
    addr = (rawaddr & BUS_PIN_MASK) | ((rawaddr & A15_PIN_MASK) ? 0x8000 : 0);
    if (addr >= lo) {
      sio_hw->gpio_out = (uint32_t)rom_table[addr - base_rom] << D0_PIN;  // D0-D7 are the only outputs in 7800 modes
      SET_DATA_MODE_OUT;
      // wait for address bus to change
      while ((gpio_get_all()&BUS15_PIN_MASK) == rawaddr) ;
      SET_DATA_MODE_IN;
    }
  }
}

// POKEY-cart variant of emulate_normala78(). Separate function so flat carts
// without POKEY keep byte-identical code to the hardware-proven version.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_normala78_pokey()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  uint32_t addr=0, addr_prev=0, rawaddr=0;
  // not_working_roms4 "Krok 21" - one general mapping instead of three hardcoded
  // size cases, taken from MAME's a78_rom_device::read_40xx (rom.cpp:257-263):
  // a flat cart sits at the TOP of the address space (m_base_rom = 0x10000 - size)
  // and stays silent below its own start. The old code only handled 16k/32k/48k;
  // any other size fell through and drove nothing at all - a blank screen. In this
  // library that silently broke "7ix" (28KB) and "Bouncing Balls (Demo)" (8KB).
  // See PicoA10400.ino for the equivalence proof against the three old cases.
  //
  // Board difference (NOT a copy of PicoA10400.ino): here BUS_PIN_MASK covers only
  // A0-A14 and A15 is gpio 26, so the full 16-bit address has to be reassembled
  // before it can be compared against a $0000-$FFFF boundary, and the wait for the
  // cycle to end compares BUS15_PIN_MASK.
  const uint32_t base_rom = (romLen >= 0x10000) ? 0x4000 : (0x10000 - (uint32_t)romLen);
  const uint32_t lo = (base_rom < 0x4000) ? 0x4000 : base_rom;
  const uint32_t pkbase = (uint32_t)pokey_base;   // volatile: hoist out of the loop
  const uint32_t pkmask = (uint32_t)pokey_mask;
  const uint32_t ymon   = (uint32_t)ym_enabled;  // ditto - never read in the loop

#if POKEY_DIAG_E6
  // E6 DIAGNOSTIC BUILD (pokey.h) - carried here to keep both sketches in step;
  // the experiment is run on PicoA10400. Same idea: non-blocking, and OE held
  // across consecutive reads via rom_in_use instead of released after every
  // fetch, in the shape emulate_supercart_ram() already proves on hardware.
  // See PicoA10400.ino for the full reasoning and the 0.18 regression it
  // avoids. The one board difference stays: A15 is gpio 26, outside
  // BUS_PIN_MASK, so the full address is reassembled before the range test.
  uint8_t rom_in_use = 0;
  while (1) {
    uint32_t raw = gpio_get_all();
    rawaddr = raw & BUS15_PIN_MASK;
    addr = (rawaddr & BUS_PIN_MASK) | ((rawaddr & A15_PIN_MASK) ? 0x8000 : 0);
    if (addr >= lo) {
      sio_hw->gpio_out = (uint32_t)rom_table[addr - base_rom] << D0_PIN;
      if (raw & RW_PIN_MASK) {
        if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
      } else if (rom_in_use) {
        SET_DATA_MODE_IN; rom_in_use = 0;
      }
    } else {
      if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
      if ((addr & pkmask) == pkbase && ymon) {
        ym_window_service_blocking(addr);
      } else if ((addr & pkmask) == pkbase) {
        if (!(gpio_get_all() & RW_PIN_MASK)) {
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS15_PIN_MASK) != rawaddr) break;
            last = cur;
          }
          uint32_t pkreg = addr & 0x0F;
          pokey_capture_write(pkreg, (uint8_t)((last >> D0_PIN) & 0xFF));
        }
      }
    }
  }
  (void)addr_prev;
#else
  while (1) {
#if POKEY_DIAG_E7
    // E7 DIAGNOSTIC BUILD (pokey.h) - see PicoA10400.ino. THREE matching samples
    // instead of two, and nothing else changed.
    do {
      while ((rawaddr = (gpio_get_all()&BUS15_PIN_MASK)) != addr_prev)
        addr_prev = rawaddr;
    } while ((gpio_get_all()&BUS15_PIN_MASK) != rawaddr);
#else
    // wait for a stable full address (A0-A14 plus A15 on gpio 26)
    while ((rawaddr = (gpio_get_all()&BUS15_PIN_MASK)) != addr_prev)
      addr_prev = rawaddr;
#endif
    addr = (rawaddr & BUS_PIN_MASK) | ((rawaddr & A15_PIN_MASK) ? 0x8000 : 0);
    if (addr >= lo) {
      sio_hw->gpio_out = (uint32_t)rom_table[addr - base_rom] << D0_PIN;  // D0-D7 are the only outputs in 7800 modes
      SET_DATA_MODE_OUT;
      // wait for address bus to change
      while ((gpio_get_all()&BUS15_PIN_MASK) == rawaddr) ;
      SET_DATA_MODE_IN;
    } else if ((addr & pkmask) == pkbase && ymon) {
      // YM2151 window ($0460/$0461). This one CANNOT be listen-only: 32 of the 45
      // YM carts sit in "BIT $0461 / BMI *-3" waiting for the BUSY bit to clear
      // and write nothing until they see it. ym_window_service_blocking() answers
      // in the same drive/wait/release shape this loop uses for ROM, so it does
      // not change how the loop behaves - see ym2151.h.
      ym_window_service_blocking(addr);
    } else if ((addr & pkmask) == pkbase) {      // POKEY register window
      // LISTEN ONLY - deliberately no read support on this path. Reads were tried
      // on the other board and REGRESSED "3D Worldrunner Theme Melody (4000)":
      // driving the bus gated on R/W is the shape that hung the cart in Krok 18,
      // because one mis-read of R/W turns a POKEY write into "drive against the CPU
      // and then block". This loop blocks in its ROM path, so it cannot afford that;
      // the SuperGame variants never block and there reads are safe.
      if (!(gpio_get_all() & RW_PIN_MASK)) {
        // End-of-cycle capture, the shape proven in version 0.15: a 6502 does not
        // drive the data lines until the second half of the cycle. Bounded at 64.
        uint32_t last = gpio_get_all(), cur;
        for (uint32_t g = 0; g < 64; g++) {
          cur = gpio_get_all();
          if ((cur & BUS15_PIN_MASK) != rawaddr) break;
          last = cur;
        }
        uint32_t pkreg = addr & 0x0F;
        pokey_capture_write(pkreg, (uint8_t)((last >> D0_PIN) & 0xFF));   // see pokey.h
      }
    }
  }
#endif  // POKEY_DIAG_E6
}

// mRAM ("masked RAM") - MAME's A78_TYPE8, test7800's external/mram.go. A FLAT
// cart plus 16KB of on-cart RAM at $4000-$7FFF. MAME's rom.cpp describes the
// board as "no bankswitch + mRAM chip"; it is selected by bit 7 of the a78
// header's low byte, which OVERRIDES whatever bankswitch bits are also set
// (a78_slot.cpp:493) - "Turrican II Circular Scroll Test" (header 0x0082) has
// the SuperGame bit on and is still an mRAM cart.
//
// Two halves, each already proven on hardware, joined: the ROM window is
// emulate_normala78()'s general mapping (version 0.16, from MAME rom.cpp:257-263
// - a flat cart sits at the TOP of the address space, base_rom = 0x10000 - size,
// and stays silent below its own start), and the RAM window is
// emulate_supercart_ram()'s $4000 path including the Krok 19 end-of-cycle write
// capture.
//
// The one new element is the address mask. test7800 (mram.go):
//
//     if address < 0x8000 { address &= 0xfeff; ram[address-0x4000] }
//
// A8 is not connected, so $4100-$41FF is the same storage as $4000-$40FF - hence
// the name. For the $4000-$7FFF range this branch handles, (address & 0xfeff) -
// 0x4000 is exactly (address & 0x3eff), so the mask is folded into the index.
//
// Board difference (NOT a copy of PicoA10400.ino): here BUS_PIN_MASK covers only
// A0-A14 and A15 is gpio 26, so the full 16-bit address has to be reassembled
// before it can be compared against the ROM start, and every end-of-cycle
// comparison uses BUS15_PIN_MASK.
//
// Unlike emulate_normala78() this loop does NOT block while the address is
// stable: it must keep servicing the RAM window, and "blocking ROM path + R/W
// gating" is what regressed 3D Worldrunner in 0.18. Shaped like
// emulate_supercart_ram(), which never blocks.
//
// No _pokey variant: every mRAM file in this library has header 0x0080 or 0x0082,
// so none declares a POKEY - and this board has no audio line anyway (pokey.h).
__attribute__((optimize("O2")))
void __time_critical_func(emulate_mram()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  const uint32_t base_rom = (romLen >= 0x10000) ? 0x4000 : (0x10000 - (uint32_t)romLen);
  const uint32_t lo = (base_rom < 0x4000) ? 0x4000 : base_rom;
  uint32_t addr = 0, rawaddr = 0;
  uint8_t rom_in_use = 1;

  while (1) {
    rawaddr = gpio_get_all();
    // A15 is gpio 26, outside BUS_PIN_MASK - fold it back into a normal address.
    addr = (rawaddr & BUS_PIN_MASK) | ((rawaddr & A15_PIN_MASK) ? 0x8000 : 0);
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
            uint32_t wfull = wlast & BUS15_PIN_MASK;
            for (uint32_t g = 0; g < 64; g++) {
                wcur = gpio_get_all();
                if ((wcur & BUS15_PIN_MASK) != wfull) break;
                wlast = wcur;
            }
            ram_table[wlast & 0x3EFF] = (wlast >> D0_PIN) & 0xff;
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
  uint32_t addr=0, rawaddr=0;
  uint32_t bank=0x4000;
  uint8_t rom_in_use=1;
  while (1) {       // Get address
    rawaddr = gpio_get_all();
    addr=rawaddr & BUS_PIN_MASK;
 // Check for A15
    if (rawaddr & A15_PIN_MASK) {
        addr |= 0x8000;
        // Set the data on the bus
        sio_hw->gpio_out = (uint32_t)rom_table[addr] << D0_PIN;
        // Check for RW
      if (rawaddr & RW_PIN_MASK) {
            // Read cycle
            if (!rom_in_use) {
                SET_DATA_MODE_OUT;
                rom_in_use = 1;
            }
        } else {
            // Write cycle to ROM
            rawaddr = gpio_get_all() & (BUS15_PIN_MASK | RW_PIN_MASK);
            // Check for bankswitch
            if (rawaddr == A15_PIN_MASK) {
                // Bankswitching write
                SET_DATA_MODE_IN;
                // Check for 0x01
                      // Krok 20: end-of-cycle capture - see emulate_supercart_ram()
                      // for the reasoning. Bounded at 64 turns; A15 is gpio 26 here so
                      // the comparison uses BUS15_PIN_MASK.
                      {
                        uint32_t last = gpio_get_all(), cur;
                        uint32_t full = last & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS15_PIN_MASK) != full) break;
                            last = cur;
                        }
                        rawaddr = last;
                      }
                if ((rawaddr & DATA_PIN_MASK) == 0x008000) {
                    // Switch to flying mode
                    bank = 0;
                } else {
                    if ((rawaddr & DATA_PIN_MASK) == 0x010000) {
                        // Switch to title page
                        bank = 0x4000;
                    }
                }
                rom_in_use = 0;
            }
        }
    } else {
        // Check for A14
        if (addr & A14_PIN_MASK) {
            // Set the data on the bus
            sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + bank] << D0_PIN;
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
      }
    }
}




// POKEY-cart variant of emulate_supercart_ef(). A SEPARATE function so carts without POKEY
// keep byte-identical code to the version proven on hardware.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_ef_pokey()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      // Hoisted: pokey_base/pokey_mask are volatile, and reading them from
      // memory on every pass through the cold branch is what made a cart with
      // no POKEY at all (Alien Brigade) start glitching on the other board.
      const uint32_t pkbase = (uint32_t)pokey_base;
      const uint32_t pkmask = (uint32_t)pokey_mask;
      uint32_t bank=0, addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      
      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (rawaddr & A15_PIN_MASK) {
            addr=addr | 0x8000;
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
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    // Check for bankswitch
                    if (rawaddr == A15_PIN_MASK) {
                        // Bankswitching write
                        SET_DATA_MODE_IN;
                        // Check for 0x01
                        // Krok 20: end-of-cycle capture - see emulate_supercart_ram()
                        // for the reasoning. Bounded at 64 turns; A15 is gpio 26 here so
                        // the comparison uses BUS15_PIN_MASK.
                        {
                          uint32_t last = gpio_get_all(), cur;
                          uint32_t full = last & BUS15_PIN_MASK;
                          for (uint32_t g = 0; g < 64; g++) {
                              cur = gpio_get_all();
                              if ((cur & BUS15_PIN_MASK) != full) break;
                              last = cur;
                          }
                          rawaddr = last;
                        }
                        switch ((rawaddr >> D0_PIN) & 0xff) {
                        case 0:
                            bank = 0;
                            break;
                        case 1:
                            bank = 0x4000;
                            break;
                        case 2:
                            bank = 0x8000;
                            break;
                        case 3:
                            bank = 0xc000;
                            break;
                        case 4:
                            bank = 0x10000;
                            break;
                        case 5:
                            bank = 0x14000;
                            break;
                        case 6:
                            bank = 0x18000;
                            break;
                        case 7:
                            bank = 0x1c000;
                            break;
                        default:
                            break;
                        }
                        rom_in_use = 0;
                    }
                }
            }
        } else {
            // EXFIX - bank 6 is in 0x4000
            if (addr & 0x4000) {
                sio_hw->gpio_out = (uint32_t)rom_table[(addr & 0x3fff) + 0x18000] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
                rawaddr = gpio_get_all() & (READ_PIN_MASK );
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
                // $0000-$3FFF. The $0450/$0440/$0800 POKEY windows live here, below
                // every SuperGame window, so they cannot collide with ROM/RAM.
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

// POKEY-cart variant of emulate_supercart_ram(). A SEPARATE function so carts without POKEY
// keep byte-identical code to the version proven on hardware.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_ram_pokey()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      // Hoisted: pokey_base/pokey_mask are volatile, and reading them from
      // memory on every pass through the cold branch is what made a cart with
      // no POKEY at all (Alien Brigade) start glitching on the other board.
      const uint32_t pkbase = (uint32_t)pokey_base;
      const uint32_t pkmask = (uint32_t)pokey_mask;
      uint32_t bank=0;
      // v0.13 (P1): romLen is volatile, so the fixed-bank index below was
      // re-read from memory on every pass. Constant for the whole game - hoist.
      const uint32_t fixed_base = (uint32_t)romLen - 0x8000;
      uint32_t addr=0, addr_prev=0, rawaddr=0;
      uint8_t rom_in_use=1;
      // Bank-number mask derived from the real ROM size, mirroring MAME
      // (a78_slot.cpp:74-77 + rom.cpp:388: data & m_bank_mask, where
      // m_bank_mask = (nbanks odd) ? nbanks-2 : nbanks-1) and ProSystem/JS7800
      // (Cartridge.js:729, which instead drops any write with bank >= size/16384).
      // For the 128KB/8-bank carts this type actually sees, this is exactly the
      // "& 0x07" that used to be hardcoded here; deriving it keeps PicoA10400 and
      // Pico2A10400 identical and stays correct for 4/9/16-bank carts too.
      const uint32_t sc_nbanks = (uint32_t)romLen / 0x4000;
      const uint32_t sc_bank_mask = (sc_nbanks < 2) ? 0
                                  : ((sc_nbanks & 1) ? (sc_nbanks - 2) : (sc_nbanks - 1));

      while (1) {    // Get address
             // Get address
        rawaddr = gpio_get_all();
        addr = rawaddr & BUS_PIN_MASK;
        // Check for A15
        if (rawaddr & A15_PIN_MASK) {
            // Check for A14
            if (addr & A14_PIN_MASK) {
                // Set the data on the bus for fixed bank 7
                sio_hw->gpio_out = (uint32_t)rom_table[addr + fixed_base] << D0_PIN;  // v0.13: D0-D7 are the only outputs in 7800 modes
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
                    rawaddr = gpio_get_all() & (RW_PIN_MASK | A15_PIN_MASK);
                    // Check for bankswitch
                    if (rawaddr == A15_PIN_MASK) {
                        // Bankswitching write
                        SET_DATA_MODE_IN;
                        // Krok 14b BUG: rawaddr still held the RW/A15-masked value from
                        // the test above, so inside this branch it is exactly A15_PIN_MASK
                        // and every data bit in it is 0 - ((0x04000000 >> 15) & 0x07) is a
                        // constant 0. A bankswitch write always selected bank 0, so no
                        // SUPERCART_RAM cart could ever leave its first 16KB bank on this
                        // board. Krok 19: and the byte must come from the END of the write
                        // cycle - a 6502 does not drive the data lines until the second
                        // half of the cycle. Bounded (see PicoA10400.ino for the cycle
                        // arithmetic); an unbounded wait here hung the cart in Krok 18.
                        // A15 is gpio 26 on this board and sits OUTSIDE BUS_PIN_MASK, so
                        // end-of-cycle detection must compare BUS15_PIN_MASK.
                        uint32_t last = gpio_get_all(), cur;
                        uint32_t full = last & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS15_PIN_MASK) != full) break;
                            last = cur;
                        }
                        bank = ((last >> D0_PIN) & sc_bank_mask) * 0x4000;
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
                        // Krok 19: end-of-cycle capture, same reasoning as the bank
                        // register above. 'addr' is already a 14-bit offset here, so keep
                        // a full-width address for the comparison.
                        uint32_t wlast = gpio_get_all(), wcur;
                        uint32_t wfull = wlast & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS15_PIN_MASK) != wfull) break;
                            wlast = wcur;
                        }
                        ram_table[wlast & 0x3fff] = (wlast >> D0_PIN) & 0xff;
                        rom_in_use = 0;
                  } else {
                    if (rom_in_use) {
                        SET_DATA_MODE_IN;
                        rom_in_use = 0;
                    }
                  }
                }
            } else {
                // $0000-$3FFF. The $0450/$0440/$0800 POKEY windows live here, below
                // every SuperGame window, so they cannot collide with ROM/RAM.
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
// change is needed. MegaCart+ is the same board with a 5-bit mask and up to
// 512KB of ROM; deliberately NOT implemented, since 512KB cannot fit rom_table
// and MAME only picks it over VersaBoard above 256KB (a78_slot.cpp:424).
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
        // A15 is gpio 26 on this board, so it is tested in the RAW word, and
        // addr stays a 15-bit offset - exactly as emulate_supercart_ram() does.
        if (rawaddr & A15_PIN_MASK) {
            if (addr & A14_PIN_MASK) {
                // $C000-$FFFF: fixed last bank
                sio_hw->gpio_out = (uint32_t)rom_table[addr + fixed_base] << D0_PIN;
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
                        // Krok 19 end-of-cycle capture; A15 sits outside
                        // BUS_PIN_MASK, so compare BUS15_PIN_MASK.
                        uint32_t last = gpio_get_all(), cur;
                        uint32_t full = last & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS15_PIN_MASK) != full) break;
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
                        uint32_t wfull = wlast & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS15_PIN_MASK) != wfull) break;
                            wlast = wcur;
                        }
                        ram_table[(wlast & 0x3fff) + ram_bank] = (wlast >> D0_PIN) & 0xff;
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

// POKEY-cart variant of emulate_versa(). Kept as a SEPARATE function for the same
// reason as every other _pokey variant (version 0.18): pokey_base is volatile, so
// a shared helper re-reads it on every pass through the cold branch and that is
// what made "Alien Brigade" glitch. MAME gives this board its own device,
// a78_versapokey (cpuwiz.cpp:41), because some VersaBoard demos combine banked RAM
// with a POKEY at $0450 for XBoarD/XM compatibility - here that is exactly one
// file, "Mario Bros (Ice Stress Test)" (header 0x0062).
//
// NOTE: this board has no audio output line at all (see pokey.h), so the POKEY
// here only keeps the bus behaviour correct - the writes are absorbed instead of
// being mistaken for something else. It cannot produce sound on this hardware.
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
        if (rawaddr & A15_PIN_MASK) {
            if (addr & A14_PIN_MASK) {
                sio_hw->gpio_out = (uint32_t)rom_table[addr + fixed_base] << D0_PIN;
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
                        uint32_t full = last & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            cur = gpio_get_all();
                            if ((cur & BUS15_PIN_MASK) != full) break;
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
                        uint32_t wfull = wlast & BUS15_PIN_MASK;
                        for (uint32_t g = 0; g < 64; g++) {
                            wcur = gpio_get_all();
                            if ((wcur & BUS15_PIN_MASK) != wfull) break;
                            wlast = wcur;
                        }
                        ram_table[(wlast & 0x3fff) + ram_bank] = (wlast >> D0_PIN) & 0xff;
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

// POKEY-cart variant of emulate_supercart_large(). A SEPARATE function so carts without POKEY
// keep byte-identical code to the version proven on hardware.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_supercart_large_pokey()) {
  // v0.13 (P2): core-1 IRQs off for the lifetime of the emulation loop. Arduino
  // libraries can install handlers on whichever core first uses them; a single
  // preemption inside the bus-response window is one corrupted byte that can
  // never be reproduced. Core 0 (USB/menu) is unaffected; this function never
  // returns, so nothing needs restoring.
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
      // Hoisted: pokey_base/pokey_mask are volatile, and reading them from
      // memory on every pass through the cold branch is what made a cart with
      // no POKEY at all (Alien Brigade) start glitching on the other board.
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
        if (rawaddr & A15_PIN_MASK) {
          addr=addr | 0x8000;
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
                        // Krok 20: end-of-cycle capture, proven on CART_TYPE_SUPERCART_RAM
                        // in Krok 19 - the 6502 drives the data lines only in the second
                        // half of a write cycle, and this polling loop catches the write at
                        // a random phase. Bounded at 64 turns; an unbounded wait hung the
                        // cart in Krok 18. A15 is gpio 26 here, outside BUS_PIN_MASK, so
                        // compare BUS15_PIN_MASK.
                        {
                          uint32_t last = gpio_get_all(), cur;
                          uint32_t full = last & BUS15_PIN_MASK;
                          for (uint32_t g = 0; g < 64; g++) {
                              cur = gpio_get_all();
                              if ((cur & BUS15_PIN_MASK) != full) break;
                              last = cur;
                          }
                          rawaddr = last;
                        }
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
                // $0000-$3FFF. The $0450/$0440/$0800 POKEY windows live here, below
                // every SuperGame window, so they cannot collide with ROM/RAM.
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

// ===========================================================================
// Bankset (7800) - TODO.md "Pozycja 3"
// ===========================================================================
//
// A Bankset cartridge carries TWO complete images in one file: the first half
// is what Sally (the 6502) sees, the second half is what Maria (the video DMA
// engine) sees at the very same addresses. The cartridge picks between them
// combinationally from the console's HALT line - there is no register and no
// state, so all this firmware has to do is sample HALT in the same
// gpio_get_all() that gives it the address. That is what BANKSET_MARIA_OFFSET
// below does, in four single-cycle instructions and no branch.
//
// Sources (both transcribed, then cross-checked against each other by
// tools/bankset_sim/):
//   * MAME  ORIG/MAME-A7800/src/devices/bus/a7800/bankset.cpp - the reference
//     implementation, by the scheme's own author (Mike Saarna). Header ->
//     board selection is a78_slot.cpp:409-470, "switch (mapper & 0xe02e)".
//   * test7800 ORIG/test7800/hardware/memory/external/banksets.go - an
//     independent implementation; agrees on every mapping we need.
//
// HALT POLARITY. /HALT is asserted LOW: Maria pulls it down to take the bus
// away from Sally, exactly the way RDY stops a 6502. So HALT low = serve
// Maria's half. This is the one fact here that is NOT verified on our own
// hardware, so it is a single #define (BANKSET_HALT_ACTIVE_LOW) and there is
// a diagnostic build with the opposite sense - see build.sh, target
// PicoA10400-BSHI. A wrong guess is unmistakable rather than subtle: the
// reset vector itself would come from the wrong half, so the cart would not
// boot at all.
//
// UNCONNECTED HALT. The pad default on an RP2040 is pull-DOWN, i.e. a floating
// HALT would read "asserted" and every fetch would come from Maria's half.
// setup_bankset() therefore switches the pin to pull-UP, so a board that does
// not route HALT degrades to "Sally's half always" - the game boots and only
// the graphics are wrong - instead of failing at the reset vector.

// Maria's half is at + (romLen/2) for as long as HALT says Maria owns the bus.
// Written branchlessly: (bit - 1) is 0 when the bit is set and 0xFFFFFFFF when
// it is clear, so the AND either passes `half` through or zeroes it. On the
// Pico 2 board the HALT line is not routed at all (see the comment on
// BANKSET_HAS_HALT), and this collapses to a compile-time 0.
#if BANKSET_HAS_HALT
#if BANKSET_HALT_ACTIVE_LOW
#define BANKSET_MARIA_OFFSET(raw, half) \
  ((half) & ((((raw) >> HALT_PIN) & 1u) - 1u))
#else
#define BANKSET_MARIA_OFFSET(raw, half) \
  ((half) & (0u - (((raw) >> HALT_PIN) & 1u)))
#endif
#else
#define BANKSET_MARIA_OFFSET(raw, half) (0u)
#endif

// --- diagnostic switches, both default OFF -----------------------------------
//
// BANKSET_DIAG_CHARWIN ("is HALT usable AT ALL?") - apply the HALT selection
// ONLY inside $E000-$E7FF and serve Sally's half everywhere else.
//
// Why that window: the official Bankset demo draws in MARIA's character mode
// with CHARBASE = $E0, so every glyph MARIA fetches comes from $E000-$E7FF,
// and SALLY never executes from there (in Sally's half that region is all
// zeroes - measured, tools/bankset_sim/dump_display_list.py). So this build
// cannot corrupt the CPU: the demo is guaranteed to run and paint its purple
// background ($60, measured with dump_colors.py). The only question left on
// screen is whether the TEXT appears - which happens if and only if the HALT
// line is a usable "MARIA owns the bus" indicator at the moment MARIA fetches.
//
// BANKSET_TRACK_HALT ("follow HALT the way the real board does") - a real
// Bankset cartridge is combinational: its ROM output follows HALT for the whole
// bus cycle. The production loop samples HALT ONCE, with the address, and then
// holds that byte until the address changes. If HALT moves mid-cycle - which it
// must, at both ends of every DMA burst - the console latches a byte from the
// wrong half. This build keeps the output live instead, re-reading HALT and
// re-driving inside the wait, which is what the hardware being emulated does.
#ifndef BANKSET_DIAG_CHARWIN
#define BANKSET_DIAG_CHARWIN 0
#endif
#ifndef BANKSET_TRACK_HALT
#define BANKSET_TRACK_HALT 0
#endif

#if BANKSET_DIAG_CHARWIN
#define BANKSET_SEL(raw, half, addr) \
  ((((addr) & 0xF800u) == 0xE000u) ? BANKSET_MARIA_OFFSET(raw, half) : 0u)
#else
#define BANKSET_SEL(raw, half, addr) BANKSET_MARIA_OFFSET(raw, half)
#endif

// BANKSET_STICKY_N ("trust HALT only after it has been low for a WHILE").
//
// The hardware said, in this order: BSCH works (so HALT reads correctly while
// MARIA fetches), production hangs (so SALLY's fetches get MARIA's half), and
// BSTR hangs too (so it is not a matter of WHEN inside the cycle we sample -
// following HALT for the whole cycle changes nothing). The only reading left is
// that HALT is low for the ENTIRE duration of some Sally accesses, i.e. Sally
// is still using the bus after MARIA has asserted HALT and before MARIA
// actually starts fetching.
//
// A rule based on the level alone cannot separate those. A rule based on how
// LONG the level has held can: a DMA burst holds HALT low across many
// consecutive accesses, while the leaked Sally cycles sit at its very start. So
// count consecutive stable addresses seen with HALT low, and only switch to
// Maria's half once that run reaches N.
//
// The error is deliberately asymmetric. Serving Sally's half to MARIA costs one
// wrong graphics byte; serving MARIA's half to SALLY costs one wrong opcode,
// which is fatal. N therefore biases towards Sally: the first N-1 fetches of
// each burst come from Sally's half, which at worst blanks the leading
// character of a row.
//
// MEASURED ON HARDWARE, 2026-09-02 (PAL console), swept the way the E8
// drive-strength experiment was:
//
//     N = 2   yellow screen - still hangs
//     N = 8   WORKS - the Bankset demos AND StoneAge, a real game
//
// So the leaked Sally window is longer than 2 accesses and no longer than 8,
// and 8 is the production default. It costs nothing visible because the counter
// runs on EVERY bus access, not only cartridge ones: MARIA reads its display
// list and character codes from console RAM at the start of each burst, which
// saturates the run long before it fetches a single glyph from the cartridge.
//
// 0 disables it (the pre-0.38 behaviour, kept for bisecting). Any value can be
// swept from the compile line: build.sh targets PicoA10400-BSK<n>.
#ifndef BANKSET_STICKY_N
#define BANKSET_STICKY_N 8
#endif

// A board without the HALT line has nothing to be hysteretic ABOUT: the Maria
// offset is a compile-time zero there, and the counter would only reference a
// HALT_PIN_MASK that such a board does not define. Force it off.
//
// This is not hypothetical - promoting the default from 0 to 8 broke the Pico 2
// build outright, and it went unnoticed for one round because the build output
// was being filtered for success lines instead of checked for an exit status.
// tools/rebuild_all_bankset.sh now reports failures.
#if !BANKSET_HAS_HALT
#undef BANKSET_STICKY_N
#define BANKSET_STICKY_N 0
#endif

// R/W sampled TWICE and OR-ed: "read" wins.
//
// patches/PicoA10400_0.15.txt, "WHAT TO WATCH OUT FOR", is explicit that making
// the data output conditional on a SINGLE R/W sample costs the design its
// tolerance for one bad sample, and that three hardware failures on this board
// were traced to exactly one mis-read. Reshaping emulate_bankset_ram() put such
// a single-sample test in front of its $4000-$7FFF and $C000-$FFFF windows -
// and $C000-$FFFF is where the 6502 vectors and interrupt handlers live. On
// "Bankset Test 2x32K RAM Pokey800" the NMI vector is $F8E5 and the music
// player runs from that handler, so one mis-read there costs an opcode inside
// the routine that makes the sound: picture fine, music gone. Which is exactly
// what hardware reported, and it is why the FLAT loop - which never gates its
// drive on R/W at all - keeps its sound (StoneAge).
//
// Two samples, OR-ed, restore the tolerance without giving up the write
// capture: one spurious "write" no longer steals a fetch, while one spurious
// "read" during a real write costs a captured byte - a pixel, not an opcode.
// Same asymmetry as the HALT hysteresis, for the same reason.
#define BANKSET_IS_READ() \
  ((gpio_get_all() | gpio_get_all()) & RW_PIN_MASK)

// Hold the data lines through a READ - and ONLY through a read.
//
// A 6502 does not always change the address between a read and a write. An
// indexed store (STA abs,X / abs,Y / (zp),Y) performs a READ cycle at the
// target address right before the WRITE cycle (the "dummy read", cycle 4 of
// STA abs,Y - it happens whether or not the index crosses a page), and every
// read-modify-write instruction (INC/DEC/ASL/LSR/ROL/ROR abs) reads its
// operand and then writes at the same address twice. Between those cycles
// only R/W moves. A wait keyed on the address alone - the flat loop's
// "while (address unchanged)" - is right for ROM, where nothing ever writes,
// and wrong for RAM: it keeps driving straight through the write cycle and
// never captures the byte.
//
// That is exactly how "Bankset Test 2x32K RAM Pokey800" lost its music. It
// copies its 4 KB song into cartridge RAM with STA $4000,Y (sixteen pages,
// $809B-$8144) and the player then reads the song from there ($8146: song
// address $4000 handed to the init at $F27C). Every byte of that copy was
// lost, the player read zeros, and the POKEY got nothing but the silence
// pattern - while the picture, which lives in console RAM and ROM, stayed
// perfect. tools/bankset_sim/dummy_read_probe.py replays both loops against
// the 6502's bus timing: the shipped one drives against Sally for 228 ns of
// every indexed store and captures nothing; this one captures the byte.
//
// Exits with wr = 1 when R/W read LOW in two consecutive samples while the
// address stayed (a confirmed write at this address, still in progress), and
// wr = 0 when the address moved on. Two samples for the same reason
// BANKSET_IS_READ() takes two: one spurious "write" must not drop the drive
// in the middle of a real fetch. Should a spurious pair still slip through,
// the cost is bounded - the drive is released a few ns early and the byte
// "captured" is the one the bus still holds, i.e. our own, so the cell is
// written back with the value it already had.
// 0.40 adds the third parameter. While MARIA owns the bus there is nothing to
// watch for: MARIA only ever READS, so a "write" sampled inside one of its
// fetches is a mis-read, and acting on one costs the whole fetch - we stop
// driving, spend the rest of the cycle in the capture loop, and MARIA latches
// whatever the bus still holds. That is one wrong 8-pixel row of one character,
// i.e. a short horizontal dash that comes and goes. See BANKSET_MARIA_NOW in
// emulate_bankset_ram() for the measurement this comes from.
#define BANKSET_HOLD_WHILE_READ(addr, wr, maria)                               \
  do {                                                                         \
    (wr) = 0u;                                                                 \
    for (;;) {                                                                 \
      uint32_t h1_ = gpio_get_all();                                           \
      if ((h1_ & BUS_PIN_MASK) != (addr)) break;                               \
      if ((maria) || (h1_ & RW_PIN_MASK)) continue;                            \
      uint32_t h2_ = gpio_get_all();                                           \
      if ((h2_ & BUS_PIN_MASK) != (addr)) break;                               \
      if (!(h2_ & RW_PIN_MASK)) { (wr) = 1u; break; }                          \
    }                                                                          \
  } while (0)

// SALLY OWNS THE BUS - the rule the reference implementation states as an
// assertion. test7800's banksets.go:128 begins its Access() with
//
//     if write && ext.hlt { panic("MARIA should not be writing to memory") }
//
// i.e. a write while HALT is asserted is IMPOSSIBLE: MARIA only ever reads, and
// SALLY is off the bus. So any "write" we see with HALT asserted is not a write
// at all - it is an undriven bus, and an undriven bus reads as zero, because the
// RP2040 pads default to pull-down.
//
// NOTE (0.39): the evidence originally cited here was misread. The thousands of
// zero-byte captures on "Bankset Test 2x32K RAM Pokey800" were REAL writes by a
// real music player; the data was zero because the song itself had been lost on
// the way into cartridge RAM. The RULE above is unaffected - it comes from the
// reference implementation, not from that measurement - and 0.40 puts it to its
// proper use, on the READ path, where ignoring R/W during a MARIA fetch is what
// keeps the fetch (see BANKSET_MARIA_NOW in emulate_bankset_ram()).
//
// Gating the capture on "Sally owns the bus" throws all of them away and costs
// nothing real, because a genuine POKEY write can only ever happen when Sally is
// driving. On a board without a HALT line this collapses to a compile-time 1.
#if BANKSET_HAS_HALT
#if BANKSET_HALT_ACTIVE_LOW
#define BANKSET_SALLY_OWNS(raw)  ((raw) & HALT_PIN_MASK)
#else
#define BANKSET_SALLY_OWNS(raw)  (!((raw) & HALT_PIN_MASK))
#endif
#else
#define BANKSET_SALLY_OWNS(raw)  (1)
#endif

// BANKSET_DIAG_TONE ("do the POKEY writes reach this loop at all?").
//
// "Bankset Test 2x32K RAM Pokey800" has a correct picture and no music, and the
// music player is known to exist: scan_pokey_stores.py finds stores to $0815
// and $0818,Y in Sally's half, reached from the NMI handler at $F8E5. What is
// not known is whether those writes reach the bus loop.
//
// This build makes the answer audible without depending on the data at all.
// The cart's own initialisation routine writes the POKEY 29 times (all zeroes,
// measured), so the counter has to clear that before it means anything: after
// the 64th captured write - which only a running PLAYER can produce - a fixed
// audible tone is forced into pokey_regs[] and capturing stops, so the tone
// stays.
//
//     tone after a moment -> the player's writes ARE reaching the loop, and the
//                            fault is downstream (data or synthesis)
//     silence             -> they are NOT reaching it, and the fault is in how
//                            this loop sees the $0800 window
// Prog jest WARTOSCIA tego przelacznika, nie stala w kodzie - bo dobranie go
// zle raz juz zepsulo wniosek. Rutyna inicjalizujaca tego kartridza wykonuje
// DOKLADNIE 29 zapisow (zmierzone, dump_pokey_writes.py), wiec prog 64 lezy
// zaledwie 35 nad nia: kilkanascie przypadkowych przechwycen wystarczy, zeby go
// przekroczyc, a wtedy ton nie mowi nic o odtwarzaczu. Prog rzedu tysiecy moze
// osiagnac wylacznie cos, co pisze bez przerwy.
#ifndef BANKSET_DIAG_TONE
#define BANKSET_DIAG_TONE 0
#endif

// BANKSET_DIAG_PKFIRST - handle the aux-chip window on the first sample, in the
// RAM loop only. See the block it inserts, in emulate_bankset_ram().
#ifndef BANKSET_DIAG_PKFIRST
#define BANKSET_DIAG_PKFIRST 0
#endif

// BANKSET_DIAG_NORMALLOOP - route the banked-RAM Bankset cart through the
// ordinary emulate_normala78_pokey() instead. See the dispatch in setup1().
#ifndef BANKSET_DIAG_NORMALLOOP
#define BANKSET_DIAG_NORMALLOOP 0
#endif

#if BANKSET_DIAG_TONE
static uint32_t bankset_pk_seen = 0;
#define BANKSET_CAPTURE(reg, val)                                              \
  do {                                                                         \
    uint32_t n_ = ++bankset_pk_seen;                                           \
    if (n_ == (uint32_t)BANKSET_DIAG_TONE) {                                                           \
      pokey_regs[0x00] = 60;      /* AUDF1 */                                  \
      pokey_regs[0x01] = 0xA8;    /* AUDC1: pure tone, volume 8 */             \
      pokey_regs[0x08] = 0x00;    /* AUDCTL */                                 \
      pokey_regs[0x0F] = 0x03;    /* SKCTL released */                         \
    } else if (n_ < (uint32_t)BANKSET_DIAG_TONE) {                             \
      pokey_capture_write((reg), (val));                                       \
    }                                                                          \
  } while (0)
#else
#define BANKSET_CAPTURE(reg, val) pokey_capture_write((reg), (val))
#endif

// BANKSET_DIAG_TONE2 ("czy dociera JAKIKOLWIEK rozkaz zrobienia dzwieku?").
//
// _BSAU pokazalo, ze zapisy docieraja i synteza gra; bramka "tylko zapisy" nic
// nie zmienila. Zostaly WARTOSCI. Ta sonda arm-uje slyszalny ton dopiero wtedy,
// gdy przechwycimy zapis do rejestru GLOSNOSCI (AUDC1/2/3/4 = 1,3,5,7)
// z niezerowa glosnoscia w bitach 0-3 - czyli jedyny rodzaj zapisu, ktory
// w ogole moze cos zagrac.
//
//     ton    -> rozkazy glosnosci DOCIERAJA poprawnie; wina jest dalej
//               (czestotliwosc, AUDCTL, albo interpretacja w syntezie)
//     cisza  -> do rejestrow AUDCx trafiaja same zera; przechwytujemy zle DANE
#ifndef BANKSET_DIAG_TONE2
#define BANKSET_DIAG_TONE2 0
#endif

#if BANKSET_DIAG_TONE2
static uint32_t bankset_pk_armed = 0;
#undef BANKSET_CAPTURE
#define BANKSET_CAPTURE(reg, val)                                              \
  do {                                                                         \
    uint32_t r_ = (uint32_t)(reg), v_ = (uint32_t)(val);                       \
    if (!bankset_pk_armed &&                                                   \
        (r_ == 1u || r_ == 3u || r_ == 5u || r_ == 7u) && (v_ & 0x0Fu)) {      \
      pokey_regs[0x00] = 60;      /* AUDF1 */                                  \
      pokey_regs[0x01] = 0xA8;    /* AUDC1: pure tone, volume 8 */             \
      pokey_regs[0x08] = 0x00;    /* AUDCTL */                                 \
      pokey_regs[0x0F] = 0x03;    /* SKCTL released */                         \
      bankset_pk_armed = 1;                                                    \
    } else if (!bankset_pk_armed) {                                            \
      pokey_capture_write(r_, (uint8_t)v_);                                    \
    }                                                                          \
  } while (0)
#endif

// BANKSET_DIAG_TONE3 ("czy widzimy JAKIKOLWIEK niezerowy bajt danych?").
//
// _BSA2 dalo cisze: zaden zapis do rejestru glosnosci z niezerowa wartoscia nie
// dotarl. Zostaja dwie mozliwosci i ta sonda je rozdziela - arm-uje ton przy
// pierwszym przechwyconym zapisie o NIEZEROWYM BAJCIE DANYCH, obojetnie do
// ktorego rejestru. Rutyna inicjalizujaca tego kartridza pisze same zera
// (zmierzone), wiec nie moze tego wywolac.
//
//     ton   -> niezerowe dane DOCIERAJA; probkowanie magistrali dziala, a
//              problemem jest to, ktore zapisy widzimy
//     cisza -> KAZDY przechwycony bajt to $00; probkujemy dane ZANIM procesor
//              je wystawi (pady RP2040 maja domyslnie sciaganie do masy, wiec
//              nienapedzana magistrala czyta sie jako zero)
#ifndef BANKSET_DIAG_TONE3
#define BANKSET_DIAG_TONE3 0
#endif

#if BANKSET_DIAG_TONE3
static uint32_t bankset_pk_armed3 = 0;
#undef BANKSET_CAPTURE
#define BANKSET_CAPTURE(reg, val)                                              \
  do {                                                                         \
    uint32_t r_ = (uint32_t)(reg), v_ = (uint32_t)(val);                       \
    if (!bankset_pk_armed3 && v_ != 0u) {                                      \
      pokey_regs[0x00] = 60;      /* AUDF1 */                                  \
      pokey_regs[0x01] = 0xA8;    /* AUDC1: pure tone, volume 8 */             \
      pokey_regs[0x08] = 0x00;                                                 \
      pokey_regs[0x0F] = 0x03;                                                 \
      bankset_pk_armed3 = 1;                                                   \
    } else if (!bankset_pk_armed3) {                                           \
      pokey_capture_write(r_, (uint8_t)v_);                                    \
    }                                                                          \
  } while (0)
#endif

// BANKSET_DIAG_TONE4 ("czy do AUDCx trafia COKOLWIEK niezerowego?").
//
// _BSAU4096 pokazalo, ze zapisy odtwarzacza docieraja tysiacami (inicjalizacja
// robi 29). _BSA2 pokazalo, ze zaden z nich nie niesie niezerowej GLOSNOSCI
// (bity 0-3 rejestru AUDCx). Ta sonda pyta o slabszy warunek: czy do AUDC1/2/3/4
// trafia w ogole jakikolwiek niezerowy bajt - bo AUDCx niesie tez bity ksztaltu
// fali (zwykle $A0), ktore odtwarzacz musi ustawiac.
//
//     ton   -> bajty AUDCx docieraja, ale z wyzerowana MLODSZA POLOWKA;
//              czytamy dane czesciowo - problem w probkowaniu magistrali
//     cisza -> do AUDCx nie trafia NIC niezerowego, mimo tysiecy przechwycen;
//              czyli trafiaja tam bajty z innych zapisow albo same zera
#ifndef BANKSET_DIAG_TONE4
#define BANKSET_DIAG_TONE4 0
#endif

#if BANKSET_DIAG_TONE4
static uint32_t bankset_pk_armed4 = 0;
#undef BANKSET_CAPTURE
#define BANKSET_CAPTURE(reg, val)                                              \
  do {                                                                         \
    uint32_t r_ = (uint32_t)(reg), v_ = (uint32_t)(val);                       \
    if (!bankset_pk_armed4 &&                                                  \
        (r_ == 1u || r_ == 3u || r_ == 5u || r_ == 7u) && v_ != 0u) {          \
      pokey_regs[0x00] = 60;                                                   \
      pokey_regs[0x01] = 0xA8;                                                 \
      pokey_regs[0x08] = 0x00;                                                 \
      pokey_regs[0x0F] = 0x03;                                                 \
      bankset_pk_armed4 = 1;                                                   \
    } else if (!bankset_pk_armed4) {                                           \
      pokey_capture_write(r_, (uint8_t)v_);                                    \
    }                                                                          \
  } while (0)
#endif

// BANKSET_DIAG_TONE5 - jak TONE, ale liczy WYLACZNIE przechwycenia
// z NIEZEROWYM bajtem danych. Prog jest wartoscia.
//
// Ostatni czysty rozdzial. Wiadomo juz, ze przechwytujemy tysiace zapisow
// w 32-bajtowym oknie $0800-$081F (_BSAU4096 uzbraja ton takze PO zawezeniu),
// a mimo to zaden nie niesie niezerowego bajtu do AUDCx (_BSA2, _BSA5).
// Sonda TONE3 pytala o to samo, ale z progiem 1 - odpalala wiec na rutynie
// inicjalizujacej, ktora pisze SKCTL=$03. Z progiem rzedu tysiecy inicjalizacja
// (29 zapisow) nie ma szans.
//
//     ton   -> tysiace przechwycen ma sensowne DANE; czytanie magistrali
//              dziala, a bledny jest NUMER REJESTRU
//     cisza -> po inicjalizacji KAZDY przechwycony bajt to $00; czytamy dane
//              systematycznie jako zero
#ifndef BANKSET_DIAG_TONE5
#define BANKSET_DIAG_TONE5 0
#endif

#if BANKSET_DIAG_TONE5
static uint32_t bankset_pk_nz = 0;
#undef BANKSET_CAPTURE
#define BANKSET_CAPTURE(reg, val)                                              \
  do {                                                                         \
    uint32_t r_ = (uint32_t)(reg), v_ = (uint32_t)(val);                       \
    if (v_ != 0u) bankset_pk_nz++;                                             \
    if (bankset_pk_nz == (uint32_t)BANKSET_DIAG_TONE5) {                       \
      pokey_regs[0x00] = 60;                                                   \
      pokey_regs[0x01] = 0xA8;                                                 \
      pokey_regs[0x08] = 0x00;                                                 \
      pokey_regs[0x0F] = 0x03;                                                 \
      bankset_pk_nz++;                                                         \
    } else if (bankset_pk_nz < (uint32_t)BANKSET_DIAG_TONE5) {                 \
      pokey_capture_write(r_, (uint8_t)v_);                                    \
    }                                                                          \
  } while (0)
#endif




// Maria-half offset for the three loops that are NOT the flat one. The flat
// loop keeps its own inline form, byte for byte as it was tested at N=8 - the
// counter there also ticks while the loop is parked on a sub-$4000 address, and
// that is part of what was measured, so it is deliberately not "tidied up".
//
// Here the run counts DISTINCT ADDRESSES, which is the same measure of the
// thing that actually matters: how many consecutive bus accesses have carried
// HALT low. A high sample resets it immediately - errors are biased towards
// Sally, because a wrong byte to MARIA is one wrong pixel and a wrong byte to
// SALLY is a wrong opcode.
#if BANKSET_STICKY_N
#define BANKSET_MO(raw, lowrun, h)  (((lowrun) >= BANKSET_STICKY_N) ? (uint32_t)(h) : 0u)
#define BANKSET_RUN_DECL            uint32_t lowrun = 0, lastaddr = 0xFFFFFFFFu
#define BANKSET_RUN_STEP(raw, addr)                                            \
  do {                                                                         \
    if ((raw) & HALT_PIN_MASK) lowrun = 0;                                     \
    else if ((addr) != lastaddr) {                                             \
      lastaddr = (addr);                                                       \
      if (lowrun < BANKSET_STICKY_N) lowrun++;                                 \
    }                                                                          \
  } while (0)
#else
#define BANKSET_MO(raw, lowrun, h)  BANKSET_MARIA_OFFSET(raw, (h))
#define BANKSET_RUN_DECL            const uint32_t lowrun = 0
#define BANKSET_RUN_STEP(raw, addr) do { } while (0)
#endif

// Bank number mask for ONE bankset half. MAME derives it from the whole file
// (a78_rom_sg_device: nbanks odd ? nbanks-2 : nbanks-1) and then halves it on
// every use - "m_bank_mask/2" appears in each read_40xx in bankset.cpp.
//
// Written out in both SuperGame loops below rather than shared in a helper ON
// PURPOSE. GCC will not inline across an __attribute__((optimize)) boundary, so
// a static inline helper called from an -O2 __time_critical_func lands in FLASH
// and is reached through a RAM veneer - the exact shape PicoA10400_tune/ was
// written to hunt down. It only ran once per game here, but the rule in this
// file is that nothing in an emulate_* function calls into flash, and
// tools/check_bankset_hotpath.py enforces it.
#define BANKSET_BANK_MASK_DECL(name, rom_len)                                  \
  const uint32_t name##_nbanks = (uint32_t)(rom_len) / 0x4000u;                \
  const uint32_t name = ((name##_nbanks < 2u) ? 0u                             \
                       : ((name##_nbanks & 1u) ? (name##_nbanks - 2u)          \
                                               : (name##_nbanks - 1u))) >> 1

// Flat Bankset: 2x32K, 2x48K or 2x52K, no cartridge RAM.
// MAME a78_bankset_rom_device / _p800 / _p4000 / _52k (bankset.cpp:296-420).
// The image is mapped to the TOP of the address space, exactly like any other
// flat 7800 cart: origin = 0x10000 - (romLen/2). For a 52K half that origin is
// $3000, and MAME installs a read handler there for precisely this scheme
// (a7800.cpp:1483-1489) - the $2800-$3FFF "RAM mirror" the Software Guide
// claims is not real on hardware, which is also how the High Score Cartridge
// gets to put ROM at $3000.
//
// Shape is emulate_normala78()'s, unchanged: two matching address samples,
// drive, block until the address moves, release. That is the loop that has
// been through the not_working_roms4 hardware iterations for flat 7800 carts,
// and a Bankset flat cart is a flat cart with one extra index term.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_bankset_flat()) {
  __asm volatile ("cpsid i" ::: "memory");   // plain CPSID: no CMSIS dependency
  uint32_t raw, addr, addr_prev = 0xFFFFFFFF;
  const uint32_t half   = (uint32_t)romLen >> 1;
  const uint32_t origin = 0x10000u - half;
  // Only a 52K half is allowed to answer below $4000 - that is the one size
  // MAME gives a read_30xx handler. Anything else is clamped to the cartridge
  // window so a malformed header can never make us drive over console RAM.
  const uint32_t lo = (half == 0xD000u) ? 0x3000u
                    : ((origin < 0x4000u) ? 0x4000u : origin);
  const uint32_t pkbase = (uint32_t)pokey_base;   // volatile: hoist out of the loop
  const uint32_t pkmask = (uint32_t)pokey_mask;
  const uint32_t ymon   = (uint32_t)ym_enabled;
  // A POKEY at $4000 on a 48K/52K half sits INSIDE the ROM window, so it has to
  // be write-only there: reads must still return ROM (JS7800 calls this exact
  // case cartridge_pokey_write_only, Cartridge.js:332). pk_ovl is the base of
  // that 16-byte overlap, or an address the bus can never carry when there is
  // none - so "(addr & 0xFFF0) == pk_ovl" is one AND and one compare against a
  // register, never taken for any other cart.
  //
  // It must be a WINDOW, not just an upper bound: a 52K half starts at $3000,
  // so a bare "addr < $4010" would also swallow $3000-$3FFF and capture a write
  // there as a POKEY register.
  const uint32_t pk_ovl = (pokey_enabled && pkbase == 0x4000u && lo <= 0x4000u)
                        ? 0x4000u : 0xFFFFFFFFu;

#if BANKSET_STICKY_N
  uint32_t lowrun = 0;      // consecutive stable addresses seen with HALT low
#endif

  while (1) {
    raw  = gpio_get_all();
    addr = raw & BUS_PIN_MASK;
    if (addr != addr_prev) { addr_prev = addr; continue; }   // need two matching samples
    // got a stable address
#if BANKSET_STICKY_N
    if (raw & HALT_PIN_MASK) lowrun = 0;
    else if (lowrun < BANKSET_STICKY_N) lowrun++;
#define BANKSET_OFFSET_NOW  ((lowrun >= BANKSET_STICKY_N) ? half : 0u)
#else
#define BANKSET_OFFSET_NOW  BANKSET_SEL(raw, half, addr)
#endif
    if (addr >= lo) {
      const uint32_t off = addr - origin;
      const uint32_t idx = BANKSET_OFFSET_NOW + off;
      if ((addr & 0xFFF0u) == pk_ovl) {
        // POKEY window overlapping ROM: serve ROM on a read, listen on a write.
        if (gpio_get_all() & RW_PIN_MASK) {
          sio_hw->gpio_out = (uint32_t)rom_table[idx] << D0_PIN;
          SET_DATA_MODE_OUT;
          while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
          SET_DATA_MODE_IN;
        } else {
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS_PIN_MASK) != addr) break;
            last = cur;
          }
          BANKSET_CAPTURE(addr & 0x0F, (uint8_t)((last >> D0_PIN) & 0xFF));
        }
#if BANKSET_TRACK_HALT
      } else {
        // Keep the byte LIVE for the whole cycle, re-reading HALT each turn -
        // the real board is combinational and does exactly this. ~7 instructions
        // per turn at 250MHz, i.e. tens of refreshes inside MARIA's ~279ns.
        SET_DATA_MODE_OUT;
        do {
          raw = gpio_get_all();
          sio_hw->gpio_out =
            (uint32_t)rom_table[BANKSET_OFFSET_NOW + off] << D0_PIN;
        } while ((raw & BUS_PIN_MASK) == addr);
        SET_DATA_MODE_IN;
      }
#else
      } else {
        sio_hw->gpio_out = (uint32_t)rom_table[idx] << D0_PIN;  // D0-D7 are the only outputs in 7800 modes
        SET_DATA_MODE_OUT;
        while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
        SET_DATA_MODE_IN;
      }
#endif
    } else if ((addr & pkmask) == pkbase && ymon) {
      ym_window_service_blocking(addr);
    } else if ((addr & pkmask) == pkbase) {
      // LISTEN ONLY, same end-of-cycle capture as emulate_normala78_pokey().
      uint32_t last = gpio_get_all(), cur;
      for (uint32_t g = 0; g < 64; g++) {
        cur = gpio_get_all();
        if ((cur & BUS_PIN_MASK) != addr) break;
        last = cur;
      }
      BANKSET_CAPTURE(addr & 0x0F, (uint8_t)((last >> D0_PIN) & 0xFF));
    }
  }
}

// Flat Bankset with banked RAM: 16K for Sally and 16K for Maria at $4000-$7FFF.
// MAME a78_bankset_bankram_device / _p800 (bankset.cpp:421-520).
//
//   read  $4000-$7FFF -> RAM, Sally's or Maria's bank per HALT
//   read  >= origin   -> ROM, Sally's or Maria's half per HALT
//   write $4000-$7FFF -> Sally's RAM   ("Maria can only read, so this has to
//                                        be Sally's bankset" - bankset.cpp:487)
//   write $C000-$FFFF -> MARIA's RAM. This is the whole point of the scheme:
//                        Maria's RAM is invisible to Sally, so the board gives
//                        Sally a second window onto it that shadows the ROM.
//
// The two 16K banks are ram_table[0x0000..] and ram_table[0x4000..] - the same
// 32KB array VersaBoard already uses for its two banked RAM pages, and the
// same layout MAME uses (m_ram[offset] / m_ram[offset + 0x4000]).
//
// Shape is emulate_supercart_ram()'s: single sample, R/W gating, rom_in_use
// held across a burst, end-of-cycle data capture on writes. A cart with RAM in
// the $4000 window has to see writes, and that is the loop proven to do it.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_bankset_ram()) {
  __asm volatile ("cpsid i" ::: "memory");
  uint32_t raw, addr, addr_prev = 0xFFFFFFFFu;
  const uint32_t half   = (uint32_t)romLen >> 1;
  const uint32_t origin = 0x10000u - half;
  const uint32_t lo     = (origin < 0x8000u) ? 0x8000u : origin;   // RAM owns $4000-$7FFF
  const uint32_t pkbase = (uint32_t)pokey_base;
  const uint32_t pkmask = (uint32_t)pokey_mask;
  const uint32_t ymon   = (uint32_t)ym_enabled;
#if BANKSET_STICKY_N
  uint32_t lowrun = 0;
#endif

  while (1) {
    raw  = gpio_get_all();
    addr = raw & BUS_PIN_MASK;
#if BANKSET_DIAG_PKFIRST
    // Aux-chip window handled on the FIRST sample, before the address is
    // confirmed - which is what the pre-reshape loop did, and that loop was the
    // one that produced sound. The reshape (which fixed the picture) added the
    // confirmation, and the music went. This isolates that one difference.
    if (addr < 0x4000u) {
      if ((addr & pkmask) == pkbase && ymon) {
        ym_window_service_blocking(addr);
      } else if ((addr & pkmask) == pkbase && !BANKSET_IS_READ()
                 && BANKSET_SALLY_OWNS(raw)) {
        // CAPTURE ONLY ON A WRITE. The listen-only rule this is copied from was
        // proven on the 16-byte $4000 and 32-byte $0450 windows; a POKEY at
        // $0800 occupies 2 KB, so an unconditional capture turns any read - or
        // any transient address that lands in that range - into a bogus
        // register write that persists until something overwrites it.
        //
        // This is the OPPOSITE trade from gating the DATA OUTPUT on R/W, which
        // patches/PicoA10400_0.15.txt forbids: losing a real write to one bad
        // sample costs a register update the player repeats next frame, while
        // accepting a non-write corrupts the chip's state.
        uint32_t last = gpio_get_all(), cur;
        for (uint32_t g = 0; g < 64; g++) {
          cur = gpio_get_all();
          if ((cur & BUS_PIN_MASK) != addr) break;
          last = cur;
        }
        BANKSET_CAPTURE(addr & 0x0F, (uint8_t)((last >> D0_PIN) & 0xFF));
      }
      addr_prev = addr;
      continue;
    }
#endif
    if (addr != addr_prev) { addr_prev = addr; continue; }   // two matching samples
#if BANKSET_STICKY_N
    if (raw & HALT_PIN_MASK) lowrun = 0;
    else if (lowrun < BANKSET_STICKY_N) lowrun++;
#define BANKSET_OFF_RW(h)  ((lowrun >= BANKSET_STICKY_N) ? (uint32_t)(h) : 0u)
// MARIA OWNS THE BUS RIGHT NOW. Deliberately the SAME decision that picks the
// half, reused to decide whether R/W is worth looking at at all: if we are
// serving MARIA's bank then we are inside a MARIA fetch, and MARIA never
// writes - test7800's banksets.go:128 states it as an assertion
// ("MARIA should not be writing to memory"). So in such a cycle R/W carries no
// information, and every use of it can only lose a fetch.
//
// Measured on "Bankset Test 2x32K RAM Pokey800" (tools/bankset_sim/
// maria_fetches.py, per zone): EVERY graphics byte of this demo comes from the
// cartridge RAM window. The text at the top is zones 1 and 3, 32 bytes per
// scanline from $7000/$7100 in MARIA's bank; the animation below it takes 32
// bytes from console RAM and only 3-27 from the cartridge. So the text is the
// part with the most exposure to this loop, and one lost fetch there is one
// 8-pixel row of one character - a short horizontal dash that comes and goes.
// That is what hardware reported for 0.39, and it could only appear in 0.39:
// the two stores that fill MARIA's bank at $7000/$7100 are STA $F000,Y and
// STA $F100,Y ($801F, $8025), both INDEXED, so before 0.39 all of that text
// was lost and there was nothing on screen to glitch.
//
// The cost is bounded and known: a genuine Sally write is only ignored if it
// happens after HALT has read low for BANKSET_STICKY_N consecutive accesses,
// which is exactly the state the hysteresis was measured to mean "MARIA's burst
// is really running" (the Sally leak is 2-8 accesses long, N=8). Inside the
// leak BANKSET_MARIA_NOW is false and writes are captured as before.
#define BANKSET_MARIA_NOW  (lowrun >= BANKSET_STICKY_N)
#else
#define BANKSET_OFF_RW(h)  BANKSET_MARIA_OFFSET(raw, (h))
// No hysteresis: fall back to the raw HALT test, which on a board without the
// HALT line is a compile-time zero - so this whole gate folds away and the loop
// is byte for byte 0.39's.
#define BANKSET_MARIA_NOW  (BANKSET_MARIA_OFFSET(raw, 1u) != 0u)
#endif
    if (addr >= 0x4000u) {
      if (addr < 0x8000u) {
        // Cartridge RAM window: MARIA's bank on a read while it owns the bus,
        // SALLY's bank otherwise; a write is always Sally's ("Maria can only
        // read", bankset.cpp:487).
        //
        // The read is held with BANKSET_HOLD_WHILE_READ, not "until the
        // address changes": an indexed store or an RMW instruction writes at
        // the address it has just read, with no address change in between,
        // and the write has to be captured when R/W drops. See the macro.
        //
        // R/W is consulted ONLY when Sally owns the bus. In a MARIA fetch it
        // cannot mean anything (MARIA never writes) and can only cost the
        // fetch, so that cycle is answered the way 0.38 answered it: drive,
        // hold until the address moves, capture nothing. This also puts the
        // byte on the pins sooner, because the entry test is skipped.
        const uint32_t maria_ = BANKSET_MARIA_NOW;
        uint32_t wr;
        if (maria_ || BANKSET_IS_READ()) {
          sio_hw->gpio_out = (uint32_t)ram_table[BANKSET_OFF_RW(0x4000u)
                                                 + (addr & 0x3FFFu)] << D0_PIN;
          SET_DATA_MODE_OUT;
          BANKSET_HOLD_WHILE_READ(addr, wr, maria_);
          SET_DATA_MODE_IN;
        } else {
          wr = 1u;
        }
        if (wr) {
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS_PIN_MASK) != addr) break;
            last = cur;
          }
          ram_table[addr & 0x3FFFu] = (uint8_t)((last >> D0_PIN) & 0xFF);
        }
      } else if (addr >= 0xC000u) {
        // ROM on a read; a WRITE here is Sally filling MARIA's RAM through the
        // $C000-$FFFF shadow - the whole point of the scheme, since Maria's
        // bank is invisible to Sally any other way. Same hold rule as the
        // $4000 window and for the same reason: this cart fills Maria's RAM
        // with STA $E000,Y / STA $F000,Y ($800E-$8025), indexed stores.
        // Same MARIA gate too - a fetch from this window is a fetch like any
        // other, and MARIA reads here on every cart whose graphics live in the
        // top 16K.
        const uint32_t maria_ = BANKSET_MARIA_NOW;
        uint32_t wr;
        if (maria_ || BANKSET_IS_READ()) {
          sio_hw->gpio_out = (uint32_t)rom_table[BANKSET_OFF_RW(half)
                                                 + (addr - origin)] << D0_PIN;
          SET_DATA_MODE_OUT;
          BANKSET_HOLD_WHILE_READ(addr, wr, maria_);
          SET_DATA_MODE_IN;
        } else {
          wr = 1u;
        }
        if (wr) {
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS_PIN_MASK) != addr) break;
            last = cur;
          }
          ram_table[0x4000u + (addr & 0x3FFFu)] = (uint8_t)((last >> D0_PIN) & 0xFF);
        }
      } else if (addr >= lo) {
        // $8000-$BFFF: plain ROM. A flat board has no bank register here, so a
        // write means nothing and this is byte for byte the flat loop's path -
        // no R/W test at all, which is what emulate_normala78() has always done
        // and what 0.15 warns against "cleaning up".
        sio_hw->gpio_out = (uint32_t)rom_table[BANKSET_OFF_RW(half)
                                               + (addr - origin)] << D0_PIN;
        SET_DATA_MODE_OUT;
        while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
        SET_DATA_MODE_IN;
      }
    } else {
      if ((addr & pkmask) == pkbase && ymon) {
        ym_window_service_blocking(addr);
      } else if ((addr & pkmask) == pkbase && !BANKSET_IS_READ()
                 && BANKSET_SALLY_OWNS(raw)) {
        // CAPTURE ONLY ON A WRITE. The listen-only rule this is copied from was
        // proven on the 16-byte $4000 and 32-byte $0450 windows; a POKEY at
        // $0800 occupies 2 KB, so an unconditional capture turns any read - or
        // any transient address that lands in that range - into a bogus
        // register write that persists until something overwrites it.
        //
        // This is the OPPOSITE trade from gating the DATA OUTPUT on R/W, which
        // patches/PicoA10400_0.15.txt forbids: losing a real write to one bad
        // sample costs a register update the player repeats next frame, while
        // accepting a non-write corrupts the chip's state.
        uint32_t last = gpio_get_all(), cur;
        for (uint32_t g = 0; g < 64; g++) {
          cur = gpio_get_all();
          if ((cur & BUS_PIN_MASK) != addr) break;
          last = cur;
        }
        BANKSET_CAPTURE(addr & 0x0F, (uint8_t)((last >> D0_PIN) & 0xFF));
      }
    }
  }
}

// SuperGame Bankset: each half is a bank-switched 128K (or 64K) image.
// MAME a78_bankset_sg_device (bankset.cpp:29-88).
//
//   $4000-$7FFF  second-to-last bank of the half   (bank 6 of 8)
//   $8000-$BFFF  the selected bank; a write here latches it
//   $C000-$FFFF  last bank of the half             (bank 7 of 8)
//   + romLen/2 to every index while Maria owns the bus.
//
// Shape is emulate_supercart_ef()'s, which is the loop these three windows
// already come from - only the bank-6/bank-7 bases stop being hardcoded to a
// 128K image, because a Bankset half can be 64K (Pit Fighter's Alt 1 header).
__attribute__((optimize("O2")))
void __time_critical_func(emulate_bankset_sg()) {
  __asm volatile ("cpsid i" ::: "memory");
  uint32_t raw, addr, bank = 0;
  uint8_t rom_in_use = 0;
  BANKSET_RUN_DECL;
  const uint32_t half      = (uint32_t)romLen >> 1;
  BANKSET_BANK_MASK_DECL(bank_mask, romLen);
  const uint32_t last_bank = bank_mask * 0x4000u;                 // $C000-$FFFF
  const uint32_t fix_bank  = (bank_mask ? bank_mask - 1u : 0u) * 0x4000u;  // $4000-$7FFF
  const uint32_t pkbase = (uint32_t)pokey_base;
  const uint32_t pkmask = (uint32_t)pokey_mask;
  const uint32_t ymon   = (uint32_t)ym_enabled;

  while (1) {
    raw  = gpio_get_all();
    addr = raw & BUS_PIN_MASK;
    BANKSET_RUN_STEP(raw, addr);
    if (addr >= 0x4000u) {
      const uint32_t maria = BANKSET_MO(raw, lowrun, half);
      if (addr >= 0xC000u) {
        sio_hw->gpio_out = (uint32_t)rom_table[maria + last_bank + (addr & 0x3FFFu)] << D0_PIN;
        if (raw & RW_PIN_MASK) {
          if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
      } else if (addr >= 0x8000u) {
        sio_hw->gpio_out = (uint32_t)rom_table[maria + bank + (addr & 0x3FFFu)] << D0_PIN;
        if (raw & RW_PIN_MASK) {
          if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else {
          // Bankswitching write. End-of-cycle capture, bounded at 64 turns -
          // the reasoning is in emulate_supercart_ram(); a 6502 does not drive
          // the data lines until the second half of the cycle.
          if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS_PIN_MASK) != addr) break;
            last = cur;
          }
          bank = (((last >> D0_PIN) & 0x0Fu) & bank_mask) * 0x4000u;
        }
      } else {
        sio_hw->gpio_out = (uint32_t)rom_table[maria + fix_bank + (addr & 0x3FFFu)] << D0_PIN;
        if (raw & RW_PIN_MASK) {
          if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
      }
    } else {
      if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
      if ((addr & pkmask) == pkbase && ymon) {
        ym_window_service_blocking(addr);
      } else if ((addr & pkmask) == pkbase && !BANKSET_IS_READ()
                 && BANKSET_SALLY_OWNS(raw)) {
        // CAPTURE ONLY ON A WRITE. The listen-only rule this is copied from was
        // proven on the 16-byte $4000 and 32-byte $0450 windows; a POKEY at
        // $0800 occupies 2 KB, so an unconditional capture turns any read - or
        // any transient address that lands in that range - into a bogus
        // register write that persists until something overwrites it.
        //
        // This is the OPPOSITE trade from gating the DATA OUTPUT on R/W, which
        // patches/PicoA10400_0.15.txt forbids: losing a real write to one bad
        // sample costs a register update the player repeats next frame, while
        // accepting a non-write corrupts the chip's state.
        uint32_t last = gpio_get_all(), cur;
        for (uint32_t g = 0; g < 64; g++) {
          cur = gpio_get_all();
          if ((cur & BUS_PIN_MASK) != addr) break;
          last = cur;
        }
        BANKSET_CAPTURE(addr & 0x0F, (uint8_t)((last >> D0_PIN) & 0xFF));
      }
    }
  }
}

// SuperGame Bankset with banked RAM - MAME a78_bankset_sg_bankram_device
// (bankset.cpp:158-290). emulate_bankset_sg() with the $4000-$7FFF window
// turned into the same two 16K RAM banks emulate_bankset_ram() uses, and the
// $C000-$FFFF window taking Sally's writes into Maria's RAM.
// This is the shape Bubble Bobble, Attack of the Petscii Robots and the
// 2x128K RAM test carts want.
__attribute__((optimize("O2")))
void __time_critical_func(emulate_bankset_sg_ram()) {
  __asm volatile ("cpsid i" ::: "memory");
  uint32_t raw, addr, bank = 0;
  uint8_t rom_in_use = 0;
  BANKSET_RUN_DECL;
  const uint32_t half      = (uint32_t)romLen >> 1;
  BANKSET_BANK_MASK_DECL(bank_mask, romLen);
  const uint32_t last_bank = bank_mask * 0x4000u;
  const uint32_t pkbase = (uint32_t)pokey_base;
  const uint32_t pkmask = (uint32_t)pokey_mask;
  const uint32_t ymon   = (uint32_t)ym_enabled;

  while (1) {
    raw  = gpio_get_all();
    addr = raw & BUS_PIN_MASK;
    BANKSET_RUN_STEP(raw, addr);
    if (addr >= 0x4000u) {
      if (addr < 0x8000u) {
        // Drive first, R/W for direction only - see emulate_bankset_ram().
        sio_hw->gpio_out = (uint32_t)ram_table[BANKSET_MO(raw, lowrun, 0x4000u)
                                               + (addr & 0x3FFFu)] << D0_PIN;
        if (raw & RW_PIN_MASK) {
          if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else {
          if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS_PIN_MASK) != addr) break;
            last = cur;
          }
          ram_table[addr & 0x3FFFu] = (uint8_t)((last >> D0_PIN) & 0xFF);
        }
      } else if (addr < 0xC000u) {
        sio_hw->gpio_out = (uint32_t)rom_table[BANKSET_MO(raw, lowrun, half)
                                               + bank + (addr & 0x3FFFu)] << D0_PIN;
        if (raw & RW_PIN_MASK) {
          if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else {
          if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS_PIN_MASK) != addr) break;
            last = cur;
          }
          bank = (((last >> D0_PIN) & 0x0Fu) & bank_mask) * 0x4000u;
        }
      } else {
        // Drive first, R/W for direction only - see emulate_bankset_ram().
        sio_hw->gpio_out = (uint32_t)rom_table[BANKSET_MO(raw, lowrun, half)
                                               + last_bank + (addr & 0x3FFFu)] << D0_PIN;
        if (raw & RW_PIN_MASK) {
          if (!rom_in_use) { SET_DATA_MODE_OUT; rom_in_use = 1; }
        } else {
          if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
          uint32_t last = gpio_get_all(), cur;
          for (uint32_t g = 0; g < 64; g++) {
            cur = gpio_get_all();
            if ((cur & BUS_PIN_MASK) != addr) break;
            last = cur;
          }
          ram_table[0x4000u + (addr & 0x3FFFu)] = (uint8_t)((last >> D0_PIN) & 0xFF);
        }
      }
    } else {
      if (rom_in_use) { SET_DATA_MODE_IN; rom_in_use = 0; }
      if ((addr & pkmask) == pkbase && ymon) {
        ym_window_service_blocking(addr);
      } else if ((addr & pkmask) == pkbase && !BANKSET_IS_READ()
                 && BANKSET_SALLY_OWNS(raw)) {
        // CAPTURE ONLY ON A WRITE. The listen-only rule this is copied from was
        // proven on the 16-byte $4000 and 32-byte $0450 windows; a POKEY at
        // $0800 occupies 2 KB, so an unconditional capture turns any read - or
        // any transient address that lands in that range - into a bogus
        // register write that persists until something overwrites it.
        //
        // This is the OPPOSITE trade from gating the DATA OUTPUT on R/W, which
        // patches/PicoA10400_0.15.txt forbids: losing a real write to one bad
        // sample costs a register update the player repeats next frame, while
        // accepting a non-write corrupts the chip's state.
        uint32_t last = gpio_get_all(), cur;
        for (uint32_t g = 0; g < 64; g++) {
          cur = gpio_get_all();
          if ((cur & BUS_PIN_MASK) != addr) break;
          last = cur;
        }
        BANKSET_CAPTURE(addr & 0x0F, (uint8_t)((last >> D0_PIN) & 0xFF));
      }
    }
  }
}

// Called from setup1() before the cart is started, next to setup_cv().
static void setup_bankset(void) {
  // Both 16K RAM banks start cleared. rom_table is deliberately never cleared
  // between loads, and ram_table is not either - a Bankset cart that reads its
  // RAM before writing it would otherwise see the previous game's data, which
  // is not what a real board with a fresh SRAM chip does.
  for (uint32_t i = 0; i < 0x8000u; i++) ram_table[i] = 0;
#if BANKSET_HAS_HALT
  // See the HALT notes at the top of this block: the RP2040 pad default is a
  // pull-DOWN, which on a board that does not route HALT would read as
  // "Maria owns the bus" forever.
  gpio_pull_up(HALT_PIN);
#endif
}

// PP / WD: the only dump of "Pursuit of the Pink Panther" in circulation is
// 8195 bytes, and it is a KNOWN BAD DUMP - its 1K segments 2 and 3 are the
// wrong way round. Stella repairs it in CartridgeWD's constructor rather than
// carrying a second code path, and so do we; the three trailing bytes are not
// part of the image and are simply left unread.
//
// Not cosmetic, and not assumed: with the segments left as dumped, the same
// 200000-instruction run that is clean after the swap comes back with illegal
// opcodes and roughly half the WSYNCs (tools/pp_sim/sweep_pp.py, the second
// table). Swapped in place a byte at a time, so it needs no scratch buffer.
//
// Guarded on the length because a hand-renamed .WD file of any other size is
// by definition not this dump and must not be rearranged.
void setup_pp() {
  // Clear the 64 bytes of cartridge RAM. Stella does the same in
  // CartridgeEnhanced::reset() (initializeRAM); ram_table is shared with the
  // SuperChip loops and is never cleared between loads, so without this the
  // game starts on top of whatever the previous title left there - which is
  // neither what the emulator does nor anything a board reproduces.
  for (int i = 0; i < 64; i++) ram_table[i] = 0;
  if (romLen != 8192 + 3) return;
  for (int i = 0; i < 1024; i++) {
    uint8_t t = rom_table[2048 + i];
    rom_table[2048 + i] = rom_table[3072 + i];
    rom_table[3072 + i] = t;
  }
}

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
    // Drive the eight data lines harder than the RP2040 default. BUS_DRIVE_STRENGTH
    // is 8mA and was MEASURED on hardware (pokey.h has the full record): the 4mA
    // default is marginal for this bus - the title that streams data hardest
    // (LZSS Player) would not boot at all, and at 2mA three more titles came
    // back with distorted sound. This does NOT fix the smear band, which is
    // unchanged at every drive level. Slew rate is already SLOW by default - set
    // explicitly so the pad configuration is not split between code and defaults.
    for (uint gp = D0_PIN; gp < D0_PIN + 8; gp++) {
      gpio_set_drive_strength(gp, BUS_DRIVE_STRENGTH);
      gpio_set_slew_rate(gp, GPIO_SLEW_RATE_SLOW);
    }

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
  
  // Same shape as PicoA10400 (CLAUDE.md: keep the two in step). This board's
  // gate is >=33 so it never had the NORMALA78 hole the other one did, but the
  // structure is kept identical so a future edit cannot reintroduce it here.
  // Note the raised clock buys nothing audible on THIS board unless the GP25
  // bodge described in pokey.h has been fitted - the cartridge audio line is
  // not routed here, so ym_run() synthesises into the LED pin. If heat ever
  // matters here, this is the one branch to drop.
  // Which dispatch a cart takes used to be a pure numeric test, because every
  // 2600 type happened to sit below the 7800 block. CART_TYPE_UA/UASW broke
  // that: they are 2600 types numbered after it, and without this they would
  // take the 7800 clock and never get BUS_H driven low - which the 2600 loops
  // rely on, since they compare the full 16-bit port against 13-bit addresses.
  const bool is2600 = (cart_to_emulate <= 32)
                      || (cart_to_emulate == CART_TYPE_UA)
                      || (cart_to_emulate == CART_TYPE_UASW)
                      || (cart_to_emulate == CART_TYPE_FC);

  if (ym_enabled) {
   vreg_set_voltage(VREG_VOLTAGE_1_25);
   int ret=set_sys_clock_khz(YM_CLOCK_KHZ, true);
  }
  // 0.33: a POKEY cart gets the raised clock, same as a YM2151 cart. Confirmed
  // on hardware on the OTHER board (experiment E2, 2026-08-27): the flickering
  // lines on Bentley Bear / Donkey Kong PK-XM / Commando were present at
  // 250MHz and gone at 300MHz. Unlike PicoA10400, this board's >=33 gate below
  // already hands every POKEY cart the raised voltage, so here the change is
  // the CLOCK alone - which is in fact the half that carried the result there.
  //
  // After the ym_enabled test on purpose: identify_cartridge() sets
  // pokey_enabled for YM carts too, so testing it first would capture those.
  else if (pokey_enabled) {
   vreg_set_voltage(VREG_VOLTAGE_1_25);
   int ret=set_sys_clock_khz(POKEY_CLOCK_KHZ, true);
  }
  else if (!is2600 && cart_to_emulate>=33) {
   vreg_set_voltage(VREG_VOLTAGE_1_25);
   int ret=set_sys_clock_khz(EMU_CLOCK_KHZ, true);
  }
  if (is2600) {
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
  // E5 DIAGNOSTIC BUILD (pokey.h): route every cart to its PLAIN bus loop, so
  // core 1 behaves like a cart with no POKEY while core 0 keeps synthesising -
  // the mirror image of E4. Music stops (pokey_regs[] never receives a write);
  // judge picture only. See PicoA10400.ino for which cart shapes this is safe
  // on and which it is not.
#if POKEY_DIAG_E5
#define POKEY_BUS_ON 0
#else
#define POKEY_BUS_ON pokey_enabled
#endif

  if (cart_to_emulate == CART_TYPE_AR) setup_supercharger();
  if (cart_to_emulate == CART_TYPE_CV) setup_cv();
  if (cart_to_emulate == CART_TYPE_PP) setup_pp();
  if (cart_to_emulate >= CART_TYPE_BANKSET && cart_to_emulate <= CART_TYPE_BANKSET_SG_RAM)
    setup_bankset();

  reboot_cartridge(addr,addr_prev);
  
  //exit_cartridge(addr,addr_prev);
 
  switch (cart_to_emulate) {

     case CART_TYPE_ACTIVISION:
      emulate_activision();
        break;

     case CART_TYPE_SUPERCART_RAM: 
      if (POKEY_BUS_ON) emulate_supercart_ram_pokey(); else emulate_supercart_ram();
        break;
      
    case CART_TYPE_SUPERCART:
      // Plain SuperGame carts serve file bank 6 at $4000-$7FFF, i.e. exactly the
      // emulate_supercart_ef mapping (confirmed against MAME's a78 SG device, see
      // patches/Pico2A10400_0.10.txt). This used to fall through into
      // emulate_supercart_ram() first - a dead call, since that function never
      // returns, so the EF path below was unreachable and every plain-SG cart
      // silently got RAM at $4000 instead of the ROM data it expects there.
     case CART_TYPE_SUPERCART_EF:
      // Continually check address lines and put associated data on bus.
      if (POKEY_BUS_ON) emulate_supercart_ef_pokey();
      else               emulate_supercart_ef();
        break;
    
    case CART_TYPE_BANKSET:
      emulate_bankset_flat();
    break;

    case CART_TYPE_BANKSET_RAM:
#if BANKSET_DIAG_NORMALLOOP
      // CROSS-TEST, not a fix. "Black Lamp Music Demo (800)" - a NON-Bankset
      // cart with a POKEY at $0800 - plays through emulate_normala78_pokey(),
      // so that window works in THAT loop. The banked-RAM Bankset cart has
      // never produced music in ANY shape of emulate_bankset_ram(). This routes
      // it through the working loop instead, with romLen halved so Sally's half
      // is mapped flat.
      //
      // The picture will be wrong - MARIA gets Sally's half, whose $E000
      // character generator is all zeroes (measured), and the $4000-$7FFF RAM
      // window disappears. Judge the SOUND only.
      //
      //   music -> the fault is in emulate_bankset_ram(), and the reference
      //            loop handles this very cart's POKEY traffic fine
      //   silence -> the fault is not the loop; something about this cart's
      //            POKEY traffic differs from Black Lamp's
      romLen = (int)((uint32_t)romLen >> 1);
      emulate_normala78_pokey();
#else
      emulate_bankset_ram();
#endif
    break;

    case CART_TYPE_BANKSET_SG:
      emulate_bankset_sg();
    break;

    case CART_TYPE_BANKSET_SG_RAM:
      emulate_bankset_sg_ram();
    break;

    case CART_TYPE_NORMALA78:
      if (POKEY_BUS_ON) emulate_normala78_pokey();
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
      if (POKEY_BUS_ON) emulate_versa_pokey();
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

    // CommaVid (CV): 2K ROM + 1K RAM. isProbablyCV() has been detecting these
    // since forever, but there was no case here, so every CommaVid file was a
    // dead cartridge. setup_cv() has already built the 4K window inside
    // rom_table before the reboot; see there for the memory map and for where
    // each half of it was verified.
    //
    // ONE compare on the hot path. Both reads - RAM at $1000-$13FF and ROM at
    // $1800-$1FFF - come out of the same array at the same mask, so only the
    // write port needs testing for. That is one test fewer than the reference
    // loop in PlusCart-Pico, which nests "ROM or RAM" inside "read or write".
    //
    // A READ of the write port is served as a write here (garbage sampled off
    // the bus lands in RAM), which is what PlusCart-Pico does too; Stella
    // returns the RAM byte and MAME returns ROM. Three references, three
    // answers - so no game can depend on it. It is also what the real board
    // does: the 2600 cartridge port carries no R/W line, so a CommaVid board
    // has to derive write-enable from the address alone and cannot tell a read
    // of $1400-$17FF from a write to it either.
    case CART_TYPE_CV:
      data=0; data_prev=0;
      while (1) {
        while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
          addr_prev = addr;
        // got a stable address
        if (addr & 0x1000) {                  // A12 high
          // A11:A10 pick the quadrant of the 4K window: 0 = $1000-$13FF RAM
          // read, 1 = $1400-$17FF RAM write, 2 and 3 = ROM. Only quadrant 1 is
          // a write, and the other three are all reads of the same array at
          // the same mask - so one test decides everything.
          //
          // Written as a double shift, not as (addr & 0x0C00) == 0x0400, and
          // with the read side as the fall-through, both for reasons read out
          // of the .elf rather than assumed. This switch is register-starved:
          // the mask form compiled to "mov r6,ip / ldr r7,[sp,#8] / ands /
          // cmp" - the $0400 it compares against was reloaded from the stack
          // on every single bus cycle - and the read side sat behind an extra
          // unconditional branch. The shift form needs no constant at all
          // (lsls #20, lsrs #30, cmp #1) and this ordering puts the common
          // path first. Same 210-cycle-per-bus-cycle budget either way; this
          // just stops spending it for nothing.
          if (((addr << 20) >> 30) != 1) {    // RAM read port, or ROM
            gpio_put_masked(DATA_PIN_MASK,rom_table[addr&0xFFF]<<D0_PIN);
            SET_DATA_MODE_OUT;
            // wait for address bus to change
            while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
            SET_DATA_MODE_IN;
          } else {                            // $1400-$17FF: RAM write port
            // read last data on the bus before the address lines change
            while ((gpio_get_all()&BUS_PIN_MASK) == addr)
            { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
            rom_table[addr&0x3FF] = data_prev;
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
    // ------------------------------------------------------------------
    // The linear-bank family: EF / EFSC, DF / DFSC, BF / BFSC.
    //
    // ONE loop, not six. These are the same board with a wider bank
    // register: a contiguous run of hotspots at the top of the ROM window
    // selects one of N 4K banks, and the SC variants add the ordinary
    // 128-byte SuperChip at $1000-$10FF (write low, read high).
    //
    //   EF   $1FE0-$1FEF   16 banks    64K   Stella CartEF.cxx
    //   DF   $1FC0-$1FDF   32 banks   128K   CartDF.cxx
    //   BF   $1F80-$1FBF   64 banks   256K   CartBF.cxx
    //
    // The windows do not overlap, so nothing is lost by sharing the body,
    // and sharing it is what keeps the cost in SRAM rather than in speed:
    // this whole switch lives in .data, i.e. it is COPIED INTO SRAM at boot
    // (setup1() is __time_critical_func), so every duplicated bus loop is
    // permanently spent RAM on a board that has 29KB of it left over.
    //
    // Merging costs nothing on a Cortex-M0+, which is the part that makes
    // this free rather than a trade: neither $1FE0 nor a 16-bit mask fits an
    // 8-bit immediate, so even a constant-folded loop had to materialise
    // them in registers first. Runtime values land in exactly the same
    // instructions.
    //
    // Two shapes chosen for the hot path, both of which apply to EVERY read
    // this loop serves:
    //
    //   * the window test is `(addr - bs_base) < bs_banks`, one SUB and one
    //     CMP against a small immediate - and the SUB result IS the bank
    //     index, so the old code's second `addr - lowBS` disappears.
    //   * isSC is folded into a mask instead of being its own branch. An SC
    //     cart gets sc_mask = $0F00, so `!(addr & sc_mask)` is true exactly
    //     in $1000-$10FF (A12 is already known set here). A plain cart gets
    //     sc_mask = $1000, which in this branch can never be zero, so the
    //     test is always false and the SuperChip path is unreachable without
    //     a single instruction spent testing a flag. TST+BNE either way.
    //
    // BF/BFSC ARE LISTED BUT CANNOT BE FED A REAL IMAGE ON THIS HARDWARE.
    // A BF cart is 256KB and rom_table is 144KB; measured from the link map,
    // an RP2040 build ends .bss at 0x20038B30 of 0x20040000, i.e. 29392
    // bytes free for heap and stack together, so the missing 112KB does not
    // exist to be found. identify_cartridge() never even reaches its 256K
    // branch for such a file - the read is truncated to sizeof(rom_table)
    // first, so image_size is 147456 by then and the entry is already drawn
    // red as oversized. The cases are here because they cost two lines in
    // the parameter switch and nothing in the loop, because renaming a file
    // to .BF/.BFS reaches them and now yields the first 36 banks instead of
    // a dead cartridge, and because the day rom_table can hold 256KB they
    // are already correct.
    case CART_TYPE_EF:
    case CART_TYPE_EFSC:
    case CART_TYPE_DF:
    case CART_TYPE_DFSC:
    case CART_TYPE_BF:
    case CART_TYPE_BFSC: {
      uint32_t bs_base, bs_banks, sc_mask;
      switch (cart_to_emulate) {
        case CART_TYPE_DF:   bs_base = 0x1FC0; bs_banks = 32; sc_mask = 0x1000; break;
        case CART_TYPE_DFSC: bs_base = 0x1FC0; bs_banks = 32; sc_mask = 0x0F00; break;
        case CART_TYPE_BF:   bs_base = 0x1F80; bs_banks = 64; sc_mask = 0x1000; break;
        case CART_TYPE_BFSC: bs_base = 0x1F80; bs_banks = 64; sc_mask = 0x0F00; break;
        case CART_TYPE_EFSC: bs_base = 0x1FE0; bs_banks = 16; sc_mask = 0x0F00; break;
        default:             bs_base = 0x1FE0; bs_banks = 16; sc_mask = 0x1000; break;
      }
      // Clamp to what was actually loaded. A file truncated by
      // identify_cartridge() has fewer banks than its scheme implies, and
      // without this a hotspot for a bank that was never read would point
      // bankPtr past rom_table - which on this chip is the TinyUSB
      // descriptors, not spare memory. Ignoring the switch leaves the
      // previous bank live, which is merely wrong rather than unbounded.
      {
        uint32_t have = (uint32_t)romLen / 4096;
        if (bs_banks > have) bs_banks = have ? have : 1;
      }
      data=0;data_prev=0;
      bankPtr = &rom_table[0];
      while (1) {
        while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
          addr_prev = addr;
        if (addr & 0x1000)   { // A12 high
          uint32_t bsel = addr - bs_base;
          if (bsel < bs_banks)                          // bank-switch
            bankPtr = &rom_table[bsel << 12];
          if (!(addr & sc_mask))
          { // SuperChip window, $1000-$10FF
            if (addr & 0x0080)
            { // $1080-$10FF: a read from cartridge ram
              gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0x7F]<<D0_PIN);
              SET_DATA_MODE_OUT;
              // wait for address bus to change
              while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
              SET_DATA_MODE_IN;
            } else {  // $1000-$107F: a write to cartridge ram
              // read last data on the bus before the address lines change
              while ((gpio_get_all()&BUS_PIN_MASK) == addr)
              { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
              ram_table[addr&0x7F] = data_prev;
            }
          } else {        // normal rom access
            gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);
            SET_DATA_MODE_OUT;
            // wait for address bus to change
            while ((gpio_get_all()&BUS_PIN_MASK)== addr) ;
            SET_DATA_MODE_IN;
          }
        }
      }
    }
      break;
    // ------------------------------------------------------------------
    // 0840 "EconoBanking". 8K, two banks, and - like UA and 3F - hotspots
    // BELOW $1000, in the address space the TIA and the RIOT live in.
    //
    // DECODE, from Stella Cart0840.cxx checkSwitchBank():
    //     (addr & $1840) == $0800  ->  bank 0
    //     (addr & $1840) == $0840  ->  bank 1
    // i.e. A12 clear, A11 set, and A6 IS the bank number. This branch
    // already knows A12 is 0, so the test reduces to A11 and the bank comes
    // straight out of bit 6 - no compare against the second hotspot at all.
    //
    // Nothing in ordinary console traffic reaches it: on a 2600 the TIA is
    // selected with A12=0 and A7=0, RAM and the RIOT with A12=0 and A7=1,
    // and all of that decodes below $0800. $0800-$0FFF is the unused mirror
    // region, which is exactly why the board was built to use it. Anything
    // that could fool this decode would equally fool the real cartridge.
    //
    // Three matching address samples, not one, and a re-read before
    // latching - both for the reason the UA loop gives: in the A12-high path
    // a half-settled address costs one wasted ROM byte, but here it would
    // latch the wrong bank, and a wrong bank is a crash.
    case CART_TYPE_0840: {
      cartPages = romLen / 4096;
      {
      unsigned char *half0 = &rom_table[0];
      unsigned char *half1 = &rom_table[(cartPages > 1) ? 4096 : 0];
      bankPtr = half0;
      addr = 0; addr_prev = 0; addr_prev2 = 0;
      while (1) {
        while (((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev) || (addr != addr_prev2))
        {
          addr_prev2 = addr_prev;
          addr_prev = addr;
        }
        // got a stable address
        if (addr & 0x1000) { // A12 high - normal ROM access
          gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);
          SET_DATA_MODE_OUT;
          // wait for address bus to change
          while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
          SET_DATA_MODE_IN;
        } else if (addr & 0x0800) {   // A12 low, A11 high - the hotspots
          // Confirm before latching, as in the UA loop: the 6507 holds an
          // address for a whole bus cycle, a transition glitch does not.
          if ((gpio_get_all() & 0x1840) == (addr & 0x1840))
            bankPtr = (addr & 0x0040) ? half1 : half0;
        }
      }
      }
    }
      break;
    // ------------------------------------------------------------------
    // PP / WD - the Wickstead Design board, one game: "Pursuit of the Pink
    // Panther" (prototype). Stella CartWD.cxx. Unlike everything else here
    // the window is not one bank but FOUR 1K SLOTS, filled from eight 1K
    // segments by a fixed table, and the board has 64 bytes of RAM.
    //
    //   $1000-$103F  RAM read port     (CartEnhanced myReadOffset  = 0)
    //   $1040-$107F  RAM write port    (myWriteOffset = myRamSize, because
    //                                   CartWD sets RAM_HIGH_WP - the
    //                                   OPPOSITE way round from a SuperChip)
    //   $1080-$1FFF  ROM, four 1K slots; the first 128 bytes of slot 0 are
    //                covered by the RAM and simply unreachable (myRomOffset)
    //
    // Hotspots are reads of $0030-$003F, below $1000 again, and the bank
    // number is the low four bits taken mod 8.
    //
    // THE SWITCH IS DELAYED, AND THE DELAY IS NOT OPTIONAL. Stella only
    // *initiates* the switch at the hotspot and lands it once more than
    // three CPU cycles have passed. tools/pp_sim/ settles what that means
    // here, by running the real ROM: with the switch applied immediately the
    // run dies on 669 illegal opcodes and 20 WSYNCs, with it delayed by four
    // bus cycles it executes 200000 instructions with ZERO illegal opcodes
    // and 6878 WSYNCs. Sweeping the delay from 0 to 8 (sweep_pp.py), FOUR IS
    // THE ONLY VALUE THAT RUNS.
    //
    // And the ROM says why, without reference to any simulator arithmetic
    // (trace_hotspot.py). The dominant idiom at five of the eight hotspot
    // sites is:
    //
    //     $D677  LDA $3B      3 cycles, the hotspot read is the last of them
    //     $D679  JMP $3200    3 cycles: opcode, operand lo, operand hi
    //     ...    the jump TARGET is fetched on the 4th cycle
    //
    // $D679 and $3200 are in different slots, so the JMP's own three bytes
    // must still come from the OLD mapping and only the target fetch from
    // the new. Switching at once fetches the JMP opcode itself out of the
    // wrong segment, which is precisely the 669 illegal opcodes above.
    //
    // Our loop sees every bus cycle the 6507 runs, so "four bus cycles" and
    // "four CPU cycles" are the same statement here.
    case CART_TYPE_PP: {
      // Stella CartWD.hxx ourBankOrg: which 1K segment sits in each of the
      // four slots, for each of the eight configurations. Banks 8-15 alias
      // onto 0-7 (bank % romBankCount()), which the & 7 below does.
      const uint8_t pp_org[8][4] = {
        { 0, 0, 1, 3 }, { 0, 1, 2, 3 }, { 4, 5, 6, 7 }, { 7, 4, 2, 3 },
        { 0, 0, 6, 7 }, { 0, 1, 7, 6 }, { 2, 3, 4, 5 }, { 6, 0, 5, 1 }
      };
      // Four live slot pointers, rebuilt only when the bank actually
      // changes, so the read path is one indexed load and no table walk.
      unsigned char *pp_slot[4];
      uint32_t pp_bank = 0, pp_pending = 0, pp_delay = 0;
      // Address handled on the previous pass. The delay below is counted in
      // BUS CYCLES, and only an A12-high pass ends in a wait for the address
      // to change - an A12-low pass re-samples at once and would otherwise
      // spin the counter down inside a single 838ns cycle.
      uint32_t pp_last = 0xFFFFFFFF;
      for (int s = 0; s < 4; s++) pp_slot[s] = &rom_table[pp_org[0][s] << 10];
      addr = 0; addr_prev = 0; addr_prev2 = 0;
      data = 0; data_prev = 0;
      while (1) {
        while (((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev) || (addr != addr_prev2))
        {
          addr_prev2 = addr_prev;
          addr_prev = addr;
        }
        // One tick of the switch delay per BUS CYCLE, which is what a changed
        // address means here. A pending switch lands BEFORE this cycle is
        // served, so the access that completes the delay already sees the new
        // mapping - that is the whole point of the scheme (the JMP that
        // triggered the switch still reads its own three bytes from the old
        // bank, and only its target comes from the new one).
        if (addr != pp_last) {
          pp_last = addr;
          if (pp_delay && --pp_delay == 0 && pp_pending != pp_bank) {
            pp_bank = pp_pending;
            for (int s = 0; s < 4; s++) pp_slot[s] = &rom_table[pp_org[pp_bank][s] << 10];
          }
          if ((addr & 0x1FF0) == 0x0030) { // hotspot read, $0030-$003F
            // Confirm before arming, as in the UA loop: the 6507 holds an
            // address for a whole bus cycle, a transition glitch does not.
            if ((gpio_get_all() & 0x1FF0) == 0x0030) {
              pp_pending = addr & 7;
              pp_delay = 4;
            }
          }
        }
        if (addr & 0x1000) { // A12 high - RAM ports, then ROM
          if (!(addr & 0x0F80))
          { // $1000-$107F: the 64 bytes of cartridge RAM, seen twice.
            // Bit 6 picks the port, and it is the MIRROR IMAGE of a SuperChip:
            // CartWD sets RAM_HIGH_WP, so the WRITE port is the high half and
            // the read port the low one.
            //
            // This test used to be `(addr & 0x0F80) == 0x0000` for the read
            // port with a separate `(addr & 0x0FC0) == 0x0040` for the write
            // port - but 0x0F80 does not contain bit 6, so the first test was
            // already true across the whole window and the write branch was
            // unreachable. Every write to cartridge RAM was answered as a
            // read, i.e. the firmware drove the data lines while the 6507 was
            // driving them, and nothing was ever stored. On hardware that is
            // a game whose title screen is fine and whose sprites vanish the
            // moment it needs a variable.
            if (addr & 0x0040)
            { // $1040-$107F: write port
              // read last data on the bus before the address lines change
              while ((gpio_get_all()&BUS_PIN_MASK) == addr)
              { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
              ram_table[addr&0x3F] = data_prev;
            } else {
              // $1000-$103F: read port
              gpio_put_masked(DATA_PIN_MASK,ram_table[addr&0x3F]<<D0_PIN);
              SET_DATA_MODE_OUT;
              while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
              SET_DATA_MODE_IN;
            }
          } else {
            gpio_put_masked(DATA_PIN_MASK,pp_slot[(addr>>10)&3][addr&0x3FF]<<D0_PIN);
            SET_DATA_MODE_OUT;
            while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
            SET_DATA_MODE_IN;
          }
        }
      }
    }
      break;
    case CART_TYPE_UA:
    case CART_TYPE_UASW: {
      // UA Ltd. 8K. The one thing that makes this different from every other
      // 2600 mapper here: the hotspots are BELOW $1000, i.e. at A12 = 0, in
      // the address space the TIA and the RIOT live in. Same shape as
      // CART_TYPE_3F, which also has to watch A12 = 0 - except that 3F reads
      // the bank number off the DATA bus, while UA takes it from the address
      // alone, so no data sampling is needed here at all.
      //
      // DECODE, from Stella CartUA.cxx checkSwitchBank(): the board compares
      // only A12, A9, A6 and A5.
      //
      //     (addr & $1260) == $0220  ->  bank 0   (low 4K)
      //     (addr & $1260) == $0240  ->  bank 1   (high 4K)
      //
      // A11, A10 and A8 are not connected to the decode at all, and A7 is
      // left out because the Brazilian (Digivision) boards drive it: their
      // $02A0/$02C0 pair is bit-for-bit the same decode as $0220/$0240.
      // This is not a guess - "Fathon (Brasil) (Digivision)" boots through
      // `STA $02A0 / JMP $300C` in its high bank, which only reaches live
      // code if $02A0 selects bank 0, and $02A0 & $0260 == $0220. The same
      // masking is what makes `BIT $FB0` (Jumper, Digivision Beamrider) and
      // the $FA0/$FC0 pair of the 0FA0 scheme land on the right banks.
      //
      // Masking with $0260 rather than $1260 below is safe and one bit
      // cheaper: this branch already knows A12 is 0. Note that it also
      // ignores A13-A15, which on a 2600 the firmware drives low itself
      // (gpio_set_dir_out_masked(BUS_H_PIN_MASK) in the is2600 dispatch) -
      // so unlike 3F's exact `addr == 0x003F` compare, this one does not
      // depend on that.
      //
      // Nothing here can false-trigger on ordinary console traffic: zero page
      // and the stack have A9 = 0, and every RIOT I/O register ($280-$297)
      // has A6 = A5 = 0, so they all mask to $0200. And whatever would fool
      // this decode would equally fool the real cartridge.
      //
      // UASW swaps which hotspot picks which half of the file. It is never
      // auto-detected (Stella carries it as an md5 override for two Digivision
      // dumps); reach it by renaming the file to *.UAS.
      cartPages = romLen / 4096;
      {
      unsigned char *half0 = &rom_table[0];
      unsigned char *half1 = &rom_table[(cartPages > 1) ? 4096 : 0];
      unsigned char *lowBank  = (cart_to_emulate == CART_TYPE_UASW) ? half1 : half0;
      unsigned char *highBank = (cart_to_emulate == CART_TYPE_UASW) ? half0 : half1;
      // Power-on bank is the FIRST half of the file in both variants - what
      // Stella's startBank() gives. UASW swaps the hotspots, not the image,
      // and "Mickey (Digivision)" depends on it: neither half's RESET path
      // touches a hotspot, so whichever half is live at power-on is the one
      // that runs.
      bankPtr = half0;
      addr = 0; addr_prev = 0; addr_prev2 = 0;
      while (1) {
        while (((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev) || (addr != addr_prev2))
        { // three matching samples, as in CART_TYPE_3F: a half-settled address
          // is only a wasted ROM byte in the A12-high path, but in the A12-low
          // path it would latch the wrong bank.
          addr_prev2 = addr_prev;
          addr_prev = addr;
        }
        // got a stable address
        if (addr & 0x1000) { // A12 high - normal ROM access
          gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);
          SET_DATA_MODE_OUT;
          // wait for address bus to change
          while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
          SET_DATA_MODE_IN;
        } else {           // A12 low - the hotspots live here
          uint32_t hs = addr & 0x0260;
          if (hs == 0x0220 || hs == 0x0240) {
            // Confirm before latching. The 6507 holds an address for a whole
            // bus cycle (~838ns), a transition glitch does not - and a wrong
            // bank here is a crash. Same guard patch 0.12 needed for the
            // Activision bank register.
            if ((gpio_get_all() & 0x1260) == hs)
              bankPtr = (hs == 0x0220) ? lowBank : highBank;
          }
        }
      }
      }
      }
      break;
    // FC - Amiga "Power Play Arcade Video Game Album" (Stella CartFC.cxx).
    // Three hotspots, and the only 2600 mapper here that takes its bank
    // number off the DATA bus inside the ROM window:
    //
    //   write $1FF8   bank bits 0-1        <- data bus
    //   write $1FF9   bank bits 2 and up   <- data bus
    //   access $1FFC  commit the latched bank, and serve THAT read from the
    //                 bank just selected
    //
    // The commit-on-$1FFC is what makes the scheme work at all, because $1FFC
    // is also the RESET vector: `JMP $BFFB` fetches the 4C at $1FFB from the
    // OLD bank and the two vector bytes at $1FFC/$1FFD from the NEW one, so a
    // game reaches its own entry point without any RAM stub. Every image in
    // the library uses that: the album menu's rows, the $1FE0 return stubs in
    // banks 1 and 2, and the $1008 trampoline (STX $FFF8 / STY $FFF9 /
    // LDA $FFFC / JMP ($0080)) that 3-D Havoc and album banks 4-7 share.
    // Order below matches: switch FIRST, then read bankPtr.
    //
    // $1FF8/$1FF9 are the one place this firmware deliberately stops driving
    // the bus inside $1000-$1FFF - it has to, because the value it needs is
    // the one the 6502 is writing, and the sampling idiom is the same one the
    // SuperChip write port uses. That costs a read of those two addresses,
    // which would come back as the floating bus instead of ROM. Verified,
    // not assumed: across all seven images no instruction reads $1FF8 or
    // $1FF9 (only STA/STX/STY reach them), no code or vector lives at those
    // two offsets in any bank, and a full run of each image under
    // tools/fc_sim/run_fc.py counts zero reads of either.
    //
    // A write to $1FFC commits too. The cart has no R/W line, so this is not
    // a choice; Stella happens to switch on reads only, because its
    // CartridgeFC::poke() masks the address to 12 bits before comparing it
    // with $1FFC. The difference is invisible on every path any library image
    // actually executes - measured in the simulator, image by image - so the
    // physically forced behaviour costs nothing here.
    case CART_TYPE_FC: {
      uint32_t fcBanks    = romLen / 4096;      // 1, 2, 4 or 8
      uint32_t fcBankMask = fcBanks - 1;
      uint32_t fcTarget   = 0;                  // Stella's reset(): bank 0
      data = 0; data_prev = 0;
      addr = 0; addr_prev = 0;
      bankPtr = &rom_table[0];
      while (1) {
        while ((addr = (gpio_get_all()&BUS_PIN_MASK)) != addr_prev)
          addr_prev = addr;
        // got a stable address
        if (addr & 0x1000) {              // A12 high
          if (addr >= 0x1FF8) {           // 8 of 4096 addresses; one compare,
                                          // one fewer than the F8 loop needs
            if (addr <= 0x1FF9) {
              // read the last data on the bus before the address lines change
              while ((gpio_get_all()&BUS_PIN_MASK) == addr)
              { data_prev = data; data = (gpio_get_all()&DATA_PIN_MASK)>>D0_PIN; }
              // Bank composition verbatim from Stella CartridgeFC::poke(),
              // with its modulo written as a mask - the bank count is always
              // a power of two. The mask is also what keeps a truncated dump
              // inside its own image: a game that writes $1FF8 alone can ask
              // for bank 3 of a 2-bank file, and unmasked that would serve
              // whatever the previous cartridge left in rom_table.
              if (addr == 0x1FF8)
                fcTarget = data_prev & 0x03;
              else if ((uint32_t)(data_prev << 2) < fcBanks)
                fcTarget = (fcTarget + (data_prev << 2)) & fcBankMask;
              else                        // same value written to both hotspots
                fcTarget = data_prev & fcBankMask;
              continue;
            }
            if (addr == 0x1FFC)           // commit, before the byte is served
              bankPtr = &rom_table[(fcTarget & fcBankMask) * 4096];
          }
          // normal rom access
          gpio_put_masked(DATA_PIN_MASK,bankPtr[addr&0xFFF]<<D0_PIN);
          SET_DATA_MODE_OUT;
          // wait for address bus to change
          while ((gpio_get_all()&BUS_PIN_MASK) == addr) ;
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
      if (POKEY_BUS_ON) emulate_supercart_large_pokey(); else emulate_supercart_large();
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

// CommaVid (CV): 2K ROM + 1K RAM on one 4K window, no bankswitching.
//
//   $1000-$13FF  RAM read port    (1K)
//   $1400-$17FF  RAM write port   (the same 1K, offset by $400)
//   $1800-$1FFF  ROM              (2K)
//
// TODO.md had the two ports the other way round. Every reference in ORIG/
// agrees it is read low / write high, and they were checked against each other
// rather than trusted one at a time:
//   * Stella - CartCV.hxx class comment, and CartEnhanced.cxx:71 where
//     myRamWpHigh = true yields myWriteOffset = $400, myReadOffset = 0;
//   * MAME - a26_rom_cv_device in vcs/rom.cpp: read_rom() answers from RAM only
//     for offset < $400, write_bank() accepts writes only in $400-$7FF;
//   * all four flashcart firmwares - PlusCart-Pico
//     (cartridge_emulation.cpp:1199), UnoCart-2600, the DirtyHairy fork and
//     United-Carts-of-Atari carry the same "$F000-$F3FF 1K RAM read,
//     $F400-$F7FF 1K RAM write" comment over the same loop.
//
// The 4K window is laid out inside rom_table itself, so the hot loop needs no
// second base pointer and no second bounds test: one compare picks out the
// write port and everything else is a single indexed read of
// rom_table[addr & 0xFFF].
//
// A 4K CV FILE ALREADY HAS EXACTLY THAT LAYOUT. Stella's CartridgeCV
// constructor reads the first 1K of a 4K image as the initial RAM contents and
// the last 2K as the ROM - which is byte-for-byte the window this loop wants.
// Verified against the library rather than assumed: for 8 of the 9 distinct 4K
// images here, the last 2K hashes equal to a 2K CV ROM we already have
// (MagiCard or Video Life). So for a 4K file this function does nothing at all.
//
// STELLA IS THE ONLY REFERENCE THAT DOES THIS. All four flashcart firmwares in
// ORIG/ run isProbablyCV() for 2K images ONLY (PlusCart-Pico main.cpp:789 is
// the one our own detection was copied from), and their loader puts cart_ram
// AFTER the image, uninitialised - so none of them can load a MagiCard save.
// That is where our gap came from, and it is why the 4K branch below is worth
// having rather than being a curiosity.
//
// A 2K file is the ROM alone: move it up into the $1800 half and clear the RAM
// half. Anything shorter is tiled across the 2K, as CartridgeCV does for
// size < 2K. That is unreachable from auto-detection (which only assigns CV to
// a 2048- or 4096-byte image) but reachable by renaming a file to *.CV.
//
// Must run BEFORE reboot_cartridge(), for the same reason setup_supercharger()
// must: reboot feeds the 6502 a JMP ($FFFC), so the CPU fetches its reset
// vector from $1FFC/$1FFD as soon as that retires, and for a 2K image the byte
// it needs is not at rom_table[0xFFC] until the move below has happened.
static void setup_cv(void) {
  if (romLen >= 4096) return;    // 4K image: already in window layout

  int n = (romLen > 0 && romLen < 2048) ? romLen : 2048;
  memmove(&rom_table[0x800], &rom_table[0], n);
  for (int i = n; i < 2048; i += n)
    memcpy(&rom_table[0x800 + i], &rom_table[0x800],
           (i + n <= 2048) ? n : (2048 - i));
  memset(&rom_table[0], 0, 0x800);
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

int isProbablyUA(int size, unsigned char *bytes)
{	// UA Ltd. bankswitching puts its hotspots BELOW $1000: $0220 selects the
	// low 4K bank, $0240 the high one. The Brazilian (Digivision) boards use
	// the shifted pair $02A0/$02C0, which is the same decode - see
	// emulate_ua_cartridge() for why A7 does not matter.
	// Signature list from Stella CartDetector.cxx isProbablyUA(), MINUS its
	// seventh entry { 0x2C, 0xB0, 0x0F } (BIT $FB0). That one exists for the
	// Digivision Beamrider dump, which Stella cannot run as plain UA anyway -
	// its DefProps.hxx entry forces UASW, so the signature's own autodetect
	// answer is wrong for its own target. In this library it matches nothing
	// but "Jumper 8k 0.28" (4 files), and Jumper is a REAL F8 cart: simulated
	// on the 6502, it bank-switches 143 times by running through $1FF8/$1FF9
	// and keeps a kernel going, while under UA it hits BRK immediately.
	// Keeping the signature would therefore trade zero gains for a regression.
	unsigned char signature[6][3] = {
			{ 0x8D, 0x40, 0x02 },  // STA $240 (Funky Fish, Pleiades)
			{ 0xAD, 0x40, 0x02 },  // LDA $240 (Hobo)
			{ 0xBD, 0x1F, 0x02 },  // LDA $21F,X (Gingerbread Man, Grandma's Revenge)
			{ 0x2C, 0xC0, 0x02 },  // BIT $2C0 (Time Pilot)
			{ 0x8D, 0xC0, 0x02 },  // STA $2C0 (Fathom, Vanguard, Galaxian)
			{ 0xAD, 0xC0, 0x02 }   // LDA $2C0 (Mickey, Zaxxon, Super Soccer)
		};
	for (int i = 0; i < 6; ++i)
		if(searchForBytes(bytes, size, signature[i], 3, 1))
			return 1;
	return 0;
}

int usesF8Hotspots(int size, unsigned char *bytes)
{	// Absolute access to the F8 hotspots $1FF8/$1FF9 (or their $FFF8/$FFF9
	// mirror). NOT a Stella function - it exists to keep one file out of the
	// UA branch.
	//
	// "Super Soccer (Digivision)" (md5 0e7e7334..., 8 copies in the library)
	// carries a dead `LDA $02C0 / JMP $F400` stub at $1FE0 and therefore
	// matches isProbablyUA(). It is an F8 cart: its RESET vector ($1FFC =
	// $FFEC) lands on `LDA $FFF8 / JMP $F000`, i.e. select bank 0 and jump
	// into it, and the image holds four absolute F8-hotspot accesses.
	// Stella reaches the same verdict, but only through an explicit md5
	// override in DefProps.hxx ("F8") - autodetection alone calls it UA.
	// We have no md5 at load time, so the override is expressed as this test.
	//
	// Measured over every 8192-byte file in the library (12148 of them):
	// of the 126 files isProbablyUA() accepts, Super Soccer's 8 are the ONLY
	// ones with an F8 hotspot access. The 18 genuine UA images have none.
	// Applied inside the UA branch only, so it can move a file back to F8 and
	// can never move one away from it.
	unsigned char op[6] = { 0xAD, 0x8D, 0x2C, 0x0C, 0xBD, 0x9D };  // LDA/STA/BIT/NOP/LDA,X/STA,X
	unsigned char sig[3];
	for (int o = 0; o < 6; ++o)
		for (int lo = 0xF8; lo <= 0xF9; ++lo)
			for (int hi = 0; hi < 2; ++hi) {
				sig[0] = op[o]; sig[1] = (unsigned char)lo;
				sig[2] = hi ? 0xFF : 0x1F;
				if (searchForBytes(bytes, size, sig, 3, 1))
					return 1;
			}
	return 0;
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
 
	// "DFDF", not "DFBF". Stella CartDetector.cxx isProbablyDF(): 'DF carts
	// store strings "DFDF" and "DFSC" starting at address $FFF8'. The B was a
	// typo carried from the BF function just above, and it made this test
	// unsatisfiable - no file can carry a marker no tool ever writes. Measured
	// over the library: 4 files (2 distinct images, DF_128k_test.bin and
	// 128kMultikernelFramework.bin) hold "DFDF" at size-8 and were reaching
	// identify_cartridge()'s 128K branch only to fall out of it with
	// cart_type still CART_TYPE_NONE. That is why TODO.md position 7 counted
	// DFSC but no DF at all.
	return !memcmp(tail + 8, "DFDF", 4);
}

int isProbablyDFSC(unsigned char *tail)
{
	return !memcmp(tail + 8, "DFSC", 4);
}

// FC - Amiga "Power Play Arcade Video Game Album" and the Amiga prototypes
// built on the same board. Signatures verbatim from Stella CartDetector.cxx
// isProbablyFC(); each one is the bank-select sequence itself, which is the
// only thing on an FC cart that no other scheme writes:
//
//   STA $1FF8 / LSR / LSR / STA $1FF9   the album menus and 3-D Ghost Attack
//   STA $FFF8 / STA $FFFC               Surf's Up, S.A.C. Alert (4K)
//   STY $FFF9 / LDA $FFFC               3-D Havoc
//
// Measured over the whole library rather than assumed: exactly seven distinct
// images match, in four size buckets - 4 x 4K, 3-D Havoc (8K), and the album
// as both a 16K partial dump and a complete 32K one. No file of any other
// scheme matches any of the three, so this test cannot steal a working game.
int isProbablyFC(int size, unsigned char *bytes)
{
	unsigned char signature[3][6] = {
		{ 0x8D, 0xF8, 0x1F, 0x4A, 0x4A, 0x8D },  // STA $1FF8, LSR, LSR, STA ...
		{ 0x8D, 0xF8, 0xFF, 0x8D, 0xFC, 0xFF },  // STA $FFF8, STA $FFFC
		{ 0x8C, 0xF9, 0xFF, 0xAD, 0xFC, 0xFF }   // STY $FFF9, LDA $FFFC
	};
	for (int i = 0; i < 3; i++)
		if (searchForBytes(bytes, size, signature[i], 6, 1))
			return 1;
	return 0;
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
        
        // POKEY placement, from the 16-bit header field (byte53 high, byte54 low -
        // the order MAME, ProSystem and test7800 all agree on):
        //   bit0  -> $4000    bit6  -> $0450    bit10 -> $0440    bit15 -> $0800
        // Taken BEFORE the mask on the next line, which clears bit0.
        {
          uint16_t head_lo = A78_HEADER[54];
          // MAME's validate_header() disables POKEY@$4000 when the same header also
          // claims $4000 for RAM / bank 0 / bank 6 / banked RAM - they cannot share
          // the window and the game needs its data there, not a sound chip.
          uint8_t conflict = head_lo & 0x3d;
          if (conflict == 0x05 || conflict == 0x09 ||
              conflict == 0x11 || conflict == 0x21) {
            head_lo &= (uint16_t)~0x01;
          }
          pokey_mask = 0xFFF0;
          if      (head_lo & 0x0001)     { pokey_enabled = 1; pokey_base = 0x4000; }
          else if (head_lo & 0x0040)     { pokey_enabled = 1; pokey_base = 0x0450; }
          else if (head_lo & 0x0400)     { pokey_enabled = 1; pokey_base = 0x0440; }
          else if (A78_HEADER[53] & 0x80){ pokey_enabled = 1; pokey_base = 0x0800;
                                           // 32 BYTES, NOT 2KB. MAME installs
                                           // its handler over the whole
                                           // $0800-$0FFF slot decode
                                           // (a7800.cpp:1511), but JS7800 -
                                           // the implementation that actually
                                           // plays these carts - watches only
                                           // $0800-$081F (Cartridge.js:
                                           // "address >= 0x0800 && address <
                                           // 0x0820"), and that is what this
                                           // firmware needs.
                                           //
                                           // The difference matters because our
                                           // capture is LISTEN-ONLY off a live
                                           // bus, not a decoded chip select. A
                                           // 2KB window is 64x more addresses
                                           // for a floating or stale bus to land
                                           // in, and every stray hit costs twice:
                                           // it writes a bogus value into
                                           // pokey_regs[] AND spends up to 1us in
                                           // the end-of-cycle capture, during
                                           // which the loop is not watching for
                                           // the real write.
                                           //
                                           // Measured on "Bankset Test 2x32K RAM
                                           // Pokey800": thousands of captures per
                                           // second (_BSAU4096 armed), yet not one
                                           // carrying a non-zero AUDCx byte
                                           // (_BSA5 silent) - the signature of a
                                           // window catching noise instead of
                                           // writes. Every address this cart
                                           // really writes ($0800-$080F, $0815,
                                           // $0818+Y) is inside the 32-byte
                                           // window, so nothing is lost.
                                           pokey_mask = 0xFFE0; }  // $0800-$081F
          else                           { pokey_enabled = 0; pokey_base = 0xFFFF; }
          for (int i=0;i<16;i++) pokey_regs[i]=0;
          // SKCTL defaults to "released" (running), not the real chip's power-on
          // 0x00 (held in reset): 53/434 library files declaring POKEY never
          // write SKCTL at all, and a literal power-on-silent chip would leave
          // every one of them mute forever. Every file that DOES write SKCTL
          // still gets its own value the instant it writes it - see pokey.h.
          pokey_regs[0x0F] = 0x03;
        }

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
        //
        // THIS BOARD HAS NO AUDIO LINE - see the note at the top of ym2151.h. The
        // bus side still matters: without it a YM cart hangs in its detection spin
        // instead of running silently.
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
        } else if (A78_HEADER[53] & 0x20) {
          // ---- Bankset -------------------------------------------------
          // MAME a78_slot.cpp:409-470 selects the board with
          // "switch (mapper & 0xe02e)", and every case with bit 13 set is a
          // Bankset board. The bits inside that mask are: 0x2000 banksets,
          // 0x4000 halt-banked RAM, 0x8000 POKEY@$800, 0x0002 SuperGame,
          // and 0x0004/0x0008/0x0020 the other $4000-window options.
          //
          // Placed AFTER the Activision/Absolute tests and BEFORE the
          // "map53 == 0" chain because that chain is gated on byte53 having
          // no mapper bits left, and bit 5 is exactly such a bit - which is
          // why all of these files land on CART_TYPE_NORMALA78 today, i.e.
          // with Maria's half handed to the CPU.
          const uint16_t bs_mapper = ((uint16_t)A78_HEADER[53] << 8)
                                   | (uint16_t)A78_HEADER[54];
          const uint16_t bs_sel    = bs_mapper & 0xE02E;
          const uint32_t bs_pay    = (uint32_t)image_size;      // header-declared
          const uint32_t bs_half   = bs_pay >> 1;
          const bool bs_supergame  = (bs_sel & 0x0002) != 0;
          const bool bs_bankram    = (A78_HEADER[53] & 0x40) != 0;
          // Only these seven values are Bankset boards in MAME. 0xA000
          // (banksets + POKEY@$800, flat, no banked RAM) is deliberately NOT
          // here: MAME has no case for it either, so such a file keeps
          // falling through to the flat path exactly as it does today.
          const bool bs_known = (bs_sel == 0x2000) || (bs_sel == 0x6000)
                             || (bs_sel == 0xE000) || (bs_sel == 0x2002)
                             || (bs_sel == 0x6002) || (bs_sel == 0xA002)
                             || (bs_sel == 0xE002);
          // "Fits" means every byte the header claims is actually sitting in
          // rom_table. bytes_read is what the loader managed to store, after
          // its own truncation to sizeof(rom_table) - so this one test covers
          // both an oversized image and a header that lies about its length.
          const bool bs_fits = bs_known && (bs_pay >= 0x2000)
                            && ((bs_pay & 1) == 0)
                            && (bs_pay <= (uint32_t)bytes_read);

          if (bs_fits) {
            if (bs_supergame) cart_type = bs_bankram ? CART_TYPE_BANKSET_SG_RAM
                                                     : CART_TYPE_BANKSET_SG;
            else              cart_type = bs_bankram ? CART_TYPE_BANKSET_RAM
                                                     : CART_TYPE_BANKSET;
          } else if (bs_known && bs_half <= (uint32_t)bytes_read) {
            // DEGRADED MODE - the whole 2x image does not fit in rom_table but
            // SALLY'S HALF DOES, and it is the first thing in the file, so it
            // is complete. Serve just that half as an ordinary cart of the
            // same shape: the game boots and runs real code, and only what
            // Maria fetches is wrong. This is what every 2x128K Bankset title
            // gets on an RP2040 - 256KB of image cannot coexist with the
            // ~77KB of other globals in 264KB of SRAM, so it is a hard
            // hardware limit, not a tuning choice.
            Serial.println("Bankset image too large - serving Sally's half only");
            romLen = (int)bs_half;
            if (bs_supergame) cart_type = bs_bankram ? CART_TYPE_SUPERCART_RAM
                                                     : CART_TYPE_SUPERCART;
            else              cart_type = CART_TYPE_NORMALA78;
          } else {
            cart_type = CART_TYPE_NORMALA78;
          }
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
    // Stella runs isProbablyCV() in the 4K bucket as well, ahead of 4KSC
    // (CartDetector.cxx:51), because a 4K CommaVid file is "1K saved RAM image
    // + 2K ROM" - the format MagiCard's own save produces. We only ran it for
    // 2K, so 50 files in the library (every MagiCard sample program and picture,
    // and both Video Life dumps that carry saved RAM) came out as a plain 4K
    // cart: the ROM half was served correctly at $1800-$1FFF, but $1000-$17FF
    // answered with a mirror of the ROM instead of with RAM.
    //
    // BUT ONLY THE ROM HALF IS SEARCHED, WHICH IS WHERE THIS DIFFERS FROM
    // STELLA - and the difference was forced by a hardware test, not by taste.
    // 0.36 searched the whole 4K image, as Stella does, and that misclassified
    // "Image Patricia with Horse" (SnailSoft, 2 copies in the library): the
    // picture came out noisy and kept degrading on the real console. It is not
    // a CommaVid cart at all. It is a plain 4K one whose PICTURE DATA happens
    // to contain the bytes 9D FF F3 at offset $0D3 - inside the half a real CV
    // file would use for saved RAM, i.e. for data, never for code.
    //
    // Simulated read pattern, which is what identifies it (tools/cv_sim/):
    // every genuine CommaVid title reads ONLY $1000-$13FF and writes ONLY
    // $1400-$17FF, the split port doing exactly what it is for. Image Patricia
    // reads BOTH halves in equal measure (16542 / 16698 accesses) and writes
    // neither - the signature of a cart that simply has 4K of ROM. Served as
    // CV, its reads of $1400-$17FF decode as writes here, so the firmware
    // stops driving the bus AND scribbles the sampled floating value into the
    // RAM the picture lives in. That is the noise that showed up on the TV.
    //
    // The rule is not a special case for one file: the signature is an
    // INSTRUCTION THE ROM EXECUTES, and in this file format the ROM is the
    // last 2K. Bytes 0-2047 are a RAM image - data - so searching them can
    // only produce false positives.
    //
    // Scope, counted over the library rather than estimated: of the 37721
    // files of exactly 4096 bytes, 50 match anywhere and 48 match in the ROM
    // half. The two that drop out are the two copies of Image Patricia. All 8
    // distinct genuine images keep their match (MagiCard's at $E71, Video
    // Life's at $9A3). The 2K bucket is untouched - there the whole image IS
    // the ROM.
  	if (isProbablyCV(2048, &rom_table[2048]))
  		cart_type = CART_TYPE_CV;
  	else
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
		// Stella runs isProbablyUA() exactly here: after 3F, before FE
		// (CartDetector.cxx, the 8K bucket). usesF8Hotspots() is our stand-in
		// for the md5 override Stella needs for Super Soccer - see that
		// function for the measurement.
		else if (isProbablyUA(bytes_read, rom_table)
		         && !usesF8Hotspots(bytes_read, rom_table))
			cart_type = CART_TYPE_UA;
    else if (isProbablyFE(bytes_read, rom_table) && !f8)
			cart_type = CART_TYPE_FE;
		else if (isProbably0840(bytes_read, rom_table))
			cart_type = CART_TYPE_0840;
		// Stella runs isProbablyFC() here too, last before the F8 fallback.
		// One image in the library reaches it: 3-D Havoc, 24 files, which as
		// F8 boots and then can never leave bank 1 - its $1008 trampoline
		// touches $1FF9 on every call, and under F8 that IS the bank select.
		else if (isProbablyFC(bytes_read, rom_table))
			cart_type = CART_TYPE_FC;
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
		// Stella's position for FC in this bucket: after E7, before 3E.
		else if (isProbablyFC(bytes_read, rom_table))
			cart_type = CART_TYPE_FC;
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
		// The COMPLETE album is a 32K, 8-bank image; the 16K file above is a
		// partial dump of it whose menu can only reach two of its own games.
		// Three files in the library, all classified F4 until now.
		else if (isProbablyFC(bytes_read, rom_table))
			cart_type = CART_TYPE_FC;
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
    // Never hang here. Two cases reach this line: a listing gone stale because files
    // were changed over USB while the menu was up, and the MENU_LONGNAME_TEXT
    // placeholder, which matches no file on purpose. Returning leaves newgame==0, so
    // the caller rebuilds the listing and the selection simply does nothing.
    Serial.println("open error");
    return;
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
            // getName() returns 0 when the name does not fit, and SdFat's failure path
            // (FatLib/FatName.cpp) then zeroes byte 0 ONLY - not the byte it reached -
            // leaving '\0' followed by real characters 1..n. That single byte breaks
            // sorting, marquee and loading at once, and does it silently. The entry
            // cannot be opened from the listing either way, so show a placeholder and
            // flag it rather than a row that lies about which file it is.
            bool namefits = file.getName(filename, MAX_NAME_LEN);
            if (!namefits) {
              memset(filename,32,sizeof(filename));
              memcpy(filename,MENU_LONGNAME_TEXT,12);
              filename[12]=0;
            }
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
            direntry_toobig[contfile] = (!namefits || (!isdir && payload > sizeof(rom_table))) ? 1 : 0;
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
#if !POKEY_DIAG_E4
     if (pokey_enabled) pokey_run();   // never returns
#endif
     // E4 DIAGNOSTIC BUILD (pokey.h): with pokey_run() skipped, core 0 spins
     // here for the rest of the game - exactly what it does for a cart with no
     // POKEY. Bus side untouched, so "core 0 is synthesising" is the single
     // variable removed. No sound.
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

