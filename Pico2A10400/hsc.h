#ifndef HSC_H
#define HSC_H

// Atari 7800 High Score Cart - TODO position 11, on the nvstore primitive.
//
// Must be included AFTER nvstore.h (it uses the NV_SLOT_HSC slot) and after the
// SdFat globals (an optional /HSC.ROM on the drive overrides the built-in BIOS).
//
// ---------------------------------------------------------------------------
// What the hardware is
// ---------------------------------------------------------------------------
// A pass-through cartridge: it plugs between the console and the game and adds
// two windows to the address space. MAME and JS7800 agree, and the addresses
// were read out of MAME rather than copied from a wiki
// (src/mame/machine/a7800.cpp:1480-1481):
//
//   $1000-$17FF   2 KB battery-backed SRAM   read_10xx / write_10xx
//   $3000-$3FFF   4 KB HSC BIOS ROM          read_30xx
//
// Both windows sit BELOW $4000, i.e. in the part of the map a 7800 cartridge
// never drives, so they cannot collide with the game's own ROM window.
//
// The game does not write the table itself: it calls the BIOS through a jump
// table at the end of the ROM. Read straight out of the dump rather than
// guessed, and it matches what TODO.md recorded:
//
//   $3FEB  4C A9 39   JMP $39A9      $3FF4  4C 27 31   JMP $3127
//   $3FEE  4C 00 30   JMP $3000      $3FF7  4C 22 30   JMP $3022
//   $3FF1  4C 11 30   JMP $3011      $3FFA  4C DA 30   JMP $30DA
//                                    $3FFD  4C 2B 31   JMP $312B
//
// That is why the BIOS is required and not optional: without it the game's
// JSR lands in open bus.
//
// ---------------------------------------------------------------------------
// The BIOS is built in; /HSC.ROM overrides it
// ---------------------------------------------------------------------------
// Since 0.53 the dump lives in hsc_bios.h and is part of the firmware, for the
// same reason supercharger_bios.h always has been: it is a fixed part of the
// device being emulated, not user content. A game with an HSC header does not
// write its table itself - it calls into this ROM - so a missing file is not a
// degraded feature, it is no feature, and on a console there is no way to show
// a message saying so. 0.52 loaded it from the drive because TODO.md said to
// follow JS7800's shape; JS7800 is a browser emulator with nowhere to put it,
// which is a constraint this firmware does not have. 4096 bytes is 0.47% of the
// free sketch space on either board, and none of it is RAM: the array is const,
// so it sits in flash, and hsc_prepare() copies it into SRAM exactly the way
// setup_rom() copies supercharger_bios_bin.
//
// /HSC.ROM in the root of the drive still wins if it is there, so an
// alternative or patched dump can be tried without rebuilding. It is accepted
// on SIZE, not on checksum - a checksum gate would mean the override could only
// ever be the file it replaces, which is no override at all - with two things
// reported on the serial port:
//
//   CRC32 0x9BE408D3   the known-good dump; MD5 c8a73288ab97226c52602204ab894286,
//                      byte-for-byte what JS7800 requires (Cartridge.js:919),
//                      and the number in the archived dump's own filename
//   CRC32 0x561364E8   REJECTED BY NAME, and the built-in used instead
//   anything else      used, with a warning naming the CRC
//
// 0x561364E8 is singled out rather than caught by "not the good one" because
// the reason is not that a checksum disagrees - that would be circular - but
// that its instruction stream does not decode as 6502: "LDA #$EF / LDA #$83"
// (a dead store over the previous one) where the good dump has
// "LDA #$EF / JSR $3483", followed by an undocumented opcode. 541 of its 4096
// bytes differ, spread over all 16 pages, while the jump table at the end is
// identical - a damaged copy of the same ROM, not another revision. It is the
// one file a user is likely to find and copy over by mistake.
//
// CRC32 is computed bitwise, without a table, and only for an override: 32768
// iterations once per game start on core 0, against 1 KB of RAM for a lookup
// table. The built-in array is not re-checked at runtime - it is verified when
// hsc_bios.h is generated (tools/make_hsc_bios_h.py refuses to write a header
// whose source does not match both the MD5 and the CRC32) and its length is
// checked at compile time.
//
// ---------------------------------------------------------------------------
// Neither window costs a byte of new RAM
// ---------------------------------------------------------------------------
// The 2 KB SRAM lives in ram_table[0..2047] and the 4 KB BIOS in the tail of
// rom_table. Both are free for exactly the carts this feature accepts:
//
//   * ram_table is the 2600 cart-RAM / SuperChip array and the 7800 SuperGame
//     RAM window. A FLAT 7800 cart (CART_TYPE_NORMALA78) has no cart RAM at
//     all - emulate_normala78() never touches ram_table - so its first 2 KB are
//     unused for the whole game.
//   * rom_table is 144 KB and a flat cart is mapped to the TOP of the address
//     space, so the loop only ever reads rom_table[0..romLen-1]. The BIOS goes
//     at HSC_BIOS_OFF = 144 KB - 4 KB, and hsc_prepare() REFUSES if romLen
//     would reach it.
//
// This matters on the RP2040, where the build already reports "Low memory
// available": a dedicated 6 KB would have been a fifth of what is left.
//
// ---------------------------------------------------------------------------
// Which carts this version accepts, and why not all of them
// ---------------------------------------------------------------------------
// Measured over the whole library with tools/hsc_survey.py, not estimated. Of
// 2363 headers, 180 declare an HSC in byte 58 (0x01 or 0x03 - NOT a bare
// "byte58 & 1", which would also pull in the 8 files carrying 0x9D/0xFF
// garbage). Those 180 land on our cart types like this:
//
//   CART_TYPE_NORMALA78   152 files   134 of them with no POKEY and no YM
//   SuperGame 9-bank       16 files     9
//   SuperGame + RAM         9 files     0
//   SuperGame               3 files     3
//
// This version handles the first row and only when core 0 is free: 134 files,
// 74% of everything that declares an HSC, and the only ones where the save can
// actually be written - loop() hands core 0 to pokey_run() / ym_run() and
// NEITHER RETURNS, so a POKEY or YM cart has nowhere to run the flash write.
// Serving the windows there anyway would give a high-score table that silently
// never persists, which is worse than not offering it. TODO.md draws the same
// line ("najpierw 154 pliki bez audio ... to dobra granica wersji").
//
// NO REGION GATE, deliberately, and this is a departure from JS7800.
// Cartridge.js:898 refuses the HSC for any ROM whose header says PAL. Copying
// that would refuse three files in this library named, literally,
// "Joust (PAL) (HSC Fix)", "Dig Dug (PAL) (HSC Fix)" and
// "Xevious (PAL) (HSC Fix)" - PAL conversions made specifically to work WITH a
// High Score Cart. 13 of the 180 are PAL. The gate is JS7800 policy about a
// US-only product, not something the hardware enforces, and it would break the
// files most likely to be tried on the PAL console this project is tested on.
//
// ---------------------------------------------------------------------------
// When the table is written back
// ---------------------------------------------------------------------------
// Not on the BIOS operation code. TODO.md proposed triggering on the write to
// $1007, having read that $3483 opens with "STA $1007". Reading it again shows
// why that is the wrong edge: the opcode is stored at routine ENTRY and the
// score data goes in afterwards, so persisting there would save the state
// BEFORE the change.
//
// So: debounce. Core 1 bumps hsc_writes on every NVRAM write - one increment,
// no timer call, because time_us_32() on core 1 could be a call into flash and
// core 1 must never read flash (nvstore.h). Core 0 watches that counter and
// commits once it has been still for HSC_QUIET_US. The real cart writes SRAM
// under a battery and has no such window; ours does, and TODO.md already
// records that as the price of a design without guaranteed power backup.

#include "pico/time.h"
#include "hsc_bios.h"

#define HSC_BIOS_BYTES    4096u
#define HSC_NVRAM_BYTES   2048u
#define HSC_BIOS_CRC32    0x9BE408D3u
// The damaged dump in circulation - see the note above. Rejected by name.
#define HSC_BIOS_CRC32_BAD 0x561364E8u
#define HSC_BIOS_PATH     "/HSC.ROM"

static_assert(sizeof(hsc_bios_bin) == HSC_BIOS_BYTES,
              "hsc_bios.h is not 4096 bytes - regenerate it with tools/make_hsc_bios_h.py");
static_assert(sizeof(hsc_nvram_blank_bin) <= HSC_NVRAM_BYTES,
              "the blank SRAM image does not fit 2048 bytes");
// The BIOS is parked in the tail of rom_table - see the RAM note above.
#define HSC_BIOS_OFF      (144u * 1024u - HSC_BIOS_BYTES)

// How long the NVRAM window has to stay quiet before the table is committed.
// Long enough that a burst of BIOS writes becomes one erase, short enough that
// the console can be switched off a moment later without losing the score.
#ifndef HSC_QUIET_US
#define HSC_QUIET_US 250000u
#endif

// HSC_DIAG_NOSAVE: serve both windows, count the writes, and never touch flash.
// Splits "the two new bus windows broke the game or the picture" from "writing
// flash under a live bus broke it" - the same split FA2_DIAG_NOWRITE draws for
// FA2, and the same CLAUDE.md lesson (POKEY_DIAG_E12) behind it.
#ifndef HSC_DIAG_NOSAVE
#define HSC_DIAG_NOSAVE 0
#endif

// Read by core 1's dispatch in setup1(); written by core 0 before newgame.
volatile uint8_t  hsc_enabled = 0;
// What core 1 tells core 0 about the two windows. Deliberately counters and not
// timestamps: time_us_32() on core 1 may compile to a call into flash, and core
// 1 must never read flash. Compiled into PRODUCTION, not just the probe build,
// so a measurement describes the loop that actually ships.
volatile uint32_t hsc_writes = 0;      // writes captured in $1000-$17FF
volatile uint32_t hsc_reads = 0;       // reads served in  $1000-$17FF
volatile uint32_t hsc_bios_reads = 0;  // reads served in  $3000-$3FFF

// Can the table be written back? Separate from hsc_enabled ON PURPOSE. With no
// save slot the cartridge is still emulated and the game still works - it just
// does not remember. 0.56-0.58 refused to emulate at all in that case, which
// left $3000-$3FFF as open bus and FROZE the game the moment it called into the
// BIOS; reported from hardware 2026-09-08. Being unable to save is not a reason
// to break the game.
uint8_t hsc_persist = 0;
// Header byte 58 (the save-device field), captured by identify_cartridge().
uint8_t a78_save_dev = 0;
// Which BIOS the running game got: 0 = built in, 1 = /HSC.ROM from the drive.
uint8_t hsc_bios_source = 0;

// 0 = enabled. 1 = header byte 58 does not declare an HSC. 2 = not a flat 7800
// cart. 3 = the cart also has a POKEY or a YM, so core 0 never comes back.
// 4 = the ROM image would overlap the BIOS. 5 = no save slot resolved.
uint8_t hsc_reject = 0;

static uint32_t hsc_seen_writes = 0;
static uint32_t hsc_quiet_since = 0;
static uint8_t  hsc_pending = 0;
static uint16_t hsc_commits = 0;

// Bitwise CRC32 (the usual reflected polynomial), no table: this runs once per
// game start on core 0 and a 1 KB table would cost more than it saves.
static uint32_t hsc_crc32(const uint8_t *p, uint32_t n) {
  uint32_t c = 0xFFFFFFFFu;
  while (n--) {
    c ^= *p++;
    for (int k = 0; k < 8; k++)
      c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1)));
  }
  return ~c;
}

// A BLANK High Score Cart is NOT a zeroed one, and getting that wrong cost a
// hardware round. The BIOS reads $10B3,X and takes 0x7F to mean "slot free" -
// LDA $10B3,X / CMP #$7F / BEQ, straight out of the dump - so 2 KB of zeroes
// looks to it like 69 OCCUPIED slots belonging to game id 0/0. On hardware that
// showed as a score of 0, no previous entries, and a new entry that would not
// stick; reported on Xevious, Asteroids and Joust against 0.52/0.53. JS7800 has
// generateDefaultSram() for exactly this reason, and hsc_nvram_blank_bin is
// derived from it (tools/make_hsc_bios_h.py).
static void hsc_nvram_seed_blank(uint8_t *nv) {
  memset(nv, 0, HSC_NVRAM_BYTES);
  memcpy(nv, hsc_nvram_blank_bin, hsc_nvram_blank_bin_len);
}

// Was this table written by a firmware that did not know the above? Detected as
// "the whole index area is zero" rather than by checking the header signature:
// a table the BIOS has been using may legitimately have changed any byte of the
// header, but it can never have left the index zero - $10B3,X is 0x7F for a free
// slot and a game id for a used one. So this re-seeds a 0.52/0.53 save and
// nothing else.
static bool hsc_nvram_looks_unwritten(const uint8_t *nv) {
  for (uint32_t i = 0; i < 0x13D; i++)
    if (nv[i]) return false;
  return true;
}

// HSC_DIAG_PROBE - the build that answers "why is nothing happening" WITHOUT a
// serial port, because there is none while the cartridge is in an Atari.
//
// It writes the whole decision chain into /.save_hsc.data itself, so the answer
// comes back by plugging the cartridge into a PC and looking at the file:
//
//   once at game start   the record below, whether or not HSC was enabled
//   every 5 seconds      the same record refreshed, with the LIVE count of
//                        NVRAM writes core 1 has seen on the bus
//
// That separates the three things the reports so far cannot: "HSC was never
// enabled" (hsc_reject != 0), "enabled but the bus side sees no writes"
// (writes == 0), and "writes seen but the commit never runs" (writes > 0,
// commits == 0). It also proves by itself whether a flash write under a live
// bus works at all, since the first record is written while core 1 is serving
// the menu and the rest while it is serving the game.
//
// The record lives in the last 32 bytes of the 2 KB, which the BIOS would
// otherwise use for score data. That is deliberate and this build is therefore
// NOT for playing - it costs an erase every 5 seconds, capped at 200.
#ifndef HSC_DIAG_PROBE
#define HSC_DIAG_PROBE 0
#endif

#if HSC_DIAG_PROBE
// The probe's SECOND channel, and the one that does not depend on nvstore.
//
// The first version wrote its record INTO the save slot, which is circular: if
// the slot never resolved, the probe could not report that the slot never
// resolved - and that is exactly what came back from hardware, twice, as a file
// with no header in it at all.
//
// This writes a plain text file in the root instead, through the same call
// sequence ym2151.h uses for YM_LOG.TXT and for the same stated reason: it is
// "the only usable channel on real hardware, where the Pico is powered by the
// console and cannot also be on a USB cable". Opened against the VOLUME so it
// lands in the root rather than in whichever folder the menu is browsing, and
// flushed, because without syncBlocks() the write dies in Adafruit_SPIFlash's
// RAM cache.
//
// Timing is the same as YM_LOG.TXT's: at boot and at cart load, i.e. while core
// 1 is serving the MENU and before any emulate_* loop starts. Never during a
// game - writing a file parks core 1, and ym2151.h records what that cost when
// it was tried (audio dead from the start, no file written).
static void hsc_log(const char *line) {
  FatFile f;
  if (f.open(&fatfs, "HSC_DIAG.TXT", O_WRONLY | O_CREAT | O_APPEND)) {
    f.write(line);
    f.close();
    flash.syncBlocks();
    fatfs.cacheClear();
  }
}

// Called from loop() right after nv_init(): says whether the slots resolved at
// all, which is the question the in-slot probe could not ask.
static void hsc_log_slots(void) {
  char b[160];
  snprintf(b, sizeof(b),
           "boot pc=%d fs=%08lX..%08lX | fa2 be=%d f=%d off=%08lX | "
           "hsc be=%d f=%d off=%08lX\r\n",
           connected_to_pc ? 1 : 0,
           (unsigned long)nv_fs_lo(), (unsigned long)nv_fs_hi(),
           nv_target[NV_SLOT_FA2].backend, nv_fail[NV_SLOT_FA2],
           (unsigned long)nv_target[NV_SLOT_FA2].off,
           nv_target[NV_SLOT_HSC].backend, nv_fail[NV_SLOT_HSC],
           (unsigned long)nv_target[NV_SLOT_HSC].off);
  hsc_log(b);
}

static void hsc_log_load(int cart_type) {
  char b[160];
  snprintf(b, sizeof(b),
           "load b58=%02X cart=%d pk=%d ym=%d romLen=%ld en=%d rej=%d bios=%d "
           "persist=%d hscoff=%08lX\r\n",
           a78_save_dev, cart_type, pokey_enabled, ym_enabled, (long)romLen,
           hsc_enabled, hsc_reject, hsc_bios_source, hsc_persist,
           (unsigned long)nv_target[NV_SLOT_HSC].off);
  hsc_log(b);
}
#else
static inline void hsc_log_slots(void) {}
static inline void hsc_log_load(int) {}
#endif

#if HSC_DIAG_PROBE
#define HSC_PROBE_EVERY 5000000u
#define HSC_PROBE_MAX   200
static uint16_t hsc_probe_n = 0;
static uint32_t hsc_probe_last = 0;

// The counters go to HSC_DIAG.TXT, NOT into the table. 0.56-0.60 planted a
// 32-byte record at $17E0, and the 2 KB has no spare byte to plant it in: the
// BIOS fills $113D-$17F9 with 69 score records of 25 bytes each and keeps
// scratch at $17FA-$17FE, so the probe was overwriting the last record and the
// scratch it existed to observe. The file channel 0.57 added makes the in-table
// copy redundant, and the commit still runs, so a probe build now saves the
// table the BIOS actually wrote.
static void hsc_probe_write(void) {
  if (hsc_probe_n >= HSC_PROBE_MAX) return;
  char b[192];
  snprintf(b, sizeof(b),
           "probe n=%u b58=%02X cart=%d pk=%d ym=%d en=%d rej=%d hscbe=%d "
           "fa2be=%d bios=%d persist=%d bios_rd=%lu nv_rd=%lu nv_wr=%lu "
           "commits=%lu\r\n",
           (unsigned)hsc_probe_n, a78_save_dev, (int)cart_to_emulate,
           pokey_enabled, ym_enabled, hsc_enabled, hsc_reject,
           nv_target[NV_SLOT_HSC].backend, nv_target[NV_SLOT_FA2].backend,
           hsc_bios_source, hsc_persist,
           (unsigned long)hsc_bios_reads, (unsigned long)hsc_reads,
           (unsigned long)hsc_writes, (unsigned long)hsc_commits);
  hsc_log(b);
  hsc_probe_n++;
  hsc_probe_last = time_us_32();
  nv_write_slot(NV_SLOT_HSC, ram_table, HSC_NVRAM_BYTES);
}
#else
static inline void hsc_probe_write(void) {}
#endif

// Core 0, called from LoadGame() after identify_cartridge() and BEFORE newgame,
// so everything it touches is settled before core 1 enters the game loop.
// Reading the BIOS off the drive is pure FS reads, which are safe while core 1
// serves the menu; nothing here creates or writes a file.
static void hsc_prepare(int cart_type) {
  hsc_enabled = 0;
  hsc_writes = 0;
  hsc_reads = 0;
  hsc_bios_reads = 0;
  hsc_seen_writes = 0;
  hsc_pending = 0;
  // No slot is not a refusal any more - see hsc_persist.
  hsc_persist = nv_slot_ready(NV_SLOT_HSC) ? 1 : 0;

  // Why HSC was refused, recorded rather than dropped: on a console there is no
  // serial port to read and no way to show a message, so the probe build
  // (HSC_DIAG_PROBE) writes this code into the save file instead.
  hsc_reject = 0;
  // 0x01 = HSC, 0x03 = HSC + SaveKey. NOT "byte58 & 1": 0x9D and 0xFF also have
  // bit 0 set and are garbage in 8 headers (Pit Fighter proto, Tubes).
  if (a78_save_dev != 0x01 && a78_save_dev != 0x03)      hsc_reject = 1;
  // Flat 7800 carts only in this version - 152 of the 180 files. See above.
  else if (cart_type != CART_TYPE_NORMALA78)             hsc_reject = 2;
  // A POKEY or YM cart never gives core 0 back, so the save could never run.
  else if (pokey_enabled || ym_enabled)                  hsc_reject = 3;
  // The BIOS would land inside the game image.
  else if ((uint32_t)romLen > HSC_BIOS_OFF)              hsc_reject = 4;
  if (hsc_reject) {
    Serial.print("HSC: disabled, reason "); Serial.println(hsc_reject);
    hsc_log_load(cart_type);
    hsc_probe_write();
    return;
  }

  // The built-in BIOS is the default. Copied out of .rodata (flash) into SRAM,
  // exactly as setup_rom() copies supercharger_bios_bin: core 1 reads the copy
  // in rom_table and never the array, because core 1 must not touch flash.
  memcpy(&rom_table[HSC_BIOS_OFF], hsc_bios_bin, HSC_BIOS_BYTES);
  hsc_bios_source = 0;

  // /HSC.ROM overrides it when present. Accepted on size, not on checksum - see
  // the note at the top of this file - except for the one dump known to be
  // damaged, which is put back.
  File32 f = fatfs.open(HSC_BIOS_PATH, O_RDONLY);
  if (f) {
    const bool sized = (f.fileSize() == (uint32_t)HSC_BIOS_BYTES);
    const bool read_ok = sized &&
        (f.read(&rom_table[HSC_BIOS_OFF], HSC_BIOS_BYTES) == (int)HSC_BIOS_BYTES);
    f.close();
    if (!read_ok) {
      Serial.println("HSC: /HSC.ROM is not 4096 bytes - ignored, using the built-in BIOS");
      memcpy(&rom_table[HSC_BIOS_OFF], hsc_bios_bin, HSC_BIOS_BYTES);
    } else {
      const uint32_t crc = hsc_crc32(&rom_table[HSC_BIOS_OFF], HSC_BIOS_BYTES);
      if (crc == HSC_BIOS_CRC32_BAD) {
        Serial.println("HSC: /HSC.ROM is the damaged 0x561364E8 dump - ignored, "
                       "using the built-in BIOS");
        memcpy(&rom_table[HSC_BIOS_OFF], hsc_bios_bin, HSC_BIOS_BYTES);
      } else {
        hsc_bios_source = 1;
        if (crc == HSC_BIOS_CRC32) {
          Serial.println("HSC: using /HSC.ROM (matches the known-good dump)");
        } else {
          Serial.print("HSC: using /HSC.ROM, CRC32 0x"); Serial.print(crc, HEX);
          Serial.println(" - not a dump this firmware knows");
        }
      }
    }
  }

  // The table itself.
  if (!nv_read_slot(NV_SLOT_HSC, ram_table, HSC_NVRAM_BYTES) ||
      hsc_nvram_looks_unwritten(ram_table)) {
    hsc_nvram_seed_blank(ram_table);
    Serial.println("HSC: table seeded blank");
  }

  hsc_enabled = 1;
  Serial.print("HSC: enabled, BIOS ");
  Serial.print(hsc_bios_source ? "/HSC.ROM" : "built in");
  Serial.println(hsc_persist ? ", table persists"
                             : ", NO SAVE SLOT - table will not persist "
                               "(plug the cartridge into a PC once)");
  hsc_log_load(cart_type);
  hsc_probe_write();
}

// Core 0, from loop(), every pass. Returns after one load and one compare unless
// an HSC game has actually written something.
static void hsc_service(void) {
#if HSC_DIAG_PROBE
  // Refresh the probe record on a timer, regardless of what the bus is doing -
  // a count of zero is exactly the answer we are after, and the debounce would
  // never fire to record it.
  if ((uint32_t)(time_us_32() - hsc_probe_last) >= HSC_PROBE_EVERY) hsc_probe_write();
#endif
  if (!hsc_enabled) return;
  const uint32_t w = hsc_writes;
  if (w != hsc_seen_writes) {         // still writing - restart the timer
    hsc_seen_writes = w;
    hsc_quiet_since = time_us_32();
    hsc_pending = 1;
    return;
  }
  if (!hsc_pending) return;
  if ((uint32_t)(time_us_32() - hsc_quiet_since) < HSC_QUIET_US) return;
  hsc_pending = 0;
  if (!hsc_persist) return;   // emulated, just not remembered
#if !HSC_DIAG_NOSAVE
  // Programmed straight out of ram_table: 2048 bytes is four flash pages and
  // flash_range_program() has no alignment requirement on its source pointer -
  // checked in the SDK, where the asserts cover only the offset and the count.
  // Safe to read it this late because the window has been quiet for a quarter
  // of a second; if the game does write during the commit, the torn image is
  // replaced by the next quiet period.
  nv_write_slot(NV_SLOT_HSC, ram_table, HSC_NVRAM_BYTES);
#endif
  hsc_commits++;
}

#endif // HSC_H
