#ifndef NVSTORE_H
#define NVSTORE_H

// nvstore - non-volatile save slots, written WHILE core 1 is emulating the
// cartridge bus.
//
// Must be included AFTER the SdFat/Adafruit_SPIFlash globals (fatfs, flash),
// because the primary back-end is a file on the FAT volume.
//
// ---------------------------------------------------------------------------
// One file per emulated NVRAM device
// ---------------------------------------------------------------------------
//   /.save_fa2.data   the Harmony FA2 cartridge flash,  256 bytes  (TODO 10)
//   /.save_hsc.data   the 7800 High Score Cart SRAM,   2048 bytes  (TODO 11)
//
// Both are 8192 bytes, contiguous, and carry the HIDDEN attribute. They are
// visible from a PC over USB - so a table can be backed up, copied to another
// cartridge, or reset by deleting the file - and invisible on the Atari, twice
// over: the listing already skipped hidden entries, and 0.51 also skips any
// entry in the ROOT directory named ".<something>.data".
//
// One file per device rather than one shared file, because the devices are
// independent and sharing costs real RAM. If FA2 and HSC shared a 4 KB sector,
// saving a Star Castle score would first have to read the 2048-byte HSC table
// into RAM so the erase would not lose it - flash_range_program() takes its
// source from RAM, so "copy flash to flash" does not exist while XIP is off.
// Separate sectors mean FA2 stages 256 bytes and HSC, when it lands, can be
// programmed straight out of its own SRAM buffer. Checked in the SDK rather
// than assumed: flash_range_program() asserts page alignment on the OFFSET and
// the COUNT and says nothing about the source pointer, which the bootrom takes
// as a plain const uint8_t*.
//
// Separate files also mean an FA2 save can never destroy the HSC table - not
// through a bug, and not through power loss inside the ~45 ms erase.
//
// WHY 8192 BYTES FOR 256 BYTES OF DATA. Flash erases in 4096-byte sectors and
// in nothing smaller, so a save needs a whole sector that belongs to nobody
// else. Inside a contiguous run starting at b, the first 4096-aligned boundary
// is at most b+4095, so a run of 8192 bytes always contains one whole aligned
// sector while a run of 4096 guarantees none. 8192 is the smallest size that
// can be reasoned about rather than hoped for.
//
// ---------------------------------------------------------------------------
// Three things stand between a bug here and a destroyed ROM library
// ---------------------------------------------------------------------------
// This is the price of putting the save INSIDE the FAT volume instead of in the
// reserved sector above it, where a wrong address was impossible by
// construction. Stated plainly so it is not forgotten:
//
//  1. The offset comes from FatFile::contiguousRange() on a file opened by its
//     exact name. That walks the FAT chain and FAILS unless every cluster is
//     contiguous, so the run it returns is the file's own allocation according
//     to the filesystem itself - not to a cached flag, and not to our
//     arithmetic.
//  2. Bounds: the sector must be 4096-aligned, must lie wholly inside that run,
//     and must lie inside [_FS_start, _FS_end). Checked when the slot is
//     resolved and again immediately before the erase.
//  3. Identity: the first 32 bytes of the sector carry "A104", the layout
//     version, the slot id and THE SECTOR'S OWN OFFSET. They are read back from
//     XIP before every erase and must agree. A sector that identifies itself as
//     the other slot, or as living somewhere else, is never erased.
//
//     (3) cannot bite on the very first write to a newly created file, because
//     nothing has written the identity yet. There (1) and (2) carry it alone.
//     That is the honest limit of the guard.
//
// The reserved 4 KB EEPROM sector the linker puts immediately above the
// filesystem (_EEPROM_start == _FS_end; 0xFFF000 on the 16MB board, 0x3FE000 on
// the Pico 2) stays as the FALLBACK, claimed by the first slot that cannot get
// a file. That happens when the volume is full or too fragmented for an 8 KB
// contiguous run, and - by design - on a cartridge that has never been plugged
// into a PC: creating a file writes flash through the Adafruit driver, which
// calls rp2040.idleOtherCore(), and parking core 1 while the Atari is running
// would leave the 6507 reading a floating bus. So files are created only when
// the cartridge is connected to a PC (nv_init's may_create), and until then FA2
// saves into the reserved sector.
//
// ---------------------------------------------------------------------------
// Writing flash while core 1 drives the Atari bus
// ---------------------------------------------------------------------------
// flash_range_erase() turns XIP off for the duration, so for the tens of
// milliseconds it runs, ANY read of 0x1xxxxxxx by either core is undefined.
// Core 0 is the one calling and it disables its own interrupts. Core 1 has to
// survive on SRAM alone. It does, and this is checked rather than assumed:
//
//   * setup1() - which carries the FA2 bus loop inline - is
//     __time_critical_func(), i.e. linked into SRAM together with its literal
//     pool. `nm` puts it at 0x2000xxxx, not 0x1000xxxx.
//   * rom_table and ram_table are in SRAM.
//   * The only interrupts enabled on core 1 are the SysTick exception and the
//     multicore doorbell, and BOTH handlers are __no_inline_not_in_flash_func
//     in the Arduino core (RP2040Support.h: _SystickHandler, _MFIFO::_irq).
//   * We deliberately do NOT call rp2040.idleOtherCore() on this path.
//
// The FA2 wait loop has no timeout at all (see setup_fa2()), so 45 ms - or
// 400 ms, the datasheet's worst-case sector erase - is a pause, not a missed
// deadline. Real hardware pauses for 101 ms at exactly this point.
//
// ---------------------------------------------------------------------------
// Sector layout (version 2)
// ---------------------------------------------------------------------------
//   0x000  4  magic "A104"
//   0x004  1  layout version
//   0x005  1  flags: bit 0 = the payload holds real data
//   0x006  2  number of commits since this sector was blank
//   0x008  4  last erase   time, microseconds
//   0x00C  4  last program time, microseconds
//   0x010  4  worst erase   time seen, microseconds
//   0x014  4  worst program time seen, microseconds
//   0x018  4  this sector's own flash offset      <- the identity guard
//   0x01C  1  slot id: 0 = FA2, 1 = HSC           <- the identity guard
//   0x100  N  payload
//
// 0x000-0x017 is byte-for-byte what 0.50 wrote (version 1), so a table saved by
// a 0.50 test build into the reserved sector is still read back; it is
// rewritten as version 2 on the next commit. The timings are the phase-0
// measurement TODO.md asked for. They cannot be written by the same program
// pass that measures them, so the payload page goes first and the header page
// second - flash programs 1->0 only, and a page already written cannot be
// revised without erasing again.

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/time.h"

// Linker symbols. Declared as unbounded arrays, not scalars: GCC otherwise
// warns that reading a sector overruns a one-byte object.
extern uint8_t _EEPROM_start[];
extern uint8_t _FS_start[];
extern uint8_t _FS_end[];

#define NV_SECTOR_BYTES   4096u
#define NV_FILE_BYTES     8192u
#define NV_IMAGE_BYTES     512u   // header page + one payload page, staged in RAM

#define NV_SLOT_FA2          0
#define NV_SLOT_HSC          1
#define NV_SLOT_COUNT        2

#define NV_PAYLOAD_OFF   0x100
#define NV_FA2_LEN         256
#define NV_HSC_LEN        2048    // reserved, TODO position 11

#define NV_OFF_MAGIC     0x000
#define NV_OFF_VERSION   0x004
#define NV_OFF_FLAGS     0x005
#define NV_OFF_COUNT     0x006
#define NV_OFF_ERASE_US  0x008
#define NV_OFF_PROG_US   0x00C
#define NV_OFF_ERASE_MAX 0x010
#define NV_OFF_PROG_MAX  0x014
#define NV_OFF_SELF      0x018
#define NV_OFF_SLOT      0x01C

#define NV_VERSION           2
#define NV_FLAG_DATA      0x01

#define NV_BACKEND_NONE      0
#define NV_BACKEND_FILE      1
#define NV_BACKEND_SECTOR    2

// FA2_DIAG_NOWRITE: run the whole handshake and the whole busy delay, but never
// touch flash. Splits "the new bus branch and the core-0 handover broke the
// picture" from "erasing flash under a live bus broke it" - the two suspects
// this feature adds, and the reason CLAUDE.md keeps the POKEY_DIAG_E12 lesson
// written down.
#ifndef FA2_DIAG_NOWRITE
#define FA2_DIAG_NOWRITE 0
#endif
#define FA2_DIAG_NOWRITE_US 55000u

// The names the user sees on the USB drive. These live in .rodata, i.e. flash,
// which is fine here and would not be on core 1: nothing in this file runs on
// core 1, and nothing reads them while XIP is off.
static const char *const nv_slot_path[NV_SLOT_COUNT] = {
  "/.save_fa2.data",
  "/.save_hsc.data",
};

typedef struct {
  uint32_t off;      // 4096-aligned flash offset of this slot's own sector; 0 = none
  uint8_t  backend;  // NV_BACKEND_*
} nv_target_t;

static nv_target_t nv_target[NV_SLOT_COUNT];

// Which step refused each slot: 0 accepted, 1 open, 2 size, 3 contiguous,
// 4 bounds, 5 mapping. Two bytes, and it turns the boot log into an instrument
// - 0.58's log could say a slot was refused but not where, which cost a whole
// hardware round to narrow down by reading library sources.
uint8_t nv_fail[NV_SLOT_COUNT];
static uint8_t     nv_image[NV_IMAGE_BYTES];
static bool        nv_inited = false;

static inline uint32_t nv_fs_lo(void)   { return (uint32_t)((uintptr_t)_FS_start     - XIP_BASE); }
static inline uint32_t nv_fs_hi(void)   { return (uint32_t)((uintptr_t)_FS_end       - XIP_BASE); }
static inline uint32_t nv_res_off(void) { return (uint32_t)((uintptr_t)_EEPROM_start - XIP_BASE); }

// Pull a sector's header page into the RAM mirror.
static void nv_hdr_read(uint32_t off) {
  memcpy(nv_image, (const void *)(uintptr_t)(XIP_BASE + off), 256);
}

// Does the sector at off identify itself as this slot's save area?
static bool nv_hdr_is(uint32_t off, uint8_t slot) {
  if (memcmp(nv_image + NV_OFF_MAGIC, "A104", 4) != 0) return false;
  const uint8_t v = nv_image[NV_OFF_VERSION];
  if (v == NV_VERSION) {
    uint32_t self;
    memcpy(&self, nv_image + NV_OFF_SELF, 4);
    return self == off && nv_image[NV_OFF_SLOT] == slot;
  }
  // 0.50 wrote layout 1, which had neither field, and only ever into the
  // reserved sector holding FA2. Accept that one case so a table saved by a
  // 0.50 test build is not thrown away; the next commit rewrites it as v2.
  return v == 1 && slot == NV_SLOT_FA2 && off == nv_res_off();
}

// Does the flash offset we computed really point at this file's own bytes?
//
// This is the check the other three could not make: they are about the file's
// ALLOCATION and about staying inside the filesystem, and the 0.58 bug was
// neither - it was a volume-relative offset used as a flash-relative one. The
// bounds test even passed, because it compared a volume offset against flash
// bounds and the number happened to be large enough. Apples against oranges
// never fails loudly.
//
// HOW THE TRANSLATION IS ACTUALLY PROVEN. Not by reading the same bytes twice.
// 0.58 read 64 bytes "through the filesystem" and 64 "through XIP" and compared
// them, but Adafruit_FlashTransport_RP2040::readMemory() is
//
//     memcpy(data, (void *)(XIP_BASE + _start_addr + addr), len);
//
// so both reads take the same road, through the same FAT-derived translation.
// With nothing planted first they are the same read, and the comparison cannot
// fail for the reason it exists.
//
// The half that proves something is the WRITE. SdFat derives the sector from
// the FAT chain; we derive the address ourselves; if the two disagree, bytes
// written through the file do not appear at the address we are about to erase.
// Two independent translations, one comparison - that is a round trip.
//
// AND IT IS DONE ONCE. A write is only possible on a PC, so repeating it every
// boot made acceptance depend on may_create and left the Atari - the side that
// matters - with no way to say yes. So the answer is kept in the window itself,
// in either of the two shapes that can be there:
//
//   "A104MAP" + win + base   planted here by a PC boot that verified it
//   "A104" nvstore header    written by the first real commit, and it already
//                            carries its own address in NV_OFF_SELF
//
// Both name the address they sit at, so finding one at the address we computed
// IS the proof, and re-reading it costs nothing and works on the Atari. A wrong
// address does not land on a sector that names it by luck.
// Is what nv_hdr_read() just pulled in the mapping anchor for THIS address?
// Used twice: to accept a slot without writing, and to allow the first commit
// to erase the anchor it was authorised by.
static bool nv_hdr_is_anchor(uint32_t off) {
  if (memcmp(nv_image, "A104MAP", 8) != 0) return false;
  uint32_t swin = 0;
  memcpy(&swin, nv_image + 8, 4);
  return swin == off;
}

static bool nv_anchor_present(uint32_t base, uint32_t win, uint8_t slot) {
  nv_hdr_read(win);                          // 256 bytes of the window into nv_image
  if (nv_hdr_is(win, slot)) return true;     // committed at least once

  if (!nv_hdr_is_anchor(win)) return false;
  uint32_t sbase = 0;
  memcpy(&sbase, nv_image + 12, 4);
  return sbase == base;   // the run it was measured from has not moved
}

static bool nv_mapping_ok(File32 *f, uint32_t base, uint32_t win, uint8_t slot,
                          bool may_create) {
  if (nv_anchor_present(base, win, slot)) return true;

  // No anchor. Only a PC can make one, and until it does the slot stays refused
  // rather than written to on an address nothing has checked - that is the
  // lesson of 0.58, where a wrong address erased 4 KB of somebody's ROM.
  if (!may_create) return false;

  uint8_t sig[64], back[64];
  memcpy(sig, "A104MAP", 8);
  memcpy(sig + 8, &win, 4);
  memcpy(sig + 12, &base, 4);
  for (uint32_t i = 16; i < sizeof(sig); i++) sig[i] = (uint8_t)(i * 7u + (win >> 12));

  const uint32_t pos = win - base;
  if (!f->seekSet(pos) || f->write(sig, sizeof(sig)) != (int)sizeof(sig)) return false;
  f->sync();
  flash.syncBlocks();     // otherwise it is still in the driver's RAM cache
  fatfs.cacheClear();

  // Read it back at OUR address, the one a commit would erase.
  memcpy(back, (const void *)(uintptr_t)(XIP_BASE + win), sizeof(back));
  return memcmp(sig, back, sizeof(sig)) == 0;
}

// Resolve both slots. Core 0 only, once, at boot, BEFORE any game starts:
// creating a file writes flash through the Adafruit driver, which parks core 1.
// may_create must be false whenever the Atari is running.
static void nv_init(bool may_create) {
  if (nv_inited) return;
  nv_inited = true;

  bool reserved_taken = false;
  for (uint8_t slot = 0; slot < NV_SLOT_COUNT; slot++) {
    nv_target_t *t = &nv_target[slot];
    t->off = 0;
    t->backend = NV_BACKEND_NONE;

    nv_fail[slot] = 1;
    // O_RDWR where writing is allowed, because planting the anchor uses
    // FatFile::write(), which returns 0 unless the handle is writable
    // (SdFat FatFile.cpp:1370). 0.58 opened O_RDONLY and then tried to write,
    // so an existing file could never be anchored - only a freshly created one,
    // which is writable for preAllocate() anyway. That is why the very first PC
    // boot worked and nothing after it did.
    File32 f = fatfs.open(nv_slot_path[slot], may_create ? O_RDWR : O_RDONLY);
    if (!f && may_create) {
      f = fatfs.open(nv_slot_path[slot], O_RDWR | O_CREAT | O_EXCL);
      if (f) {
        // preAllocate() allocates CONTIGUOUS clusters and sets the size; it is
        // exactly what createContiguous() does after opening.
        if (!f.preAllocate(NV_FILE_BYTES)) {
          f.close();
          fatfs.remove(nv_slot_path[slot]);
        } else {
          f.attrib(FS_ATTRIB_HIDDEN);
          f.sync();
        }
      }
    }

    if (f) {
      uint32_t bgn = 0, end = 0;
      nv_fail[slot] = (f.fileSize() < NV_FILE_BYTES) ? 2 : 3;
      // contiguousRange() walks the FAT chain and fails unless every cluster is
      // contiguous - the filesystem's own answer, not a cached flag.
      if (f.fileSize() >= NV_FILE_BYTES && f.contiguousRange(&bgn, &end) &&
          (uint64_t)(end - bgn + 1) * 512u >= NV_FILE_BYTES) {
        // UNITS. contiguousRange() answers in sectors of the BLOCK DEVICE, and
        // that device is the filesystem region, which starts at _FS_start.
        // flash_range_erase() counts from XIP_BASE. 0.51 to 0.57 passed
        // bgn * 512 straight through, so every commit erased 0xFF000 bytes -
        // 1020 KB - BELOW the save file and still inside the volume, i.e. into
        // somebody else's ROM. Found on hardware 2026-09-08.
        const uint32_t base = nv_fs_lo() + bgn * 512u;
        const uint32_t win  = (base + NV_SECTOR_BYTES - 1u) & ~(NV_SECTOR_BYTES - 1u);
        const bool fits = (win + NV_SECTOR_BYTES <= base + NV_FILE_BYTES &&
                           win >= nv_fs_lo() && win + NV_SECTOR_BYTES <= nv_fs_hi());
        nv_fail[slot] = fits ? 5 : 4;
        if (fits && nv_mapping_ok(&f, base, win, slot, may_create)) {
          nv_fail[slot] = 0;
          t->off = win;
          t->backend = NV_BACKEND_FILE;
        }
      }
      f.close();
    }

    if (!t->off && !reserved_taken) {
      // No file: fall back to the reserved sector above the filesystem. First
      // come, first served - there is one, and FA2 is resolved first.
      t->off = nv_res_off();
      t->backend = NV_BACKEND_SECTOR;
      reserved_taken = true;
    }
  }

  // Leave both caches clean, so nothing the library is holding can later be
  // flushed over a sector we are about to write raw.
  if (may_create) {
    flash.syncBlocks();
    fatfs.cacheClear();
  }
}

// NV_DIAG_FOOTER: the twelve characters that say whether the save files were
// accepted, for the menu footer. Reads only what nv_init() already decided, so
// it writes nothing and costs one snprintf at boot. See tools/apply_nv_footer.py.
#ifndef NV_DIAG_FOOTER
#define NV_DIAG_FOOTER 0
#endif
#if NV_DIAG_FOOTER
static char nv_footer_buf[16];
static const char *nv_slots_footer(void) {
  snprintf(nv_footer_buf, sizeof(nv_footer_buf), "SAVE F%u%u H%u%u",
           (unsigned)(nv_target[NV_SLOT_FA2].backend % 10u),
           (unsigned)(nv_fail[NV_SLOT_FA2] % 10u),
           (unsigned)(nv_target[NV_SLOT_HSC].backend % 10u),
           (unsigned)(nv_fail[NV_SLOT_HSC] % 10u));
  return nv_footer_buf;
}
#endif

static bool nv_slot_ready(uint8_t slot) {
  return slot < NV_SLOT_COUNT && nv_target[slot].off != 0;
}

// True if this slot has ever been written, and leaves its header in nv_image.
// Callers must NOT copy the payload out when this is false: an unwritten slot is
// whatever the sector happened to hold, which for a score table is worse than
// empty.
static bool nv_slot_has_data(uint8_t slot) {
  if (!nv_slot_ready(slot)) return false;
  const uint32_t off = nv_target[slot].off;
  nv_hdr_read(off);
  return nv_hdr_is(off, slot) && (nv_image[NV_OFF_FLAGS] & NV_FLAG_DATA);
}

static bool nv_read_slot(uint8_t slot, void *dst, uint32_t len) {
  if (len > NV_SECTOR_BYTES - NV_PAYLOAD_OFF) return false;
  if (!nv_slot_has_data(slot)) return false;
  memcpy(dst, (const void *)(uintptr_t)(XIP_BASE + nv_target[slot].off + NV_PAYLOAD_OFF), len);
  return true;
}

// Erase this slot's sector and write the payload back. Core 0 only, and only
// when core 1 is either idle or running a loop proven to be flash-free.
// len must be a multiple of the 256-byte flash page and fit the staging mirror.
static bool nv_write_slot(uint8_t slot, const void *src, uint32_t len) {
  if (!nv_slot_ready(slot)) return false;
  if (len == 0 || (len & (FLASH_PAGE_SIZE - 1)) || NV_PAYLOAD_OFF + len > NV_SECTOR_BYTES)
    return false;

  const uint32_t off = nv_target[slot].off;

  // Bounds, again, at the last possible moment.
  if (off & (NV_SECTOR_BYTES - 1)) return false;
  if (nv_target[slot].backend == NV_BACKEND_FILE) {
    if (off < nv_fs_lo() || off + NV_SECTOR_BYTES > nv_fs_hi()) return false;
  } else if (off != nv_res_off()) {
    return false;
  }

  // Identity guard. A sector that already carries one of our headers must say it
  // is THIS slot's and must know its own address, or we do not erase it.
  //
  // The mapping anchor is the one exception, and it has to be: "A104MAP" starts
  // with "A104" but is not a header, so without this the FIRST commit to a
  // newly anchored file would be refused and the table would never be written.
  // It is only an exception when it names this exact address - the same
  // evidence nv_init() accepted the slot on.
  nv_hdr_read(off);
  const bool known = nv_hdr_is(off, slot);
  if (!known && memcmp(nv_image + NV_OFF_MAGIC, "A104", 4) == 0 &&
      !nv_hdr_is_anchor(off))
    return false;   // ours, but a different slot or a different address

  uint16_t count = 0;
  uint32_t emax = 0, pmax = 0;
  if (known) {
    memcpy(&count, nv_image + NV_OFF_COUNT, 2);
    memcpy(&emax,  nv_image + NV_OFF_ERASE_MAX, 4);
    memcpy(&pmax,  nv_image + NV_OFF_PROG_MAX, 4);
  }
  count++;

  // A payload that fits the staging mirror is COPIED before the erase, because
  // its source is cartridge RAM and the erase takes tens of milliseconds. One
  // that does not fit - the 2048-byte HSC table - is programmed straight out of
  // the caller's buffer instead of growing the mirror to hold it: the bootrom
  // takes its source as a plain const uint8_t* and flash_range_program() asserts
  // page alignment only on the OFFSET and the COUNT (checked in the SDK, not
  // assumed). The caller is responsible for the source being quiet; hsc.h waits
  // for a quarter-second of silence on the NVRAM window before it commits.
  const uint8_t *payload;
  if (NV_PAYLOAD_OFF + len <= NV_IMAGE_BYTES) {
    memset(nv_image, 0, NV_IMAGE_BYTES);
    memcpy(nv_image + NV_PAYLOAD_OFF, src, len);
    payload = nv_image + NV_PAYLOAD_OFF;
  } else {
    memset(nv_image, 0, FLASH_PAGE_SIZE);   // the header page is all we stage
    payload = (const uint8_t *)src;
  }

#if FA2_DIAG_NOWRITE
  busy_wait_us_32(FA2_DIAG_NOWRITE_US);
  (void)emax; (void)pmax; (void)count;
#else
  uint32_t ints = save_and_disable_interrupts();
  const uint32_t t0 = time_us_32();
  flash_range_erase(off, NV_SECTOR_BYTES);
  const uint32_t t1 = time_us_32();
  flash_range_program(off + NV_PAYLOAD_OFF, payload, len);
  const uint32_t t2 = time_us_32();

  const uint32_t eus = t1 - t0, pus = t2 - t1;
  if (eus > emax) emax = eus;
  if (pus > pmax) pmax = pus;

  memcpy(nv_image + NV_OFF_MAGIC, "A104", 4);
  nv_image[NV_OFF_VERSION] = NV_VERSION;
  nv_image[NV_OFF_FLAGS]   = NV_FLAG_DATA;
  memcpy(nv_image + NV_OFF_COUNT, &count, 2);
  memcpy(nv_image + NV_OFF_ERASE_US,  &eus,  4);
  memcpy(nv_image + NV_OFF_PROG_US,   &pus,  4);
  memcpy(nv_image + NV_OFF_ERASE_MAX, &emax, 4);
  memcpy(nv_image + NV_OFF_PROG_MAX,  &pmax, 4);
  memcpy(nv_image + NV_OFF_SELF, &off, 4);
  nv_image[NV_OFF_SLOT] = slot;
  flash_range_program(off, nv_image, FLASH_PAGE_SIZE);
  restore_interrupts(ints);
#endif
  return true;
}

// Worst erase/program times and the commit count, for the FA2T footer.
static bool nv_stat(uint8_t slot, uint32_t *erase_us, uint32_t *prog_us, uint16_t *count) {
  if (!nv_slot_has_data(slot)) return false;   // leaves the header in nv_image
  memcpy(erase_us, nv_image + NV_OFF_ERASE_MAX, 4);
  memcpy(prog_us,  nv_image + NV_OFF_PROG_MAX, 4);
  memcpy(count,    nv_image + NV_OFF_COUNT, 2);
  return true;
}

#endif // NVSTORE_H
