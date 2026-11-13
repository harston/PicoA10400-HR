#ifndef ARM_CART_H
#define ARM_CART_H

// ARM-coprocessor cartridges - Harmony's DPC+, CDF, CDFJ and CDFJ+ (the Champ
// Games formats), plus BUS, ACE and ELF. CHAMP_ARM_TODO.md stage 0.
//
// Must be included AFTER the CART_TYPE_* constants, ext_to_cart_type_map[], rom.h
// (rom_table) and the SdFat globals.
//
// ---------------------------------------------------------------------------
// Why this exists
// ---------------------------------------------------------------------------
// None of these can run here yet: every game in them runs its own C code on the
// cartridge's ARM chip (CHAMP_ARM.md). Until 0.76 identify_cartridge() did not know
// them at all, so a 32 KB image fell through to F4, a 64 KB one to F0 and a 128 KB
// one to no type, and each of them was started and hung the console - about 1 260
// files of the library, confirmed on hardware with TEST_ROMS_ARM (2026-09-29).
//
// What the firmware does with them now: the menu MARKS them (MENU32: "X" and "A" in
// the two mark columns, the colour of an oversized file; 12 columns: red like an
// oversized file), and choosing one is REFUSED - the file is not loaded, the menu
// stays where it was and the footer says why. arm_cart_runnable() is the single
// place later stages open up as the formats become supported.
//
// ---------------------------------------------------------------------------
// How they are recognised
// ---------------------------------------------------------------------------
// Stella's signatures (CartDetector.cxx, CartCDF.cxx), in Stella's order and only in
// the size buckets where Stella looks for them:
//   ELF    "\x7fELF", little endian, ET_REL, ARM - any size
//   ACE    "ACE-2600" at offset 0 (the check is_ace_cartridge() makes) - any size
//   CDF*   "CDF" three times, or "PLUSCDFJ"       - 32/64/128/256/512 KB
//            variant from the driver: the words PLUS,CDFJ,1 -> CDFJ+; else the byte
//            after the "CDF" triplet: 0 -> CDF0, 'J' -> CDFJ, anything else -> CDF1
//   DPC+   "DPC+" twice                         - 29 and 32 KB
//   BUS    "BUS" twice                          - 32 KB
// but in the first 4 KB of the file only, not in the whole image. Measured over the
// whole library (tools/arm_carts/arm_scan.py --sigpos): the furthest signature byte
// of any ARM image sits at offset 3063, so 4 KB sees every one of them. The same
// function runs in the listing, in LoadGame() before anything is loaded, and in
// identify_cartridge() - on the same 4 KB - so the three can never disagree.
//
// One gate comes before the signatures (0.79): a Harmony image - DPC+, CDF*, BUS -
// begins with its ARM driver, and the driver's first instruction loads PC
// (MOV PC,#imm or LDR PC,[PC,#imm]: condition "always", destination PC). All 1 262
// such images in the library start that way; of the 3 128 other files of the same
// sizes, 51 do. The listing therefore reads 64 bytes of every file and the 4 KB only
// when the first word passes - 4 KB per file had cost 238 ms for a directory of 84
// files of 32 KB on the RP2350A clone (ARMT image, 2026-11-13), 64 bytes cost 10 ms.
//
// Only files whose extension leaves the type to autodetection (.bin, .a26, .rom,
// none, anything unknown) are checked. A .ace file is refused by its name alone:
// the extension table sends it straight to CART_TYPE_ACE, which has no bus loop,
// and not every one carries the "ACE-2600" magic - UnoCart's "ACE-UF00" images
// (StarGiraf, three files in the library) do not. A file forced to any other type
// by its extension (.f4 ...) is the user's explicit choice and is left alone,
// exactly as identify_cartridge() leaves it alone.
//
// Where the 4 KB go: rom_table past its first 32 KB. The 12-column kernel lives in
// the first 4 KB and the MENU32 page is an F4 image of exactly 32 KB, so that part
// is untouched while the menu runs - and a game, which owns rom_table, only ever
// comes after the menu. No RAM is added for this.
// tools/rom_compat_detect.py mirrors all of this.

#define ARM_KIND_NONE  0
#define ARM_KIND_DPCP  1
#define ARM_KIND_CDF0  2
#define ARM_KIND_CDF1  3
#define ARM_KIND_CDFJ  4
#define ARM_KIND_CDFJP 5
#define ARM_KIND_BUS   6
#define ARM_KIND_ACE   7
#define ARM_KIND_ELF   8

#define ARM_PROBE_BYTES 4096u
#define ARM_PROBE_HEAD  64u        // enough for the ELF and ACE headers
#define ARM_PROBE_BUF   (rom_table + 0x8000)

// Kind of the ARM cartridge identify_cartridge() last recognised (for the stages
// that will run them; nothing reads it yet).
static uint8_t arm_variant = ARM_KIND_NONE;

static const char *arm_kind_name(uint8_t k) {
  switch (k) {
    case ARM_KIND_DPCP:  return "DPC+";
    case ARM_KIND_CDF0:  return "CDF0";
    case ARM_KIND_CDF1:  return "CDF1";
    case ARM_KIND_CDFJ:  return "CDFJ";
    case ARM_KIND_CDFJP: return "CDFJ+";
    case ARM_KIND_BUS:   return "BUS";
    case ARM_KIND_ACE:   return "ACE";
    case ARM_KIND_ELF:   return "ELF";
    default:             return "";
  }
}

static int arm_cart_type_of(uint8_t k) {
  switch (k) {
    case ARM_KIND_DPCP:  return CART_TYPE_DPCP;
    case ARM_KIND_CDF0:
    case ARM_KIND_CDF1:
    case ARM_KIND_CDFJ:
    case ARM_KIND_CDFJP: return CART_TYPE_CDF;
    case ARM_KIND_BUS:   return CART_TYPE_BUS;
    case ARM_KIND_ACE:   return CART_TYPE_ACE;
    case ARM_KIND_ELF:   return CART_TYPE_ELF;
    default:             return CART_TYPE_NONE;
  }
}

// Stage 0: none of them can run. Later stages open cases here, one at a time.
static bool arm_cart_runnable(uint8_t k) {
  (void)k;
  return false;
}

static bool arm_size_is(uint32_t size, uint32_t kb) { return size == kb * 1024u; }

// Occurrences of sig in b[0..n), counted at every position (Stella's searchForBytes).
static uint32_t arm_count(const uint8_t *b, uint32_t n, const char *sig, uint32_t len) {
  uint32_t hits = 0;
  for (uint32_t i = 0; i + len <= n; i++)
    if (memcmp(b + i, sig, len) == 0) hits++;
  return hits;
}

static uint32_t arm_word(const uint8_t *b, uint32_t i) {
  return (uint32_t)b[i] | (uint32_t)b[i + 1] << 8 | (uint32_t)b[i + 2] << 16 | (uint32_t)b[i + 3] << 24;
}

// The gate above: does the image start with an ARM instruction that loads PC?
static bool arm_harmony_entry(const uint8_t *head, uint32_t n) {
  return n >= 4 && (arm_word(head, 0) & 0xF000F000u) == 0xE000F000u;
}

// head = the first n bytes of a file of file_size bytes (n may be less than 4 KB,
// only the first 4 KB are ever looked at).
static uint8_t arm_probe(const uint8_t *head, uint32_t n, uint32_t file_size) {
  if (n > ARM_PROBE_BYTES) n = ARM_PROBE_BYTES;
  if (n >= 52 && file_size >= 52 && head[0] == 0x7F && head[1] == 'E' && head[2] == 'L' &&
      head[3] == 'F' && head[5] == 1 && head[0x10] == 1 && head[0x12] == 0x28)
    return ARM_KIND_ELF;
  if (n >= sizeof(ACEFileHeader) && memcmp(head, "ACE-2600", 8) == 0)   // is_ace_cartridge()'s rule
    return ARM_KIND_ACE;
  if (!arm_harmony_entry(head, n))
    return ARM_KIND_NONE;

  const bool s29 = arm_size_is(file_size, 29), s32 = arm_size_is(file_size, 32);
  const bool cdf_size = s32 || arm_size_is(file_size, 64) || arm_size_is(file_size, 128) ||
                        arm_size_is(file_size, 256) || arm_size_is(file_size, 512);
  if (cdf_size && (arm_count(head, n, "CDF", 3) >= 3 || arm_count(head, n, "PLUSCDFJ", 8) >= 1)) {
    const uint32_t drv = n < 2048 ? n : 2048;          // the driver is the first 2 KB
    for (uint32_t i = 0; i + 12 <= drv; i += 4)
      if (arm_word(head, i) == 0x53554C50u && arm_word(head, i + 4) == 0x4A464443u &&
          arm_word(head, i + 8) == 1u)                  // "PLUS", "CDFJ", 1
        return ARM_KIND_CDFJP;
    for (uint32_t i = 0; i + 12 <= drv; i += 4)
      if (memcmp(head + i, "CDF", 3) == 0 && memcmp(head + i + 4, "CDF", 3) == 0 &&
          memcmp(head + i + 8, "CDF", 3) == 0)
        return head[i + 3] == 0 ? ARM_KIND_CDF0 : head[i + 3] == 0x4A ? ARM_KIND_CDFJ : ARM_KIND_CDF1;
    return arm_count(head, n, "PLUSCDFJ", 8) ? ARM_KIND_CDFJP : ARM_KIND_CDF1;
  }
  if ((s29 || s32) && arm_count(head, n, "DPC+", 4) >= 2)
    return ARM_KIND_DPCP;
  if (s32 && arm_count(head, n, "BUS", 3) >= 2)
    return ARM_KIND_BUS;
  return ARM_KIND_NONE;
}

// What the extension decides, by the same parse and table identify_cartridge()
// uses: ARM_EXT_PROBE - the type is left to autodetection (CART_TYPE_NONE, or an
// extension the table does not know), so the content decides; ARM_EXT_ACE - the
// table forces CART_TYPE_ACE; ARM_EXT_FORCED - any other type, the user's choice.
#define ARM_EXT_PROBE  0
#define ARM_EXT_ACE    1
#define ARM_EXT_FORCED 2

static uint8_t arm_ext_rule(const char *name) {
  int pos = 0;
  const char *dot = strrchr(name, '.');
  if (dot != NULL) pos = (int)(dot - name);
  char ext[3];
  for (int i = 0; i < 3; i++) {
    char c = name[pos + 1 + i];
    ext[i] = (c > 96) ? (char)(c - 32) : c;
    if (c == 0) {                       // shorter than three: the rest is zero too
      for (; i < 3; i++) ext[i] = 0;
      break;
    }
  }
  for (const EXT_TO_CART_TYPE_MAP *p = ext_to_cart_type_map; p->ext; p++) {
    char test[3];
    memcpy(test, p->ext, 3);
    if (test[0] == ext[0] && test[1] == ext[1] && test[2] == ext[2])
      return p->cart_type == CART_TYPE_NONE ? ARM_EXT_PROBE
           : p->cart_type == CART_TYPE_ACE  ? ARM_EXT_ACE : ARM_EXT_FORCED;
  }
  return ARM_EXT_PROBE;                 // unknown extension: autodetected
}

// f open. Reads what the probe needs into ARM_PROBE_BUF and returns the kind:
// 64 bytes of every file, and the rest of the first 4 KB only for a size where an
// ARM format can be AND a first word that passes arm_harmony_entry().
static uint8_t arm_probe_file(FatFile *f) {
  const uint32_t size = f->fileSize();
  const bool candidate = arm_size_is(size, 29) || arm_size_is(size, 32) || arm_size_is(size, 64) ||
                         arm_size_is(size, 128) || arm_size_is(size, 256) || arm_size_is(size, 512);
  uint32_t want = ARM_PROBE_HEAD;
  if (want > size) want = size;
  if (want < 8 || !f->seekSet(0)) return ARM_KIND_NONE;
  int got = f->read(ARM_PROBE_BUF, want);
  if (got <= 0) return ARM_KIND_NONE;
  if (candidate && arm_harmony_entry(ARM_PROBE_BUF, (uint32_t)got)) {
    // Again from offset 0, in one read: sector-aligned, the same read 0.76-0.78
    // made for every file of these sizes (measured ~2.7 ms per file on the
    // RP2350A clone), rather than an unaligned tail from byte 64.
    const uint32_t full = size < ARM_PROBE_BYTES ? size : ARM_PROBE_BYTES;
    if (!f->seekSet(0)) return ARM_KIND_NONE;
    got = f->read(ARM_PROBE_BUF, full);
    if (got <= 0) return ARM_KIND_NONE;
  }
  return arm_probe(ARM_PROBE_BUF, (uint32_t)got, size);
}

// What the listing and LoadGame() ask of an open file: is it an ARM cartridge,
// and which kind.
static uint8_t arm_classify(const char *name, FatFile *f) {
  switch (arm_ext_rule(name)) {
    case ARM_EXT_ACE:   return ARM_KIND_ACE;
    case ARM_EXT_PROBE: return arm_probe_file(f);
    default:            return ARM_KIND_NONE;
  }
}

#endif
