#ifndef DPCPLUS_H
#define DPCPLUS_H

// DPC+ (Harmony) - the 6507 side of the cartridge: six 4 KB banks, eight data
// fetchers with fractional and windowed variants, the random number generator and
// the three-voice music generator. CHAMP_ARM_TODO.md stage 1.
//
// Everything here is static inline and touches nothing but the dpcp_state it is
// given, so the same code runs in emulate_dpcplus() on the cartridge and in the
// host test tools/dpcp_sim/, which replays a Gopher2600 bus trace through it and
// compares every byte. Keep it that way: no globals, no Arduino calls, no flash
// constants.
//
// Semantics follow Stella (CartDPCPlus.cxx), checked against Gopher2600
// (hardware/memory/cartridge/dpcplus). The one deliberate difference is fast
// fetch, see dpcp_state.lda_next.
//
// 0.81: the value of every read register is kept ready in dpcp_state.rv[], so a
// register read on the bus is one table load, as fast as a ROM read. 0.80 computed
// it at the read, and on the RP2040 (Cortex-M0+) the slowest reads took 85 cycles
// before the byte went out - the DPC+ games did not start there. Every function
// below that changes state refreshes the entries that depend on it;
// dpcp_read_compute() is the old computation, kept for the host test, which checks
// rv[] against it after every access.
//
// dpcp_init() calls memset/memcpy and is meant for core 0 at load time
// (identify_cartridge()); the rest is what the bus loop calls, and none of it
// calls anything. Switches in the loop must be compiled without jump tables: on
// the RP2040 those go through libgcc's __gnu_thumb1_case_* helpers, in flash.
//
// What is NOT here: running the cartridge's ARM program. A write of 254 or 255 to
// CALLFUNCTION is counted and otherwise ignored - LoadGame() never starts an image
// whose 6507 program writes CALLFUNCTION at all (arm_cart.h), so a running image
// should never get there. Functions 1 and 2 (copy into display data) are stage 2.

#include <stdint.h>
#include <string.h>

// Everything the bus loop calls must be pasted into it: left to itself GCC
// outlined dpcp_amp_update() into a .constprop clone in flash and called it
// through a veneer (seen in the .elf of the first build). always_inline is what
// keeps check_hotpath.sh and the "no flash calls" rule honest here.
#define DPCP_INLINE static inline __attribute__((always_inline))

// The host test (tools/dpcp_sim/replay.c) defines DPCP_HOST_TEST to switch off
// single refreshes of rv[] (--break=6/7) or the pointer counts (--break=8) and show
// that it notices. On the
// cartridge DPCP_BROKEN() is the constant 0 and compiles to nothing.
#ifdef DPCP_HOST_TEST
extern int dpcp_test_break;
#define DPCP_BROKEN(n) (dpcp_test_break == (n))
#else
#define DPCP_BROKEN(n) 0
#endif

#define DPCP_RANDOM_INIT 0x2B435044u   // "DPC+", Stella and Gopher2600 alike
#define DPCP_FRAC_JITTER 0x0F00FFu     // DFxFRACLOW keeps the low byte (most drivers)
#define DPCP_FRAC_STABLE 0x0F0000u     // ... or clears it (drivers 5f80b5a5, 8dd73b44)

// Harmony RAM layout used by the drivers (and by stage 3's ARM code later): the
// display data 4 KB at 0xC00, the frequency table 1 KB at 0x1C00. Past the 8 KB of
// Harmony RAM, 4 KB more for the pointer counts (dpcp_state.cnt, 0.81).
#define DPCP_RAM_DISPLAY 0x0C00u
#define DPCP_RAM_FREQ    0x1C00u
#define DPCP_RAM_CNT     0x2000u
#define DPCP_RAM_SIZE    0x3000u

typedef struct {
  const uint8_t *prog;     // the 6507 program, 6 banks of 4 KB
  uint8_t *disp;           // display data, 4 KB (read and written by the 6507)
  const uint8_t *freq;     // frequency table, 1 KB, 32-bit little-endian entries
  uint8_t *cnt;            // 4 KB: how many pointers stand at each display address
  uint32_t bank;           // offset of the current bank within prog
  uint32_t rnd;            // 32-bit LFSR
  uint32_t frac_mask;      // DPCP_FRAC_JITTER or DPCP_FRAC_STABLE
  uint32_t frac[8];        // fractional pointers, 12.8 fixed point (20 bits)
  uint32_t mctr[3];        // music phase counters
  uint32_t mfreq[3];       // music phase increments, per 20 kHz tick
  uint16_t ctr[8];         // data pointers, 12 bits
  uint8_t  frac_inc[8];
  uint8_t  top[8], bot[8];
  uint8_t  wave[3];        // waveform number per voice (7 bits, 32-byte waveforms)
  uint8_t  rv[0x28];       // what a read of register $00-$27 returns now (0.81)
  uint8_t  param[8], param_ptr;
  uint8_t  fastfetch;      // FASTFETCH written with 0
  // Fast fetch: after the 6507 fetched $A9 (LDA #) from ROM at address A, the byte
  // at A+1 is the operand, and an operand below $28 names a register to read
  // instead. Stella and Gopher2600 arm on ANY ROM byte $A9 and fire on the next ROM
  // read, wherever it is - so a table byte $A9 followed by an opcode below $28
  // (JSR, BPL, CLC...) would turn that opcode into a register read. The 6507
  // always fetches an immediate operand at A+1, right after A, so this fires only
  // then: lda_next is A+1 (13-bit), or 0xFFFF when not armed.
  uint16_t lda_next;
  uint32_t callfn_arm;     // CALLFUNCTION 254/255 seen (diagnostic; should stay 0)
} dpcp_state;

DPCP_INLINE uint32_t dpcp_rand_next(uint32_t r) {
  return ((r & (1u << 10)) ? 0x10adab1eu : 0u) ^ ((r >> 11) | (r << 21));
}

DPCP_INLINE uint32_t dpcp_rand_prev(uint32_t r) {
  return (r & (1u << 31)) ? (((0x10adab1eu ^ r) << 11) | ((0x10adab1eu ^ r) >> 21))
                          : ((r << 11) | (r >> 21));
}

// Is the first word a known "stable" driver (DFxFRACLOW clears the low byte)?
// Stella tells by the MD5 of the 3 KB driver (5f80b5a5, 8dd73b44). Measured over
// the library's 12 driver versions: these two share the words below at 0x908 and
// no other version has any of them, so three compares stand in for an MD5.
static inline int dpcp_driver_is_stable(const uint8_t *drv, uint32_t drv_len) {
  const uint32_t off = 0x908;
  if (drv_len < off + 12) return 0;
  return drv[off + 0] == 0x40 && drv[off + 1] == 0x6D && drv[off + 2] == 0x8D && drv[off + 3] == 0xE2 &&
         drv[off + 4] == 0x00 && drv[off + 5] == 0x80 && drv[off + 6] == 0x85 && drv[off + 7] == 0xE5 &&
         drv[off + 8] == 0xD1 && drv[off + 9] == 0xF0 && drv[off + 10] == 0x21 && drv[off + 11] == 0xE3;
}

// The window flag of fetcher i, as Stella computes it at the start of a read.
DPCP_INLINE uint8_t dpcp_flag(const dpcp_state *st, uint32_t i) {
  return (((st->top[i] - (st->ctr[i] & 0xFF)) & 0xFF) > ((st->top[i] - st->bot[i]) & 0xFF)) ? 0xFF : 0;
}

// The display address voice k plays from now.
DPCP_INLINE uint32_t dpcp_sample_addr(const dpcp_state *st, uint32_t k) {
  return ((uint32_t)st->wave[k] << 5) + (st->mctr[k] >> 27);
}

DPCP_INLINE uint8_t dpcp_amplitude(const dpcp_state *st) {
  return (uint8_t)(st->disp[dpcp_sample_addr(st, 0)] + st->disp[dpcp_sample_addr(st, 1)] +
                   st->disp[dpcp_sample_addr(st, 2)]);
}

// Pointer counts (0.81): cnt[a] is how many of the 19 display pointers - eight data
// pointers, eight fractional pointers (frac >> 8), three music sample addresses -
// stand at a. With it a display write knows from one byte whether anything besides
// the writing fetcher reads what it wrote; scanning the pointers at every write took
// 272 cycles on the RP2040, more than a 6507 cycle. Every pointer moves through
// these three functions.
DPCP_INLINE void dpcp_ptr_move(dpcp_state *st, uint32_t from, uint32_t to) {
  if (DPCP_BROKEN(8)) return;
  st->cnt[from]--;
  st->cnt[to]++;
}

DPCP_INLINE void dpcp_set_ctr(dpcp_state *st, uint32_t i, uint32_t c) {
  dpcp_ptr_move(st, st->ctr[i], c);
  st->ctr[i] = (uint16_t)c;
}

DPCP_INLINE void dpcp_set_frac(dpcp_state *st, uint32_t i, uint32_t f) {
  dpcp_ptr_move(st, st->frac[i] >> 8, f >> 8);
  st->frac[i] = f;
}

// AMPLITUDE (rv[5]) is a sum of three samples. It depends only on the music
// counters, the waveform numbers and the display data, so it is recomputed wherever
// one of those changes - a music tick, WAVEFORMx, a display write at a sample address
// (DFxPUSH, DFxWRITE; in stage 2 the copy functions). That is the same value Stella
// computes at the read, which the host test checks byte for byte.
DPCP_INLINE void dpcp_amp_update(dpcp_state *st) {
  st->rv[5] = dpcp_amplitude(st);
}

// The read registers of fetcher i that follow its data pointer and window:
// DFxDATA, DFxDATAW and (fetchers 0-3) DFxFLAG.
DPCP_INLINE void dpcp_rv_fetcher(dpcp_state *st, uint32_t i) {
  const uint8_t d = st->disp[st->ctr[i]];
  const uint8_t fl = dpcp_flag(st, i);
  st->rv[0x08 + i] = d;
  st->rv[0x10 + i] = d & fl;
  if (i < 4) st->rv[0x20 + i] = fl;
}

// DFxFRACDATA of fetcher i.
DPCP_INLINE void dpcp_rv_frac(dpcp_state *st, uint32_t i) {
  st->rv[0x18 + i] = st->disp[st->frac[i] >> 8];
}

// RANDOM0NEXT, RANDOM0PRIOR (both: the byte after the step) and RANDOM1-3.
DPCP_INLINE void dpcp_rv_rand(dpcp_state *st) {
  const uint32_t r = st->rnd;
  st->rv[0] = (uint8_t)dpcp_rand_next(r);
  st->rv[1] = (uint8_t)dpcp_rand_prev(r);
  st->rv[2] = (uint8_t)(r >> 8);
  st->rv[3] = (uint8_t)(r >> 16);
  st->rv[4] = (uint8_t)(r >> 24);
}

// Display data byte a was written and cnt[a] says another pointer stands there:
// every fetcher pointing at it now reads something else, and so may AMPLITUDE. The
// slow path - the writes that need it are rare (4 of the 22 stage-1 images, at most
// 612 times in 1200 frames).
DPCP_INLINE void dpcp_disp_written(dpcp_state *st, uint32_t a) {
  if (DPCP_BROKEN(6)) { dpcp_amp_update(st); return; }
  for (uint32_t j = 0; j < 8; j++) {
    if (st->ctr[j] == a) dpcp_rv_fetcher(st, j);
    if ((st->frac[j] >> 8) == a) dpcp_rv_frac(st, j);
  }
  dpcp_amp_update(st);
}

// image/len: the file as loaded (32 KB with the driver; 29 KB without, as Stella
// accepts - then there is no driver to look at and the mask stays "jitter").
// ram: 8 KB of Harmony RAM plus 4 KB for cnt (DPCP_RAM_SIZE). Same reset as
// Stella's setInitialState().
static inline void dpcp_init(dpcp_state *st, const uint8_t *image, uint32_t len, uint8_t *ram) {
  const uint32_t drv = len > 29u * 1024u ? len - 29u * 1024u : 0;   // 3072 for a 32 KB file
  memset(st, 0, sizeof(*st));
  st->prog = image + drv;
  st->disp = ram + DPCP_RAM_DISPLAY;
  st->freq = ram + DPCP_RAM_FREQ;
  st->cnt = ram + DPCP_RAM_CNT;
  memset(ram, 0, DPCP_RAM_SIZE);
  memcpy(ram + DPCP_RAM_DISPLAY, st->prog + 24u * 1024u, 5u * 1024u);   // display + frequencies
  st->bank = 5u * 4096u;                                                // DPC+ starts in bank 5
  st->rnd = DPCP_RANDOM_INIT;
  st->frac_mask = (drv && dpcp_driver_is_stable(image, drv)) ? DPCP_FRAC_STABLE : DPCP_FRAC_JITTER;
  st->lda_next = 0xFFFF;
  for (uint32_t i = 0; i < 8; i++) { st->cnt[st->ctr[i]]++; st->cnt[st->frac[i] >> 8]++; }
  for (uint32_t k = 0; k < 3; k++) st->cnt[dpcp_sample_addr(st, k)]++;
  for (uint32_t i = 0; i < 8; i++) { dpcp_rv_fetcher(st, i); dpcp_rv_frac(st, i); }
  dpcp_rv_rand(st);
  dpcp_amp_update(st);                 // rv[6], rv[7], rv[0x24-0x27] stay 0
}

// A read of register r ($00-$27), computed from the state - what 0.80 did on the
// bus. Now only the host test calls it, to check rv[] (dpcp_read_value).
DPCP_INLINE uint8_t dpcp_read_compute(const dpcp_state *st, uint32_t r) {
  const uint32_t f = r >> 3, i = r & 7;
  if (f == 1) return st->disp[st->ctr[i]];                        // DFxDATA
  if (f == 3) return st->disp[st->frac[i] >> 8];                  // DFxFRACDATA
  if (f == 2) return st->disp[st->ctr[i]] & dpcp_flag(st, i);     // DFxDATAW
  if (f == 0) {
    if (i == 5) return dpcp_amplitude(st);                        // AMPLITUDE
    if (i == 0) return (uint8_t)dpcp_rand_next(st->rnd);          // RANDOM0NEXT: advance, then read
    if (i == 1) return (uint8_t)dpcp_rand_prev(st->rnd);          // RANDOM0PRIOR: step back, then read
    if (i == 2) return (uint8_t)(st->rnd >> 8);
    if (i == 3) return (uint8_t)(st->rnd >> 16);
    if (i == 4) return (uint8_t)(st->rnd >> 24);
    return 0;
  }
  return (f == 4 && i < 4) ? dpcp_flag(st, i) : 0;                // DFxFLAG
}

// A read of register r ($00-$27): the byte for the bus. No state changes here, so
// the caller can put it on the bus first and call dpcp_read_post() afterwards.
DPCP_INLINE uint8_t dpcp_read_value(const dpcp_state *st, uint32_t r) {
  return st->rv[r];
}

// What the read of register r does once its byte is on the bus.
DPCP_INLINE void dpcp_read_post(dpcp_state *st, uint32_t r) {
  const uint32_t i = r & 7;
  switch (r >> 3) {
    case 0:
      if (i == 0) { st->rnd = dpcp_rand_next(st->rnd); dpcp_rv_rand(st); }
      else if (i == 1) { st->rnd = dpcp_rand_prev(st->rnd); dpcp_rv_rand(st); }
      break;
    case 1:
    case 2:
      dpcp_set_ctr(st, i, (st->ctr[i] + 1) & 0x0FFF);
      dpcp_rv_fetcher(st, i);
      break;
    case 3:
      dpcp_set_frac(st, i, (st->frac[i] + st->frac_inc[i]) & 0x0FFFFF);
      dpcp_rv_frac(st, i);
      break;
    default:
      break;
  }
}

// A write of value v to register r ($28-$7F). The three that move a data pointer
// come first, in the order of how often the stage-1 images write them (DFxLOW 47k,
// DFxHI 37k, DFxWRITE 35k of 220k writes in 1200 frames each): the bus loop is
// built without jump tables, and as a switch GCC tested DFxWRITE last, 31 cycles
// in on the RP2040.
DPCP_INLINE void dpcp_write(dpcp_state *st, uint32_t r, uint8_t v) {
  const uint32_t i = r & 7;
  const uint32_t f = (r - 0x28) >> 3;
  if (f == 0x5) {                                                 // DFxLOW
    dpcp_set_ctr(st, i, (st->ctr[i] & 0x0F00) | v);
  } else if (f == 0x8) {                                          // DFxHI
    dpcp_set_ctr(st, i, (((uint32_t)v & 0x0F) << 8) | (st->ctr[i] & 0x00FF));
  } else if (f == 0xA) {                                          // DFxWRITE
    const uint32_t a = st->ctr[i];
    st->disp[a] = v;
    dpcp_set_ctr(st, i, (a + 1) & 0x0FFF);
    if (st->cnt[a]) dpcp_disp_written(st, a);                     // the others at a
  } else {
    switch (f) {
      case 0x0:                                                   // DFxFRACLOW
        dpcp_set_frac(st, i, (st->frac[i] & st->frac_mask) | ((uint32_t)v << 8));
        dpcp_rv_frac(st, i);
        break;
      case 0x1:                                                   // DFxFRACHI
        dpcp_set_frac(st, i, (((uint32_t)v & 0x0F) << 16) | (st->frac[i] & 0x00FFFF));
        dpcp_rv_frac(st, i);
        break;
      case 0x2:                                                   // DFxFRACINC
        st->frac_inc[i] = v;
        st->frac[i] &= 0x0FFF00;                                  // pointer unchanged
        break;
      case 0x3: st->top[i] = v; if (!DPCP_BROKEN(7)) dpcp_rv_fetcher(st, i); break;   // DFxTOP
      case 0x4: st->bot[i] = v; if (!DPCP_BROKEN(7)) dpcp_rv_fetcher(st, i); break;   // DFxBOT
      case 0x6:
        switch (i) {
          case 0: st->fastfetch = (v == 0); break;                // FASTFETCH
          case 1: if (st->param_ptr < 8) st->param[st->param_ptr++] = v; break;   // PARAMETER
          case 2:                                                 // CALLFUNCTION
            if (v == 0) st->param_ptr = 0;
            else if (v >= 254) st->callfn_arm++;
            break;
          case 5: case 6: case 7: {                               // WAVEFORM0-2
            const uint32_t old = dpcp_sample_addr(st, i - 5);
            st->wave[i - 5] = v & 0x7F;
            dpcp_ptr_move(st, old, dpcp_sample_addr(st, i - 5));
            dpcp_amp_update(st);
            break;
          }
          default: break;
        }
        break;
      case 0x7: {                                                 // DFxPUSH
        const uint32_t a = (st->ctr[i] - 1) & 0x0FFF;
        dpcp_set_ctr(st, i, a);
        st->disp[a] = v;
        if (st->cnt[a] > 1) dpcp_disp_written(st, a);             // fetcher i included
        else dpcp_rv_fetcher(st, i);
        break;
      }
      case 0x9:
        switch (i) {
          case 0: st->rnd = DPCP_RANDOM_INIT; dpcp_rv_rand(st); break;            // RRESET
          case 1: st->rnd = (st->rnd & 0xFFFFFF00u) | v; dpcp_rv_rand(st); break; // RWRITE0-3
          case 2: st->rnd = (st->rnd & 0xFFFF00FFu) | ((uint32_t)v << 8); dpcp_rv_rand(st); break;
          case 3: st->rnd = (st->rnd & 0xFF00FFFFu) | ((uint32_t)v << 16); dpcp_rv_rand(st); break;
          case 4: st->rnd = (st->rnd & 0x00FFFFFFu) | ((uint32_t)v << 24); dpcp_rv_rand(st); break;
          default: {                                              // NOTE0-2
            const uint8_t *fq = st->freq + ((uint32_t)v << 2);
            st->mfreq[i - 5] = (uint32_t)fq[0] | (uint32_t)fq[1] << 8 | (uint32_t)fq[2] << 16 | (uint32_t)fq[3] << 24;
            break;
          }
        }
        break;
      default:
        break;
    }
    return;
  }
  dpcp_rv_fetcher(st, i);                                         // i's pointer moved
}

// Bank switch on an access to $1FF6-$1FFB (read or write alike).
DPCP_INLINE void dpcp_hotspot(dpcp_state *st, uint32_t off) {
  if (off >= 0x0FF6 && off <= 0x0FFB) st->bank = (off - 0x0FF6) << 12;
}

// The music generator advanced by n ticks of its 20 kHz oscillator.
DPCP_INLINE void dpcp_music(dpcp_state *st, uint32_t n) {
  for (uint32_t k = 0; k < 3; k++) {
    const uint32_t old = dpcp_sample_addr(st, k);
    st->mctr[k] += st->mfreq[k] * n;
    dpcp_ptr_move(st, old, dpcp_sample_addr(st, k));
  }
  dpcp_amp_update(st);
}

#endif
