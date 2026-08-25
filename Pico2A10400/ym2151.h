// ym2151.h - Yamaha YM2151 (OPM) FM synthesis for Pico2A10400
//
// !!! THIS BOARD HAS NO AUDIO LINE CONNECTED !!!
// Same situation as in pokey.h: Pico2A10400 uses the standard Raspberry Pi Pico
// footprint, which does not expose GP29, so the ExtAudio net from connector pin 18
// ends in mid-air. The synthesis does run, there is just nothing to output it to.
//
// The code is carried over here in full so both sketches stay in sync (CLAUDE.md),
// and the BUS-SIDE PART MATTERS here too: without a response on $0460/$0461 a YM
// cartridge hangs in its chip-detection loop instead of running silently.
//
// WHAT THIS IS FOR
// ----------------
// 45 files in this library declare a YM2151 in their a78 header (byte53 bit 3,
// "ym2151 at $460/$461"): 40 music demos plus the games 1942, Wonder Boy,
// Pac-Man Collection 40th Anniversary and Block'Em Sock'Em.  The chip lives in
// Atari's XM expansion module, but these carts carry it on the board, so the
// flashcart has to BE the chip.  Unlike the POKEY there is no useful "roughly
// right" FM: an approximation of 4-operator FM does not sound like a rough
// version of the instrument, it sounds like a different instrument.  This is
// therefore a full port of MAME's OPM core, not a simplification.
//
// SOURCE AND FIDELITY
// -------------------
// Ported from ORIG/MAME-A7800/src/devices/sound/ym2151.cpp - Jarek Burczynski's
// implementation, GPL-2.0+, the same code lineage the reference emulators in
// ORIG/ use.  The DSP is transcribed unchanged; what was replaced is only the
// MAME plumbing:
//   * init_tables() -> precomputed const tables (tools/make_ym2151_tables.py).
//     MAME builds them with pow()/log()/sin() into ~34KB of RAM; here they are
//     const, so they sit in FLASH and cost zero SRAM and no libm.
//   * tl_tab[13*2*256] (26KB) -> ym_tl(), which reconstructs the same value
//     from a 256-entry base table and a shift.  Same numbers, 512 bytes.
//   * emu_timer -> plain sample counters.  Timer A ticks every (1024-index)
//     samples and Timer B every (256-index)*16, which is EXACT at the native
//     rate, because the chip's sample rate is its clock/64 by construction.
//   * the IRQ and CT1/CT2 output pins are dropped: neither is wired to a 7800.
// tools/ym2151_selftest/ checks the port against a host build of MAME's own
// code, sample for sample.
//
// WHY IT FITS ON THE PICO
// -----------------------
// Native rate is clock/64 = 3579545/64 = 55930 Hz, i.e. 4470 cycles per sample
// at 250MHz.  32 operators cost roughly 1500-2500 of those, and operators whose
// envelope is below ENV_QUIET are skipped entirely, which is most of them in
// real music.  Core 0 is idle for the whole life of a game (see loop()), so the
// budget is the whole core.
//
// The tables are deliberately left in FLASH rather than pulled into RAM.  Core 1
// executes its bus loop from RAM and reads rom_table from RAM; flash is a
// different bus slave reached through the XIP cache, so running the synthesis
// out of flash keeps core 0 OFF core 1's SRAM ports.  The per-sample working
// set is ym_sin (2KB) + ym_tl_base (512B) + the code, which fits the 16KB XIP
// cache; ym_freq (33KB) is touched only when a register write changes a pitch.
//
// AUDIO PATH: none on this board - see the warning above.  On PicoA10400 it is
// cartridge pin 18 "AUD IN" -> GPIO 29, PWM, no passive conditioning on the PCB.

#ifndef YM2151_H
#define YM2151_H

#include <string.h>
#ifndef YM_HOST_TEST
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#endif
#include "ym2151_tables.h"

// ---------------------------------------------------------------------------
// Bus-side state.  Written by core 1 from the cartridge bus, read by core 0.
// ---------------------------------------------------------------------------

// Set by identify_cartridge() when the a78 header declares the chip.  Selects
// the YM branch inside the shared $04xx window service, and makes loop() hand
// core 0 to ym_run() instead of pokey_run().
volatile uint8_t  ym_enabled = 0;

// Raised by core 1 once it has finished setting the emulation clock and voltage
// for this cart. Core 0 reaches ym_run() through loop() as soon as newgame is
// set, which can be BEFORE core 1 gets to set_sys_clock_khz() - and unlike every
// earlier use of that call, the YM path makes it a real PLL reconfiguration
// rather than a re-set of the value already in force. Starting synthesis in the
// middle of that means configuring the PWM against a clock that is about to
// change underneath it. Cheap to just wait.
volatile uint8_t  emu_clock_ready = 0;

// Status register as the 6502 sees it.  Bit 0 = timer A overflow, bit 1 =
// timer B overflow, bit 7 = BUSY.  BUSY is held at 0 on purpose: every one of
// the 104 status polls in this library is "BIT $0461 / BMI *-3", i.e. wait
// while busy, so a chip that is never busy is the fast path and can never hang
// the caller.  We lose nothing by it - the write queue below absorbs writes at
// any rate the 6502 can produce them.
volatile uint8_t  ym_status = 0;

// Register writes cross cores through a single-producer/single-consumer ring.
// A flat shadow register file (the trick pokey.h uses) is NOT usable here:
// YM2151 register $08 is KEY ON/OFF, an event, and collapsing two writes of $08
// into "the last value" silently drops notes.  Order has to survive, so the
// events queue.
//
// Depth: core 0 drains the whole ring every sample (17.9us) and a 6502 needs
// ~8 cycles (4.5us) per register, so four entries would do; 256 is free.
#define YM_QUEUE_SIZE 256                       // must stay a power of two
volatile uint16_t ym_queue[YM_QUEUE_SIZE];      // (reg << 8) | value
volatile uint32_t ym_q_head = 0;                // core 1 writes
volatile uint32_t ym_q_tail = 0;                // core 0 writes
volatile uint32_t ym_q_drops = 0;               // diagnostics only

static uint8_t ym_addr_latch = 0;               // $0460 latch, core 1 only

// Capture one completed bus cycle in the $0460/$0461 window.  'data' must
// already be the END-OF-CYCLE byte: a 6502 does not drive the data lines until
// the second half of its cycle, so the callers sample it the same way the bank
// and RAM write paths do (bounded spin while the address is unchanged).  That
// also guarantees exactly ONE call per bus cycle, which is what lets this push
// into a queue at all.
static inline __attribute__((always_inline))
void ym_bus_write(uint32_t addr, uint8_t data) {
  if (addr & 1) {                               // $0461 - data register
    uint32_t h = ym_q_head;
    if ((uint32_t)(h - ym_q_tail) < YM_QUEUE_SIZE) {
      ym_queue[h & (YM_QUEUE_SIZE - 1)] = (uint16_t)(((uint16_t)ym_addr_latch << 8) | data);
      // Publish the entry before the index that makes it visible. DMB is
      // ARMv6-M (and ARMv8-M) baseline, so this needs no CMSIS - same reason
      // the emulate loops write "cpsid i" by hand.
      __asm volatile ("dmb" ::: "memory");
      ym_q_head = h + 1;
    } else {
      ym_q_drops++;                             // core 0 fell behind; drop, never corrupt
    }
  } else {                                      // $0460 - address register
    ym_addr_latch = data;
  }
}

// A0=0 reads 0xFF on a real chip (MAME ym2151.cpp:1683 "confirmed on a real
// YM2151"); A0=1 reads the status register.
static inline __attribute__((always_inline))
uint8_t ym_bus_read(uint32_t addr) {
  return (addr & 1) ? (uint8_t)ym_status : 0xFF;
}

#ifndef YM_HOST_TEST
// The two sketches differ in how the address bus is wired: on PicoA10400 all of
// A0-A15 land inside BUS_PIN_MASK, on Pico2A10400 that mask covers A0-A14 only
// and A15 is gpio 26, so "has the address changed" has to compare BUS15_PIN_MASK
// there. Deriving it here keeps the two copies of this header identical.
#ifdef BUS15_PIN_MASK
#define YM_BUS_STABLE_MASK  BUS15_PIN_MASK
#else
#define YM_BUS_STABLE_MASK  BUS_PIN_MASK
#endif

// Service one bus cycle that landed in the $0460/$0461 window - non-blocking
// read, for the SuperGame-shaped loops that must never stop answering MARIA.
// Same contract as pokey_window_service().
//
// The write path DOES use the bounded end-of-cycle spin that pokey_window_service
// deliberately avoids, and that is not an oversight: a POKEY register file
// tolerates being written repeatedly with a settling value, because only the last
// store survives.  A YM2151 does not - $08 is KEY ON/OFF and the value has to be
// pushed into an ordered queue exactly once per bus cycle, which means the byte
// must already be final when it is pushed.  The spin is the same shape, and the
// same 64 iterations, as the bank-select and RAM write paths a few lines above in
// every one of these loops (Krok 19/20), and it exits the moment the address
// moves.  If 7800 titles ever glitch specifically with a YM cart loaded, this is
// the first thing to look at.
static inline __attribute__((always_inline))
void ym_window_service(uint32_t addr, uint8_t *rom_in_use) {
  uint32_t g = gpio_get_all();
  if (g & RW_PIN_MASK) {                        // read cycle
    sio_hw->gpio_out = (uint32_t)ym_bus_read(addr) << D0_PIN;
    if (!*rom_in_use) { SET_DATA_MODE_OUT; *rom_in_use = 1; }
  } else {                                      // write cycle
    if (*rom_in_use) { SET_DATA_MODE_IN; *rom_in_use = 0; }
    const uint32_t stable = g & YM_BUS_STABLE_MASK;
    uint32_t last = g, cur;
    for (uint32_t k = 0; k < 64; k++) {
      cur = gpio_get_all();
      if ((cur & YM_BUS_STABLE_MASK) != stable) break;
      last = cur;
    }
    ym_bus_write(addr, (uint8_t)((last >> D0_PIN) & 0xFF));
  }
}

// Same window, but shaped like emulate_normala78_pokey()'s own ROM path: drive,
// wait for the cycle to end, release.  That loop has no rom_in_use flag - it
// brackets every response with SET_DATA_MODE_OUT/IN - so a service routine that
// left the pin direction latched would desynchronise it.  28 of the 45 YM carts
// in this library are flat ROMs and take this path, and 32 of the 45 poll the
// status register ("BIT $0461 / BMI *-3"), so the read side is not optional here.
// The wait is BOUNDED for the reason Krok 18 recorded: an unbounded one hangs the
// cart outright if R/W is ever misread.
static inline __attribute__((always_inline))
void ym_window_service_blocking(uint32_t addr) {
  uint32_t g = gpio_get_all();
  const uint32_t stable = g & YM_BUS_STABLE_MASK;
  if (g & RW_PIN_MASK) {                        // read cycle
    sio_hw->gpio_out = (uint32_t)ym_bus_read(addr) << D0_PIN;
    SET_DATA_MODE_OUT;
    for (uint32_t k = 0; k < 256; k++)
      if ((gpio_get_all() & YM_BUS_STABLE_MASK) != stable) break;
    SET_DATA_MODE_IN;
  } else {                                      // write cycle
    uint32_t last = g, cur;
    for (uint32_t k = 0; k < 64; k++) {
      cur = gpio_get_all();
      if ((cur & YM_BUS_STABLE_MASK) != stable) break;
      last = cur;
    }
    ym_bus_write(addr, (uint8_t)((last >> D0_PIN) & 0xFF));
  }
}
#endif // YM_HOST_TEST

// ---------------------------------------------------------------------------
// The OPM core.  Namespaced because MAME's member names - status, test, noise,
// freq, connect, mem - are far too generic to drop into a 3900-line sketch.
// ---------------------------------------------------------------------------
// O3 for the whole DSP, and it has to be a scoped pragma rather than an attribute
// on one function. 2026-08-24: __attribute__((optimize("O2"))) was put on
// ym_run() alone, on the assumption that the synthesis was inlined into it. The
// linker map says otherwise - ym_run() is 276 bytes while ym2151::next_sample()
// is a separate 1600-byte symbol, and chan_calc/advance/advance_eg are folded
// into THAT. So the attribute covered the scheduling loop and essentially none of
// the arithmetic, which stayed at the sketch's default -Os. That is why the first
// "-O2" build only moved the needle slightly.
//
// push_options/pop_options rather than a bare #pragma GCC optimize: this is a
// header, and a bare one would silently re-optimize the whole rest of the sketch
// that follows the include - including the emulate_* loops, whose optimization
// levels were chosen deliberately and proven on hardware.
//
// Every function here gets the SAME level on purpose. GCC will decline to inline
// across mismatched optimize settings, so annotating only next_sample() risks
// un-inlining chan_calc() and coming out slower. If O3's code growth ever turns
// out to hurt the 16KB XIP cache more than the unrolling helps, "O2" here is the
// one-word fallback - YM_LOG.TXT measures the difference directly, since the
// benchmark always runs at the same 250MHz.
#pragma GCC push_options
#pragma GCC optimize("O3")

namespace ym2151 {

#define YM_FREQ_SH      16              /* 16.16 fixed point (frequency)   */
#define YM_EG_SH        16              /* 16.16 fixed point (EG timing)   */
#define YM_LFO_SH       10              /* 22.10 fixed point (LFO)         */
#define YM_FREQ_MASK    ((1 << YM_FREQ_SH) - 1)

#define YM_ENV_BITS     10
#define YM_ENV_LEN      (1 << YM_ENV_BITS)
#define YM_MAX_ATT_INDEX (YM_ENV_LEN - 1)       /* 1023 */
#define YM_MIN_ATT_INDEX (0)

#define YM_EG_ATT       4
#define YM_EG_DEC       3
#define YM_EG_SUS       2
#define YM_EG_REL       1
#define YM_EG_OFF       0

#define YM_RATE_STEPS   8
#define YM_TL_RES_LEN   256
#define YM_TL_TAB_LEN   (13 * 2 * YM_TL_RES_LEN)        /* 6656 */
#define YM_ENV_QUIET    (YM_TL_TAB_LEN >> 3)            /* 832  */
#define YM_SIN_MASK     ((1 << 10) - 1)

// tl_tab[] without the 26KB.  MAME lays the table out as
//     tl_tab[x*2 + s + i*2*TL_RES_LEN] = (s ? -1 : 1) * (base[x] >> i)
// so index p decomposes as i = p>>9, x = (p>>1)&0xff, s = p&1.
static inline int32_t tl(uint32_t p) {
  int32_t v = (int32_t)ym_tl_base[(p >> 1) & 0xFF] >> (p >> 9);
  return (p & 1) ? -v : v;
}

struct OP {
  uint32_t phase;               // accumulated operator phase
  uint32_t freq;                // operator frequency count
  int32_t  dt1;                 // current DT1 phase inc/decrement
  uint32_t mul;                 // frequency count multiply
  uint32_t dt1_i;               // DT1 index * 32
  uint32_t dt2;                 // current DT2 value

  int32_t *connect;             // operator output 'direction'
  int32_t *mem_connect;         // where to put the delayed sample (MEM)
  int32_t  mem_value;           // delayed sample (MEM) value

  // channel specific data; only operator 0 of each channel carries it
  uint32_t fb_shift;
  int32_t  fb_out_curr;
  int32_t  fb_out_prev;
  uint32_t kc;                  // channel KC (copied to all operators)
  uint32_t kc_i;                // just for speedup
  uint32_t pms;
  uint32_t ams;

  uint32_t AMmask;              // LFO AM enable mask
  uint32_t state;               // envelope state
  uint8_t  eg_sh_ar,  eg_sel_ar;
  uint32_t tl;                  // total attenuation level
  int32_t  volume;              // current envelope attenuation level
  uint8_t  eg_sh_d1r, eg_sel_d1r;
  uint32_t d1l;
  uint8_t  eg_sh_d2r, eg_sel_d2r;
  uint8_t  eg_sh_rr,  eg_sel_rr;
  uint32_t key;                 // 0 = last key was KEY OFF
  uint32_t ks, ar, d1r, d2r, rr;
};

static OP       oper[32];
static int32_t  chanout[8];
static int32_t  m2, c1, c2;     // phase modulation input for operators 2,3,4
static int32_t  mem;            // one sample delay memory
static uint32_t pan[16];
static uint8_t  connect_alg[8];

static uint32_t eg_cnt, eg_timer;
static const uint32_t eg_timer_add      = 1 << YM_EG_SH;
static const uint32_t eg_timer_overflow = 3 * (1 << YM_EG_SH);

static uint32_t lfo_phase, lfo_timer, lfo_overflow, lfo_counter, lfo_counter_add;
static const uint32_t lfo_timer_add = 1 << YM_LFO_SH;
static uint8_t  lfo_wsel, amd;
static int8_t   pmd;
static uint32_t lfa;
static int32_t  lfp;

static uint8_t  test, ct;
static uint8_t  noise;
static uint32_t noise_rng, noise_p, noise_f;

static uint8_t  csm_req, irq_enable, status;
static uint32_t timer_A_index, timer_B_index;
static uint32_t timer_A_count, timer_B_count;   // samples left until overflow
static uint8_t  timer_A_on,    timer_B_on;

// --- envelope key on/off ---------------------------------------------------

static void key_on(OP *o, uint32_t key_set) {
  if (!o->key) {
    o->phase = 0;
    o->state = YM_EG_ATT;
    o->volume += (~o->volume * (ym_eg_inc[o->eg_sel_ar + ((eg_cnt >> o->eg_sh_ar) & 7)])) >> 4;
    if (o->volume <= YM_MIN_ATT_INDEX) {
      o->volume = YM_MIN_ATT_INDEX;
      o->state  = YM_EG_DEC;
    }
  }
  o->key |= key_set;
}

static void key_off(OP *o, uint32_t key_set) {
  if (o->key) {
    o->key &= ~key_set;
    if (!o->key) {
      if (o->state > YM_EG_REL) o->state = YM_EG_REL;
    }
  }
}

static void envelope_KONKOFF(OP *o, int v) {
  static const uint8_t masks[4] = { 0x08, 0x20, 0x10, 0x40 };   // m1, m2, c1, c2
  for (int i = 0; i != 4; i++) {
    if (v & masks[i]) key_on(&o[i], 1);
    else              key_off(&o[i], 1);
  }
}

// --- algorithm routing -----------------------------------------------------

static void set_connect(OP *om1, int cha, int v) {
  OP *om2 = om1 + 1;
  OP *oc1 = om1 + 2;

  switch (v & 7) {
  case 0:   /* M1---C1---MEM---M2---C2---OUT */
    om1->connect = &c1;  oc1->connect = &mem; om2->connect = &c2;
    om1->mem_connect = &m2;  break;
  case 1:   /* M1------+-MEM---M2---C2---OUT ; C1-+ */
    om1->connect = &mem; oc1->connect = &mem; om2->connect = &c2;
    om1->mem_connect = &m2;  break;
  case 2:   /* M1-----------------+-C2---OUT ; C1---MEM---M2-+ */
    om1->connect = &c2;  oc1->connect = &mem; om2->connect = &c2;
    om1->mem_connect = &m2;  break;
  case 3:   /* M1---C1---MEM------+-C2---OUT ; M2-+ */
    om1->connect = &c1;  oc1->connect = &mem; om2->connect = &c2;
    om1->mem_connect = &c2;  break;
  case 4:   /* M1---C1-+-OUT ; M2---C2-+ */
    om1->connect = &c1;  oc1->connect = &chanout[cha]; om2->connect = &c2;
    om1->mem_connect = &mem; break;
  case 5:   /* +----C1----+ ; M1-+-MEM---M2-+-OUT ; +----C2----+ */
    om1->connect = nullptr;                             // special mark
    oc1->connect = &chanout[cha]; om2->connect = &chanout[cha];
    om1->mem_connect = &m2;  break;
  case 6:   /* M1---C1-+ ; M2-+-OUT ; C2-+ */
    om1->connect = &c1;  oc1->connect = &chanout[cha]; om2->connect = &chanout[cha];
    om1->mem_connect = &mem; break;
  case 7:   /* M1-+ C1-+ M2-+ C2-+-OUT */
    om1->connect = &chanout[cha]; oc1->connect = &chanout[cha];
    om2->connect = &chanout[cha]; om1->mem_connect = &mem; break;
  }
}

// --- envelope rate refresh -------------------------------------------------

static inline void refresh_EG_one(OP *o, uint32_t kc) {
  uint32_t v = kc >> o->ks;
  if ((o->ar + v) < 32 + 62) {
    o->eg_sh_ar  = ym_eg_rate_shift [o->ar + v];
    o->eg_sel_ar = ym_eg_rate_select[o->ar + v];
  } else {
    o->eg_sh_ar  = 0;
    o->eg_sel_ar = 17 * YM_RATE_STEPS;
  }
  o->eg_sh_d1r  = ym_eg_rate_shift [o->d1r + v];
  o->eg_sel_d1r = ym_eg_rate_select[o->d1r + v];
  o->eg_sh_d2r  = ym_eg_rate_shift [o->d2r + v];
  o->eg_sel_d2r = ym_eg_rate_select[o->d2r + v];
  o->eg_sh_rr   = ym_eg_rate_shift [o->rr  + v];
  o->eg_sel_rr  = ym_eg_rate_select[o->rr  + v];
}

static void refresh_EG(OP *o) {
  uint32_t kc = o->kc;                  // all four share the channel's KC
  refresh_EG_one(o + 0, kc);
  refresh_EG_one(o + 1, kc);
  refresh_EG_one(o + 2, kc);
  refresh_EG_one(o + 3, kc);
}

// --- register write --------------------------------------------------------

void write_reg(int r, int v) {
  OP *op = &oper[(r & 0x07) * 4 + ((r & 0x18) >> 3)];
  r &= 0xff;
  v &= 0xff;

  switch (r & 0xe0) {
  case 0x00:
    switch (r) {
    case 0x01:                                  // LFO reset (bit 1), test register
      test = v;
      if (v & 2) lfo_phase = 0;
      break;

    case 0x08:                                  // KEY ON / KEY OFF
      envelope_KONKOFF(&oper[(v & 7) * 4], v);
      break;

    case 0x0f:                                  // noise mode enable, noise period
      noise   = v;
      noise_f = ym_noise_tab[v & 0x1f];
      break;

    case 0x10: timer_A_index = (timer_A_index & 0x003) | (v << 2); break;
    case 0x11: timer_A_index = (timer_A_index & 0x3fc) | (v & 3);  break;
    case 0x12: timer_B_index = v;                                  break;

    case 0x14:                                  // CSM, flag reset, enable, start/stop
      irq_enable = v;                           // bit3 timer B, bit2 timer A, bit7 CSM
      if (v & 0x10) status &= ~1;               // reset timer A flag
      if (v & 0x20) status &= ~2;               // reset timer B flag
      // MAME loads a timer only on the off->on edge (timer->enable() returns the
      // previous state); a repeated write with the bit already set must NOT
      // restart it, or a driver that rewrites $14 every frame never overflows.
      if (v & 0x02) {
        if (!timer_B_on) { timer_B_on = 1; timer_B_count = (256 - timer_B_index) * 16; }
      } else timer_B_on = 0;
      if (v & 0x01) {
        if (!timer_A_on) { timer_A_on = 1; timer_A_count = 1024 - timer_A_index; }
      } else timer_A_on = 0;
      ym_status = status;
      break;

    case 0x18:                                  // LFO frequency
      lfo_overflow    = (1 << ((15 - (v >> 4)) + 3)) * (1 << YM_LFO_SH);
      lfo_counter_add = 0x10 + (v & 0x0f);
      break;

    case 0x19:                                  // PMD (bit7=1) or AMD (bit7=0)
      if (v & 0x80) pmd = v & 0x7f;
      else          amd = v & 0x7f;
      break;

    case 0x1b:                                  // CT2, CT1, LFO waveform
      ct       = v >> 6;                        // CT1/CT2 pins do not exist here
      lfo_wsel = v & 3;
      break;
    }
    break;

  case 0x20:
    op = &oper[(r & 7) * 4];
    switch (r & 0x18) {
    case 0x00:                                  // RL enable, feedback, connection
      op->fb_shift = ((v >> 3) & 7) ? ((v >> 3) & 7) + 6 : 0;
      pan[(r & 7) * 2    ] = (v & 0x40) ? ~0u : 0u;
      pan[(r & 7) * 2 + 1] = (v & 0x80) ? ~0u : 0u;
      connect_alg[r & 7] = v & 7;
      set_connect(op, r & 7, v & 7);
      break;

    case 0x08:                                  // Key Code
      v &= 0x7f;
      if ((uint32_t)v != op->kc) {
        uint32_t kc, kc_channel;
        kc_channel  = (v - (v >> 2)) * 64;
        kc_channel += 768;
        kc_channel |= (op->kc_i & 63);
        for (int i = 0; i < 4; i++) { op[i].kc = v; op[i].kc_i = kc_channel; }
        kc = v >> 2;
        for (int i = 0; i < 4; i++) {
          op[i].dt1  = ym_dt1_freq[op[i].dt1_i + kc];
          op[i].freq = ((ym_freq[kc_channel + op[i].dt2] + op[i].dt1) * op[i].mul) >> 1;
        }
        refresh_EG(op);
      }
      break;

    case 0x10:                                  // Key Fraction
      v >>= 2;
      if ((uint32_t)v != (op->kc_i & 63)) {
        uint32_t kc_channel = (uint32_t)v | (op->kc_i & ~63u);
        for (int i = 0; i < 4; i++) {
          op[i].kc_i = kc_channel;
          op[i].freq = ((ym_freq[kc_channel + op[i].dt2] + op[i].dt1) * op[i].mul) >> 1;
        }
      }
      break;

    case 0x18:                                  // PMS, AMS
      op->pms = (v >> 4) & 7;
      op->ams = (v & 3);
      break;
    }
    break;

  case 0x40: {                                  // DT1, MUL
    uint32_t olddt1_i = op->dt1_i;
    uint32_t oldmul   = op->mul;
    op->dt1_i = (v & 0x70) << 1;
    op->mul   = (v & 0x0f) ? (v & 0x0f) << 1 : 1;
    if (olddt1_i != op->dt1_i)
      op->dt1 = ym_dt1_freq[op->dt1_i + (op->kc >> 2)];
    if ((olddt1_i != op->dt1_i) || (oldmul != op->mul))
      op->freq = ((ym_freq[op->kc_i + op->dt2] + op->dt1) * op->mul) >> 1;
    break;
  }

  case 0x60:                                    // TL
    op->tl = (v & 0x7f) << (YM_ENV_BITS - 7);
    break;

  case 0x80: {                                  // KS, AR
    uint32_t oldks = op->ks;
    uint32_t oldar = op->ar;
    op->ks = 5 - (v >> 6);
    op->ar = (v & 0x1f) ? 32 + ((v & 0x1f) << 1) : 0;
    if ((op->ar != oldar) || (op->ks != oldks)) {
      if ((op->ar + (op->kc >> op->ks)) < 32 + 62) {
        op->eg_sh_ar  = ym_eg_rate_shift [op->ar + (op->kc >> op->ks)];
        op->eg_sel_ar = ym_eg_rate_select[op->ar + (op->kc >> op->ks)];
      } else {
        op->eg_sh_ar  = 0;
        op->eg_sel_ar = 17 * YM_RATE_STEPS;
      }
    }
    if (op->ks != oldks) {
      op->eg_sh_d1r  = ym_eg_rate_shift [op->d1r + (op->kc >> op->ks)];
      op->eg_sel_d1r = ym_eg_rate_select[op->d1r + (op->kc >> op->ks)];
      op->eg_sh_d2r  = ym_eg_rate_shift [op->d2r + (op->kc >> op->ks)];
      op->eg_sel_d2r = ym_eg_rate_select[op->d2r + (op->kc >> op->ks)];
      op->eg_sh_rr   = ym_eg_rate_shift [op->rr  + (op->kc >> op->ks)];
      op->eg_sel_rr  = ym_eg_rate_select[op->rr  + (op->kc >> op->ks)];
    }
    break;
  }

  case 0xa0:                                    // LFO AM enable, D1R
    op->AMmask     = (v & 0x80) ? ~0u : 0u;
    op->d1r        = (v & 0x1f) ? 32 + ((v & 0x1f) << 1) : 0;
    op->eg_sh_d1r  = ym_eg_rate_shift [op->d1r + (op->kc >> op->ks)];
    op->eg_sel_d1r = ym_eg_rate_select[op->d1r + (op->kc >> op->ks)];
    break;

  case 0xc0: {                                  // DT2, D2R
    uint32_t olddt2 = op->dt2;
    op->dt2 = ym_dt2_tab[v >> 6];
    if (op->dt2 != olddt2)
      op->freq = ((ym_freq[op->kc_i + op->dt2] + op->dt1) * op->mul) >> 1;
    op->d2r        = (v & 0x1f) ? 32 + ((v & 0x1f) << 1) : 0;
    op->eg_sh_d2r  = ym_eg_rate_shift [op->d2r + (op->kc >> op->ks)];
    op->eg_sel_d2r = ym_eg_rate_select[op->d2r + (op->kc >> op->ks)];
    break;
  }

  case 0xe0:                                    // D1L, RR
    op->d1l        = ym_d1l_tab[v >> 4];
    op->rr         = 34 + ((v & 0x0f) << 2);
    op->eg_sh_rr   = ym_eg_rate_shift [op->rr + (op->kc >> op->ks)];
    op->eg_sel_rr  = ym_eg_rate_select[op->rr + (op->kc >> op->ks)];
    break;
  }
}

// --- operator output -------------------------------------------------------

static inline int32_t op_calc(OP *o, uint32_t env, int32_t pm) {
  uint32_t p = (env << 3) +
      ym_sin[(((int32_t)((o->phase & ~YM_FREQ_MASK) + (pm << 15))) >> YM_FREQ_SH) & YM_SIN_MASK];
  if (p >= YM_TL_TAB_LEN) return 0;
  return tl(p);
}

static inline int32_t op_calc1(OP *o, uint32_t env, int32_t pm) {
  int32_t  i = (int32_t)(o->phase & ~YM_FREQ_MASK) + pm;
  uint32_t p = (env << 3) + ym_sin[(i >> YM_FREQ_SH) & YM_SIN_MASK];
  if (p >= YM_TL_TAB_LEN) return 0;
  return tl(p);
}

#define YM_VOL(o)  ((o)->tl + (uint32_t)(o)->volume + (AM & (o)->AMmask))

static void chan_calc(unsigned int chan) {
  OP *op;
  unsigned int env;
  uint32_t AM = 0;

  m2 = c1 = c2 = mem = 0;
  op = &oper[chan * 4];                         // M1
  *op->mem_connect = op->mem_value;             // restore delayed sample (MEM)

  if (op->ams) AM = lfa << (op->ams - 1);
  env = YM_VOL(op);
  {
    int32_t out = op->fb_out_prev + op->fb_out_curr;
    op->fb_out_prev = op->fb_out_curr;
    if (!op->connect) mem = c1 = c2 = op->fb_out_prev;      // algorithm 5
    else              *op->connect = op->fb_out_prev;
    op->fb_out_curr = 0;
    if (env < YM_ENV_QUIET) {
      if (!op->fb_shift) out = 0;
      op->fb_out_curr = op_calc1(op, env, out << op->fb_shift);
    }
  }

  env = YM_VOL(op + 1);                         // M2
  if (env < YM_ENV_QUIET) *(op + 1)->connect += op_calc(op + 1, env, m2);

  env = YM_VOL(op + 2);                         // C1
  if (env < YM_ENV_QUIET) *(op + 2)->connect += op_calc(op + 2, env, c1);

  env = YM_VOL(op + 3);                         // C2
  if (chan == 7 && (noise & 0x80)) {
    uint32_t noiseout = 0;
    if (env < 0x3ff) noiseout = (env ^ 0x3ff) * 2;  // YM2151 noise is -2044..2040
    chanout[7] += (noise_rng & 0x10000) ? (int32_t)noiseout : -(int32_t)noiseout;
  } else {
    if (env < YM_ENV_QUIET) chanout[chan] += op_calc(op + 3, env, c2);
  }

  op->mem_value = mem;                          // M1
}

// --- envelope generator ----------------------------------------------------

static void advance_eg() {
  eg_timer += eg_timer_add;

  while (eg_timer >= eg_timer_overflow) {
    eg_timer -= eg_timer_overflow;
    eg_cnt++;

    OP *op = &oper[0];
    unsigned int i = 32;
    do {
      switch (op->state) {
      case YM_EG_ATT:
        if (!(eg_cnt & ((1u << op->eg_sh_ar) - 1))) {
          op->volume += (~op->volume *
              (ym_eg_inc[op->eg_sel_ar + ((eg_cnt >> op->eg_sh_ar) & 7)])) >> 4;
          if (op->volume <= YM_MIN_ATT_INDEX) {
            op->volume = YM_MIN_ATT_INDEX;
            op->state  = YM_EG_DEC;
          }
        }
        break;
      case YM_EG_DEC:
        if (!(eg_cnt & ((1u << op->eg_sh_d1r) - 1))) {
          op->volume += ym_eg_inc[op->eg_sel_d1r + ((eg_cnt >> op->eg_sh_d1r) & 7)];
          if ((uint32_t)op->volume >= op->d1l) op->state = YM_EG_SUS;
        }
        break;
      case YM_EG_SUS:
        if (!(eg_cnt & ((1u << op->eg_sh_d2r) - 1))) {
          op->volume += ym_eg_inc[op->eg_sel_d2r + ((eg_cnt >> op->eg_sh_d2r) & 7)];
          if (op->volume >= YM_MAX_ATT_INDEX) {
            op->volume = YM_MAX_ATT_INDEX;
            op->state  = YM_EG_OFF;
          }
        }
        break;
      case YM_EG_REL:
        if (!(eg_cnt & ((1u << op->eg_sh_rr) - 1))) {
          op->volume += ym_eg_inc[op->eg_sel_rr + ((eg_cnt >> op->eg_sh_rr) & 7)];
          if (op->volume >= YM_MAX_ATT_INDEX) {
            op->volume = YM_MAX_ATT_INDEX;
            op->state  = YM_EG_OFF;
          }
        }
        break;
      }
      op++;
      i--;
    } while (i);
  }
}

// --- LFO, noise, phase generator, CSM --------------------------------------

static void advance() {
  OP *op;
  unsigned int i;
  int a, p;

  if (test & 2) {
    lfo_phase = 0;
  } else {
    lfo_timer += lfo_timer_add;
    if (lfo_timer >= lfo_overflow) {
      lfo_timer   -= lfo_overflow;
      lfo_counter += lfo_counter_add;
      lfo_phase   += (lfo_counter >> 4);
      lfo_phase   &= 255;
      lfo_counter &= 15;
    }
  }

  i = lfo_phase;
  switch (lfo_wsel) {
  case 0:                                       // saw
    a = 255 - i;
    p = (i < 128) ? (int)i : (int)i - 255;
    break;
  case 1:                                       // square
    if (i < 128) { a = 255; p =  128; }
    else         { a = 0;   p = -128; }
    break;
  case 2:                                       // triangle
    a = (i < 128) ? 255 - (int)(i * 2) : (int)(i * 2) - 256;
    if      (i < 64)  p = (int)i * 2;
    else if (i < 128) p = 255 - (int)i * 2;
    else if (i < 192) p = 256 - (int)i * 2;
    else              p = (int)i * 2 - 511;
    break;
  case 3:
  default:                                      // random (snapshot of a real chip)
    a = ym_lfo_noise[i];
    p = a - 128;
    break;
  }
  lfa = a * amd / 128;
  lfp = p * pmd / 128;

  // 17-bit noise shift register; bit16 is the output.
  noise_p += noise_f;
  i = (noise_p >> 16);
  noise_p &= 0xffff;
  while (i) {
    uint32_t j = ((noise_rng ^ (noise_rng >> 3)) & 1) ^ 1;
    noise_rng = (j << 16) | (noise_rng >> 1);
    i--;
  }

  // phase generator
  op = &oper[0];
  i  = 8;
  do {
    if (op->pms) {                              // LFO phase modulation on
      int32_t mod_ind = lfp;                    // -128..+127
      if (op->pms < 6) mod_ind >>= (6 - op->pms);
      else             mod_ind <<= (op->pms - 5);
      if (mod_ind) {
        uint32_t kc_channel = op->kc_i + mod_ind;
        for (int k = 0; k < 4; k++)
          op[k].phase += ((ym_freq[kc_channel + op[k].dt2] + op[k].dt1) * op[k].mul) >> 1;
      } else {
        for (int k = 0; k < 4; k++) op[k].phase += op[k].freq;
      }
    } else {
      for (int k = 0; k < 4; k++) op[k].phase += op[k].freq;
    }
    op += 4;
    i--;
  } while (i);

  // CSM is calculated AFTER the phase generator (verified on a real chip).
  if (csm_req) {
    if (csm_req == 2) {                         // KEY ON
      for (int k = 0; k < 32; k++) key_on(&oper[k], 2);
      csm_req = 1;
    } else {                                    // KEY OFF
      for (int k = 0; k < 32; k++) key_off(&oper[k], 2);
      csm_req = 0;
    }
  }
}

// --- timers ----------------------------------------------------------------
//
// At the native rate the chip's own dividers ARE sample counts: timer A ticks
// every 64*(1024-index) chip clocks and one sample is 64 chip clocks, so it is
// (1024-index) samples exactly.  Timer B is 1024*(256-index) clocks = 16 times
// that.  No approximation anywhere.
static inline void advance_timers() {
  if (timer_A_on && --timer_A_count == 0) {
    timer_A_count = 1024 - timer_A_index;
    if (irq_enable & 0x80) csm_req = 2;         // CSM key on/off sequence
    if (irq_enable & 0x04) status |= 1;
  }
  if (timer_B_on && --timer_B_count == 0) {
    timer_B_count = (256 - timer_B_index) * 16;
    if (irq_enable & 0x08) status |= 2;
  }
}

// --- reset -----------------------------------------------------------------

void reset() {
  for (int i = 0; i < 32; i++) {
    memset(&oper[i], 0, sizeof(OP));
    oper[i].volume = YM_MAX_ATT_INDEX;
    oper[i].kc_i   = 768;                       // min kc_i value
  }
  for (int i = 0; i < 8; i++)  chanout[i] = 0;
  for (int i = 0; i < 16; i++) pan[i] = 0;
  m2 = c1 = c2 = mem = 0;

  eg_timer = 0; eg_cnt = 0;
  lfo_timer = 0; lfo_counter = 0; lfo_phase = 0; lfo_wsel = 0;
  lfo_overflow = (1 << (15 + 3)) * (1 << YM_LFO_SH);
  lfo_counter_add = 0x10;
  pmd = 0; amd = 0; lfa = 0; lfp = 0;
  test = 0; ct = 0;

  irq_enable = 0;
  timer_A_on = timer_B_on = 0;
  timer_A_index = timer_B_index = 0;
  timer_A_count = timer_B_count = 0;

  noise = 0; noise_rng = 0; noise_p = 0; noise_f = ym_noise_tab[0];
  csm_req = 0; status = 0; ym_status = 0;

  write_reg(0x1b, 0);                           // LFO waveform
  write_reg(0x18, 0);                           // LFO frequency
  for (int i = 0x20; i < 0x100; i++) write_reg(i, 0);
}

// --- one output sample -----------------------------------------------------
//
// Mono: the console has one audio pin, so left and right are averaged. A
// channel enabled on both sides comes out at full level, one panned hard to a
// side at half - which is what a mono downmix of the real chip does.
static inline int32_t next_sample() {
  advance_eg();
  for (int ch = 0; ch < 8; ch++) chanout[ch] = 0;
  for (int ch = 0; ch < 8; ch++) chan_calc(ch);

  int32_t outl = 0, outr = 0;
  for (int ch = 0; ch < 8; ch++) {
    outl += (int32_t)((uint32_t)chanout[ch] & pan[2 * ch]);
    outr += (int32_t)((uint32_t)chanout[ch] & pan[2 * ch + 1]);
  }
  if (outl >  32767) outl =  32767; else if (outl < -32768) outl = -32768;
  if (outr >  32767) outr =  32767; else if (outr < -32768) outr = -32768;

  // Order matters and is MAME's: the chip advances, and only then can a timer
  // expire.  Running the timers first would consume a CSM key-on request one
  // sample early - which the self-test catches as a mismatch at exactly the
  // first timer A overflow.
  advance();
  advance_timers();
  return (outl + outr) >> 1;
}

}  // namespace ym2151

#pragma GCC pop_options

// ---------------------------------------------------------------------------
// Core 0 audio loop.  Everything below is Pico-only, so that
// tools/ym2151_selftest/ can compile the DSP above on a host with -DYM_HOST_TEST
// and diff it against MAME's own code.
// ---------------------------------------------------------------------------
#ifndef YM_HOST_TEST

// GP25, matching POKEY on this board. GP25 is the on-board LED pin, not a bus
// pin, and it is NOT routed to the cartridge connector. A classic Pico 2
// (RP2350A) on the standard Raspberry Pi Pico footprint brings out only
// GP0-22 and GP26-28 - 26 pins - and the 7800 bus uses every one of them
// (A0-A15 + D0-D7 + RW + CLK = 26). No GPIO is left to carry audio out to
// the console, which is why this cannot simply be routed like on PicoA10400.
// To actually HEAR this: remove the on-board LED and run a wire from GP25 to
// pin 18 (AUD IN) of the 7800 slot. UNTESTED - no such bodge has been built
// or measured. Without it ym_run() still synthesises, and the only visible
// effect is the LED changing brightness with the music. See pokey.h.
#define YM_AUDIO_PIN    25
#define YM_PWM_WRAP     511         // 9-bit duty; carrier = clk_sys/512 = 488kHz
#define YM_VOLUME_SHIFT 7           // 16-bit signed -> 9-bit unsigned duty

// Diagnostics. OFF by default: core 0 is supposed to stay away from SRAM and USB
// once a game is running (patch 0.27), and a print does both. Set to 1 to have
// the firmware report, once, how many samples per second it actually produced -
// the only honest way to answer "does the RP2040 keep up with 32 operators at
// 55930 Hz" without guessing at cycle counts. If the reported rate is materially
// below YM_SAMPLE_RATE the music plays flat and slow, and the fix is to lower
// YM_SAMPLE_RATE (and rescale nothing else - see the note on the native rate).
// Off for normal builds. Setting it to 1 makes identify_cartridge() time the
// synthesis under two loads and append the result to YM_LOG.TXT in the root of
// the ROM drive - the only usable channel on real hardware, where the Pico is
// powered by the console and cannot also be on a USB cable. It answered the
// 2026-08-24 "distorted and slow" report and is left in place because that
// question will come back the next time this core is touched.
//
// Nothing under this switch runs during a game: the measurement happens at cart
// load time, before core 1 starts emulating, so the gameplay path is identical
// either way. It costs ~0.7s of load time and one file write per YM cart.
#define YM_REPORT_RATE 0

// Native OPM rate: the XM clocks the chip at 2x the console colour clock
// (MAME xm.cpp:110, YM2151 at CLK_NTSC*2 = 3579544 Hz) and the chip divides by
// 64.  Kept identical for the PAL build - the XM has its own crystal, so the
// chip does not run at the console's rate on either machine.
#define YM_CHIP_CLOCK   3579544u
#define YM_SAMPLE_RATE  (YM_CHIP_CLOCK / 64u)               // 55930 Hz

// Core clock while a YM2151 cart is emulated. Raised ONLY for those, so a board
// playing anything else stays exactly as cool as it was.
//
// Measured on hardware 2026-08-24 (ym2151.h, YM_REPORT_RATE, logged to
// YM_LOG.TXT), 20000 samples per run, dead repeatable across five boots:
//     load  samples/s   of 55930 needed   us/sample (budget 17.879)
//     typ      55396        99.0%             18.051
//     max      51273        91.7%             19.503
// "typ" is an ordinary voicing; "max" is a synthetic worst case - all 32
// operators at full level AND phase modulation on every channel, which forces a
// scattered per-operator lookup into the 33KB ym_freq[] table that cannot stay
// in the 16KB XIP cache. So even normal music was 1% short at 250MHz, which is
// not a margin at all, and dense passages were 8% short.
//
// The clock needed to close those gaps is 252.4MHz (typ) and 272.7MHz (max).
// 300MHz gives 119% / 110% - real margin rather than a break-even figure, and
// margin is wanted here because the benchmark ran while core 1 was on the MENU,
// not in an emulate_* loop hammering SRAM. 300MHz is also well below the 400MHz
// this project ran for a period, and the 7800 dispatch already raises the core
// to 1.30V regardless of the clock, so no extra voltage - and no voltage-driven
// heat - comes with it.
//
// If heat matters more than headroom, 276000 still clears both loads (109%/101%).
// Nothing about the BUS timing needs this: per the margin table above, 250MHz
// was already ~3.9x inside MARIA's tightest access interval. This is purely to
// buy core 0 enough cycles to finish a sample on time.
#define YM_CLOCK_KHZ 300000
// Sample period: a whole number of microseconds plus a 16-bit fraction, tracked
// SEPARATELY. Plain integer microseconds would be 17 instead of 17.879 - a 5%
// sharp chip, most of a semitone - so the fraction has to be carried somehow.
//
// It must NOT be carried by holding the target time itself in 16.16 microseconds,
// which is what this did until 2026-08-24: time_us_32() counts microseconds in a
// full 32 bits, so "time_us_32() << 16" throws the top 16 bits away and the target
// only tracks the low 65.536ms of the clock. Comparing that truncated target
// against a full-width time_us_32() is then true on essentially every pass - at
// 5s after boot the comparison reads (5000000 - 19264), i.e. "overdue by 5
// seconds" - so the loop never waited at all. It free-ran, emitting samples as
// fast as core 0 could compute them, which is audible exactly as reported from
// hardware: playback slowed to core 0's throughput, and roughened by that
// throughput varying from one sample to the next.
#define YM_PERIOD_US_INT  (1000000u / YM_SAMPLE_RATE)                    /* 17 */
#define YM_PERIOD_US_FRAC (((uint32_t)(((uint64_t)1000000u << 16) / YM_SAMPLE_RATE)) \
                           - ((1000000u / YM_SAMPLE_RATE) << 16))        /* 0.879 */

static inline void ym_audio_init(void) {
  gpio_set_function(YM_AUDIO_PIN, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(YM_AUDIO_PIN);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, YM_PWM_WRAP);
  pwm_init(slice, &cfg, true);
  pwm_set_gpio_level(YM_AUDIO_PIN, (YM_PWM_WRAP + 1) / 2);   // mid rail = silence
}

#if YM_REPORT_RATE
// Throughput measurement, taken at CART LOAD TIME - not from inside the running
// game. Two hard reasons it cannot be done from ym_run():
//
//   * Writing a file means programming the same flash chip the firmware executes
//     from. The SDK has to disable XIP across BOTH cores to do it, and core 1 is
//     by then spinning in an emulate_* loop with interrupts disabled ("cpsid i")
//     and can neither be locked out nor acknowledge anything. Nothing in this
//     sketch had ever written to the filesystem before - the only syncBlocks()
//     call in the whole codebase is inside the USB MSC callback, which only runs
//     when no game is running. A 2026-08-24 build that tried it came back from
//     hardware with the audio dead from the start and no file written.
//   * That earlier attempt also skipped flash.syncBlocks(), so even the part that
//     did run left the data sitting in Adafruit_SPIFlash's RAM cache, to be lost
//     at power-off. Both mistakes are gone: this runs where identify_cartridge()
//     is already doing ordinary file I/O, and it flushes.
//
// The load is deliberately WORST CASE - all 32 operators keyed on and loud, so
// none of them can take the ENV_QUIET early-out, plus PMS=7 on every channel to
// force the expensive per-operator ym_freq[] lookup in the phase generator. Real
// music is lighter than this, so a pass here is a real pass.
//
// Caveat worth stating: core 1 is running the MENU here, not an emulate_* loop,
// so SRAM contention is not identical to gameplay. This measures core 0's raw
// capability, which is the question actually being asked.
// One timed run under a given register load. 'worst' picks between the two loads
// described at ym_benchmark_and_log().
static uint32_t ym_bench_one(int worst, char *out, size_t outsz, const char *label) {
  ym2151::reset();
  for (int ch = 0; ch < 8; ch++) {
    // ALG 7 routes all four operators to the output, so none of them can be
    // skipped; ALG 4 is the ordinary two-stack voicing most music actually uses.
    ym2151::write_reg(0x20 + ch, worst ? 0xC7 : 0xC4);
    for (int op = 0; op < 4; op++) {
      int s = op * 8 + ch;
      ym2151::write_reg(0x40 + s, 0x01);          // DT1 0, MUL 1
      // TL 0 keeps every operator far above ENV_QUIET, so the early-out in
      // chan_calc() never fires. The typical load leaves the modulators quiet
      // enough to be skipped, which is what real voicings look like.
      ym2151::write_reg(0x60 + s, worst ? 0x00 : ((op & 1) ? 0x18 : 0x40));
      ym2151::write_reg(0x80 + s, 0x1F);          // KS 0, AR 31
      ym2151::write_reg(0xA0 + s, worst ? 0x00 : 0x0C);
      ym2151::write_reg(0xC0 + s, worst ? 0x00 : 0x06);
      ym2151::write_reg(0xE0 + s, worst ? 0x00 : 0x3A);
    }
    ym2151::write_reg(0x28 + ch, 0x4A);           // KC
    ym2151::write_reg(0x30 + ch, 0x00);           // KF
    // PMS != 0 is the expensive one: it forces a per-operator ym_freq[] lookup
    // in the phase generator every sample, scattered across a 33KB table that
    // does not stay in the 16KB XIP cache. PMS 0 collapses that to "phase +=
    // freq". Almost all real 7800 YM music leaves PMS at 0.
    ym2151::write_reg(0x38 + ch, worst ? 0x73 : 0x00);
  }
  ym2151::write_reg(0x18, 0x00);                  // LFO frequency
  ym2151::write_reg(0x19, worst ? 0xFF : 0x80);   // PMD
  ym2151::write_reg(0x19, worst ? 0x7F : 0x00);   // AMD
  ym2151::write_reg(0x1B, 0x02);                  // triangle LFO
  ym2151::write_reg(0x0F, worst ? 0x9F : 0x00);   // noise on channel 7
  for (int ch = 0; ch < 8; ch++) ym2151::write_reg(0x08, 0x78 | ch);   // key on

  const uint32_t N = 20000;                       // ~0.36s of audio
  uint32_t t0 = time_us_32();
  for (uint32_t i = 0; i < N; i++) (void)ym2151::next_sample();
  uint32_t dt = time_us_32() - t0;
  if (!dt) dt = 1;
  uint32_t rate   = (uint32_t)(((uint64_t)N * 1000000u) / dt);
  uint32_t per_ns = (uint32_t)(((uint64_t)dt * 1000u) / N);

  // The benchmark necessarily runs BEFORE core 1 raises the clock for this cart -
  // it has to, because writing the log file is only safe while no game is
  // running. So report both: what was measured at the current clock, and what
  // that projects to at the clock the game is about to use. Without the
  // projection the log reads "TOO SLOW" forever and says nothing about whether
  // the raised clock is enough.
  uint32_t now_khz  = (uint32_t)(clock_get_hz(clk_sys) / 1000);
  uint32_t play_khz = YM_CLOCK_KHZ;
  uint32_t proj     = (uint32_t)(((uint64_t)rate * play_khz) / (now_khz ? now_khz : 1));
  snprintf(out, outsz,
           "YM bench %s: %lu/s @%lukHz (%lu.%03lu us) -> %lu/s @%lukHz of %u needed = %lu%% %s\n",
           label, (unsigned long)rate, (unsigned long)now_khz,
           (unsigned long)(per_ns / 1000), (unsigned long)(per_ns % 1000),
           (unsigned long)proj, (unsigned long)play_khz, (unsigned)YM_SAMPLE_RATE,
           (unsigned long)((uint64_t)proj * 100u / YM_SAMPLE_RATE),
           proj >= YM_SAMPLE_RATE ? "OK" : "TOO SLOW");
  return rate;
}

static void ym_benchmark_and_log(void) {
  // Two loads, because one number was not enough to act on. The 2026-08-24
  // hardware run measured 51273 samples/s against 55930 needed under the "max"
  // load - 8.3% short - but that load is a synthetic worst case that no real
  // cartridge produces: 32 operators all at full level AND phase modulation
  // enabled on every channel. "typ" is what an ordinary voicing costs. If typ
  // passes and max does not, real music fits and only dense passages will make
  // the drift clamp in ym_run() drop a few samples.
  char lmax[160], ltyp[160];
  ym_bench_one(1, lmax, sizeof(lmax), "max");
  ym_bench_one(0, ltyp, sizeof(ltyp), "typ");
  Serial.print(lmax);
  Serial.print(ltyp);

  // FatFile in this SdFat fork is not a Print/Stream - write() only, hence the
  // pre-built lines. Opened against the VOLUME so it lands in the root rather
  // than in whichever folder the menu happens to be browsing.
  FatFile logf;
  if (logf.open(&fatfs, "YM_LOG.TXT", O_WRONLY | O_CREAT | O_APPEND)) {
    logf.write(lmax);
    logf.write(ltyp);
    logf.close();
    flash.syncBlocks();      // without this the write dies in the RAM cache
    fatfs.cacheClear();
  }
  ym2151::reset();           // leave the chip clean for the game about to start
}
#endif

// Runs only while a YM2151 cart is playing; never returns.
// No optimize attribute here on purpose: this function is just the scheduling
// loop, and all the work it schedules lives in ym2151::next_sample(), which is
// covered by the scoped O3 pragma above. Putting one here was measured to buy
// almost nothing, for exactly that reason.
static inline void ym_run(void) {
  // Bounded: a timeout can only cost us a slightly wrong PWM carrier, whereas an
  // unbounded wait on a flag that never arrives would be silent audio forever.
  uint32_t wait_t0 = time_us_32();
  while (!emu_clock_ready && (uint32_t)(time_us_32() - wait_t0) < 200000u)
    tight_loop_contents();

  ym2151::reset();
  ym_audio_init();

  uint32_t next_us = time_us_32();          // when the next sample is due
  uint32_t frac    = 0;                     // carried 1/65536ths of a microsecond
  while (1) {
    // Apply everything core 1 has captured since the last sample. Draining the
    // whole ring here is what keeps the queue shallow: a 6502 cannot produce
    // more than a handful of register writes per 17.9us sample.
    while (ym_q_tail != ym_q_head) {
      uint16_t e = ym_queue[ym_q_tail & (YM_QUEUE_SIZE - 1)];
      ym_q_tail++;
      ym2151::write_reg(e >> 8, e & 0xFF);
    }

    uint32_t now = time_us_32();
    if ((int32_t)(now - next_us) >= 0) {     // signed difference: wrap-safe
      int32_t s = ym2151::next_sample();
      s = (s >> YM_VOLUME_SHIFT) + ((YM_PWM_WRAP + 1) / 2);
      if (s < 0) s = 0; else if (s > YM_PWM_WRAP) s = YM_PWM_WRAP;
      pwm_set_gpio_level(YM_AUDIO_PIN, (uint16_t)s);
      ym_status = ym2151::status;               // publish timer flags to core 1

      // Schedule the next sample one native period after this one was DUE, not
      // after "now" - that is what keeps the average rate exact even though each
      // individual wake-up lands a microsecond or so late.
      frac += YM_PERIOD_US_FRAC;
      next_us += YM_PERIOD_US_INT + (frac >> 16);
      frac &= 0xFFFF;

      // If core 0 could not keep up for a stretch, do NOT let the backlog grow
      // without bound: "catching up" then means firing a burst of samples
      // back-to-back with no pacing between them as soon as spare cycles appear.
      // Clamp instead - resync to now and drop the backlog. A few lost
      // milliseconds of a soundtrack is inaudible; an unpaced burst is not.
      int32_t behind = (int32_t)(time_us_32() - next_us);
      if (behind > (int32_t)(YM_PERIOD_US_INT * 4)) {
        next_us = time_us_32();
        frac = 0;
      }
    }
  }
}

#endif // YM_HOST_TEST

#endif // YM2151_H
