#ifndef SN76489_H
#define SN76489_H

// SN76489 - the sound chip on the SN/Eagle family of 7800 cartridges (.s78),
// sitting at a single write-only latch at $043F.
//
// Split the same way pokey.h is, and for the same reason: core 1 is inside a
// __time_critical_func() bus loop and must do nothing but drop the byte in a
// queue, while core 0 does the synthesis and drives the PWM pin. Nothing on
// the core 1 path touches flash - the queue is .bss and the capture function
// has no table lookups at all.
//
// The chip model is a transcription of MAME's sn76496.cpp using the
// parameters MAME gives to sn76489_device specifically (feedback mask
// 0x4000, taps 0x01/0x02, 15 bit shift register). The same model was written
// in Go first, dropped into our copy of test7800 and checked there before
// being ported here: tone frequencies matched clock/(32*N) to a tenth of a
// hertz over five different periods, and the white-noise sequence repeated
// after exactly 32767 shifts. That is what makes this a port rather than a
// fresh guess - see TEST_ROMS_SN/README.md.

// The PWM side is skipped when the model is compiled on a host for testing,
// the same arrangement pokey.h uses with POKEY_HOST_TEST.
#ifndef SN76489_HOST_TEST
#include "hardware/pwm.h"
#endif

// The single write latch. Same shape as the POKEY/YM windows - one address in
// the $0000-$3FFF branch - which is why the bus loop can test for it with the
// same two instructions.
#define SN76489_BASE   0x043Fu

// Shares the audio pin and carrier with POKEY: one line to the console, one
// cart playing at a time. GPIO29 -> 7800 connector pin 18 ("AUD IN").
#define SN_AUDIO_PIN   29
#define SN_PWM_WRAP    511          // 9-bit, carrier = clk_sys / 512

// The clock the cartridge feeds the chip: the 6502's own, ~1.79MHz. Taken
// from Ocelot, the emulator written by the author of the SN board, which says
// "SN76489 runs at CPU_CLOCK/16 = 111720 Hz" and clocks its tone counters at
// exactly that rate. 0.62-0.64 assumed the ~3.58MHz colourburst instead - what
// an SN76489 is given in most other designs - and so played every note an
// octave above Ocelot. The test ROMs could not show that: their periods are
// arbitrary, so only the board author's emulator could say which octave is
// right. This is the NTSC machine clock; a PAL 7800 runs its 6502 0.9% slower,
// which nothing here can see - and Ocelot uses one figure for both regions.
#define SN_CHIP_CLOCK_HZ  1789773u

// The chip divides its input clock by 16 before the tone counters see it
#define SN_TICK_HZ     (SN_CHIP_CLOCK_HZ / 16u)    // 111860

// Audio sample rate, matching pokey.h
#define SN_SAMPLE_HZ   32000u

// Counter ticks per audio sample, 16.16 fixed point. 3.4956 - the fraction
// has to be carried or the pitch drifts sharp, the same trap pokey.h and
// ym2151.h both document in their pacing.
#define SN_TICKS_Q16   ((uint32_t)(((uint64_t)SN_TICK_HZ << 16) / SN_SAMPLE_HZ))

// Set by identify_cartridge() for a .s78 file
volatile uint8_t sn76489_enabled = 0;

// ---------------------------------------------------------------------------
// core 1 -> core 0 write queue
// ---------------------------------------------------------------------------
//
// A power of two so the index is a mask. Writes to this chip are sparse - a
// music driver touches it a few dozen times a frame at most - so 256 is
// generous; icebloxplus manages four writes per frame.
#define SN_Q_SIZE      256u

static volatile uint8_t  sn_queue[SN_Q_SIZE];
static volatile uint32_t sn_q_head = 0;
static volatile uint32_t sn_q_tail = 0;

// Called from the bus loop on core 1. Three stores and a compare, no lookup
// tables, nothing in flash. A full queue drops the write rather than
// blocking: core 1 must never wait for core 0.
static inline __attribute__((always_inline))
void sn76489_capture_write(uint8_t val) {
  uint32_t h = sn_q_head;
  uint32_t n = h + 1u;
  if ((uint32_t)(n - sn_q_tail) <= SN_Q_SIZE) {
    sn_queue[h & (SN_Q_SIZE - 1u)] = val;
    sn_q_head = n;
  }
}

// ---------------------------------------------------------------------------
// SN_DIAG - hardware diagnostic build, default OFF
// ---------------------------------------------------------------------------
//
// 0.62 and 0.63 both came back from the console as "the same tone on both
// files" while every emulator played the same files correctly, and a reasoned
// fix in between did not move it. So this build measures instead of guessing
// a third time. With SN_DIAG 1:
//
//   * core 1 records every bus cycle that lands on $043F - the raw GPIO word
//     at entry and the last in-window word, how long the cycle held and
//     whether it passed the R/W gate - and every bank-select write, and it
//     counts RAM writes, $FFFF writes and fetches of the reset entry and the
//     three vectors, so a ROM that keeps restarting shows up as a number;
//   * core 0 records every byte it actually hands to the chip model;
//   * twelve seconds in, logging freezes and core 0 writes all of it, plus
//     the chip model's registers, into the High Score Cart slot through
//     nv_write_slot() - the in-game flash write FA2 and the HSC probe already
//     make. tools/sn_diag_decode.py reads it back out of /.save_hsc.data.
//
// THIS OVERWRITES /.save_hsc.data. Copy it off the drive first.
//
// All of it compiles to nothing with SN_DIAG 0.
#ifndef SN_DIAG
#define SN_DIAG 0
#endif

#if SN_DIAG
#define SND_ENTRIES        120u
#define SND_BANKW           32u
#define SND_APPLIED        192u
#define SND_DUMP_AFTER_US  12000000u

typedef struct { uint32_t first, last, meta; } snd_entry_t;

static volatile uint32_t snd_frozen = 0;
static volatile uint32_t snd_n_entry = 0, snd_n_capt = 0, snd_n_rwrej = 0, snd_n_bound = 0;
static volatile uint32_t snd_n_bankw = 0, snd_n_ramw = 0, snd_n_ctrl = 0;
static volatile uint32_t snd_rd_f000 = 0, snd_rd_fffc = 0, snd_rd_fffa = 0, snd_rd_fffe = 0;
static volatile uint32_t snd_n_applied = 0;
static snd_entry_t snd_entry[SND_ENTRIES];
static uint32_t    snd_bankw[SND_BANKW][2];
static uint8_t     snd_applied[SND_APPLIED];
static uint8_t     snd_dump[2048] __attribute__((aligned(4)));

// Core 1. Everything here touches only .bss and is forced inline, so none of
// it can bring a flash read or a call into the bus loop - checked in the .elf.
static inline __attribute__((always_inline))
void snd_log_entry(uint32_t first, uint32_t last, uint32_t meta) {
  if (snd_frozen) return;
  const uint32_t n = snd_n_entry;
  snd_n_entry = n + 1u;
  if (meta & 1u) snd_n_capt = snd_n_capt + 1u;
  else           snd_n_rwrej = snd_n_rwrej + 1u;
  if ((meta >> 16) >= 256u) snd_n_bound = snd_n_bound + 1u;
  if (n < SND_ENTRIES) {
    snd_entry[n].first = first;
    snd_entry[n].last  = last;
    snd_entry[n].meta  = meta;
  }
}

static inline __attribute__((always_inline))
void snd_log_bankw(uint32_t first, uint32_t last) {
  if (snd_frozen) return;
  const uint32_t n = snd_n_bankw;
  snd_n_bankw = n + 1u;
  if (n < SND_BANKW) { snd_bankw[n][0] = first; snd_bankw[n][1] = last; }
}

static inline __attribute__((always_inline)) void snd_count_ramw(void) {
  if (!snd_frozen) snd_n_ramw = snd_n_ramw + 1u;
}

static inline __attribute__((always_inline)) void snd_count_ctrl(void) {
  if (!snd_frozen) snd_n_ctrl = snd_n_ctrl + 1u;
}

// 'off' is the offset inside the fixed bank ($F000 -> 0x000, $FFFC -> 0xFFC),
// which is the same on both boards whatever BUS_PIN_MASK keeps of A15.
// Counted only on the first pass over a new address: the loop comes round
// many times while one fetch holds, and a raw count would measure loop speed.
static inline __attribute__((always_inline))
void snd_count_fixed_read(uint32_t off, uint32_t edge) {
  if (!edge || snd_frozen) return;
  if      (off == 0x000u) snd_rd_f000 = snd_rd_f000 + 1u;
  else if (off == 0xFFCu) snd_rd_fffc = snd_rd_fffc + 1u;
  else if (off == 0xFFAu) snd_rd_fffa = snd_rd_fffa + 1u;
  else if (off == 0xFFEu) snd_rd_fffe = snd_rd_fffe + 1u;
}

// Core 0 - every byte the chip model is actually given, in order.
static inline void snd_log_applied(uint8_t b) {
  if (snd_frozen) return;
  const uint32_t k = snd_n_applied;
  if (k < SND_APPLIED) snd_applied[k] = b;
  snd_n_applied = k + 1u;
}
#endif // SN_DIAG

// ---------------------------------------------------------------------------
// chip state - core 0 only
// ---------------------------------------------------------------------------

// SN76489, not SN76489A. MAME keeps them as separate devices because the
// noise generator differs: this one has a 15 bit register and taps 0 and 1.
#define SN_FEEDBACK    0x4000u
#define SN_TAP1        0x0001u
#define SN_TAP2        0x0002u

// 2dB per step from a peak of 127, so four channels at full volume come to
// 508 and land just inside the 9-bit PWM range without a scaling divide.
// Attenuation 15 is silence.
//
// That is the same per-channel scale POKEY gets (15/60 of the range), so an
// SN game playing three voices is as loud as a POKEY game playing three. A
// test ROM playing ONE voice is correspondingly quieter - which is most of
// what "two to three times quieter" came back as on hardware in 0.62.
//
// SN_LOUD doubles the peak. One voice then reaches half the range, and four
// voices overflow it and are clipped by the clamp in sn76489_next_sample() -
// only audible when three or four channels are loud at once. Built as its own
// variant (_SNLOUD) rather than folded into the fix, so a hardware test moves
// one dial at a time: the lesson E6 recorded in pokey.h.
#ifndef SN_LOUD
#define SN_LOUD 0
#endif
#if SN_LOUD
static const uint8_t sn_vol_table[16] = {
  254, 202, 160, 127, 101, 80, 64, 51, 40, 32, 25, 20, 16, 13, 10, 0
};
#else
static const uint8_t sn_vol_table[16] = {
  127, 101, 80, 64, 51, 40, 32, 25, 20, 16, 13, 10, 8, 6, 5, 0
};
#endif

static uint16_t sn_reg[8];
static int      sn_last_reg;
static int32_t  sn_period[4];
static int32_t  sn_count[4];
static uint8_t  sn_out[4];
static int32_t  sn_vol[4];
static uint32_t sn_rng;
static uint32_t sn_tick_acc;

static inline void sn76489_reset_state(void) {
  for (int i = 0; i < 8; i++) sn_reg[i] = 0;
  sn_last_reg = 0;
  for (int i = 0; i < 4; i++) {
    sn_period[i] = 0;
    sn_count[i]  = 0;
    sn_out[i]    = 0;
    // Silent until the game writes an attenuation. The period registers are
    // zero at this point and would otherwise toggle the output on every
    // single tick - a reset screech.
    sn_vol[i]    = 0;
  }
  sn_rng = SN_FEEDBACK;
  sn_out[3] = (uint8_t)(sn_rng & 1u);
  sn_tick_acc = 0;
}

// One byte written to the latch. Bit 7 set is a LATCH byte, which picks the
// register and carries the low four bits; bit 7 clear is a DATA byte that
// continues whichever register was latched last.
static inline void sn76489_apply(uint8_t data) {
  int r;

  if (data & 0x80u) {
    r = (int)((data & 0x70u) >> 4);
    sn_last_reg = r;
    sn_reg[r] = (uint16_t)((sn_reg[r] & 0x3f0u) | (uint16_t)(data & 0x0fu));
  } else {
    r = sn_last_reg;
  }

  int c = r >> 1;

  switch (r) {
    case 0: case 2: case 4:                     // tone period
      if (!(data & 0x80u))
        sn_reg[r] = (uint16_t)((sn_reg[r] & 0x0fu) | ((uint16_t)(data & 0x3fu) << 4));
      // a period register of zero means the full 1024, not "toggle every tick"
      sn_period[c] = sn_reg[r] ? (int32_t)sn_reg[r] : 0x400;
      if (r == 4 && (sn_reg[6] & 0x03u) == 0x03u)
        sn_period[3] = sn_period[2] << 1;       // noise slaved to tone 2
      break;

    case 1: case 3: case 5: case 7:             // attenuation
      sn_vol[c] = (int32_t)sn_vol_table[data & 0x0fu];
      if (!(data & 0x80u))
        sn_reg[r] = (uint16_t)((sn_reg[r] & 0x3f0u) | (uint16_t)(data & 0x0fu));
      break;

    case 6: {                                   // noise control
      if (!(data & 0x80u))
        sn_reg[r] = (uint16_t)((sn_reg[r] & 0x3f0u) | (uint16_t)(data & 0x0fu));
      uint16_t n = sn_reg[6];
      if ((n & 0x03u) == 0x03u) sn_period[3] = sn_period[2] << 1;
      else                      sn_period[3] = (int32_t)(1u << (5u + (n & 0x03u)));
      sn_rng = SN_FEEDBACK;
      break;
    }
  }
}

static inline void sn76489_tick(void) {
  for (int i = 0; i < 3; i++) {
    if (--sn_count[i] <= 0) {
      sn_out[i] ^= 1u;
      sn_count[i] = sn_period[i];
    }
  }
  if (--sn_count[3] <= 0) {
    // in periodic mode the second tap is held at zero
    uint32_t t1 = sn_rng & SN_TAP1;
    uint32_t t2 = (sn_rng & SN_TAP2) && (sn_reg[6] & 0x04u);
    sn_rng >>= 1;
    if ((t1 != 0u) != (t2 != 0u)) sn_rng |= SN_FEEDBACK;
    sn_out[3] = (uint8_t)(sn_rng & 1u);
    sn_count[3] = sn_period[3];
  }
}

static inline uint32_t sn76489_mix(void) {
  uint32_t s = 0;
  if (sn_out[0]) s += (uint32_t)sn_vol[0];
  if (sn_out[1]) s += (uint32_t)sn_vol[1];
  if (sn_out[2]) s += (uint32_t)sn_vol[2];
  if (sn_out[3]) s += (uint32_t)sn_vol[3];
  return s;
}

// One audio sample. Every write queued since the last call is applied first -
// at 32kHz that puts a write within 31us of where the game made it, which is
// far below anything audible. Then the chip is advanced tick by tick and the
// output averaged, which is also the anti-aliasing: without it a tone above
// 16kHz would fold back down as a whistle.
static inline uint16_t sn76489_next_sample(void) {
  uint32_t t = sn_q_tail;
#if SN_DIAG
  while (t != sn_q_head) {
    const uint8_t b = sn_queue[t & (SN_Q_SIZE - 1u)];
    snd_log_applied(b);
    sn76489_apply(b);
    t++;
  }
#else
  while (t != sn_q_head) {
    sn76489_apply(sn_queue[t & (SN_Q_SIZE - 1u)]);
    t++;
  }
#endif
  sn_q_tail = t;

  sn_tick_acc += SN_TICKS_Q16;
  uint32_t n = sn_tick_acc >> 16;
  sn_tick_acc &= 0xFFFFu;
  if (n == 0u) return (uint16_t)sn76489_mix();

  uint32_t sum = 0;
  for (uint32_t i = 0; i < n; i++) {
    sn76489_tick();
    sum += sn76489_mix();
  }
  uint32_t out = sum / n;
  if (out > SN_PWM_WRAP) out = SN_PWM_WRAP;
  return (uint16_t)out;
}

#ifndef SN76489_HOST_TEST

static inline void sn76489_audio_init(void) {
  gpio_set_function(SN_AUDIO_PIN, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(SN_AUDIO_PIN);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, SN_PWM_WRAP);
  pwm_init(slice, &cfg, true);
  pwm_set_gpio_level(SN_AUDIO_PIN, 0);
}

#if SN_DIAG
static void snd_put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

// Lays out the 2048-byte image that tools/sn_diag_decode.py documents and
// hands it to nv_write_slot(). Called once, with logging frozen, so the buffer
// stays quiet through the erase and program - nv_write_slot() programs a
// payload of this size straight out of the caller's buffer.
static void snd_write_dump(uint32_t elapsed_ms) {
  uint8_t *d = snd_dump;
  memset(d, 0, sizeof(snd_dump));
  memcpy(d, "SNDG", 4);
  snd_put32(d +   4, 1u);                       // format version
  snd_put32(d +   8, elapsed_ms);
  snd_put32(d +  12, snd_n_entry);
  snd_put32(d +  16, snd_n_capt);
  snd_put32(d +  20, snd_n_rwrej);
  snd_put32(d +  24, snd_n_bound);
  snd_put32(d +  28, snd_n_bankw);
  snd_put32(d +  32, snd_n_ramw);
  snd_put32(d +  36, snd_n_ctrl);
  snd_put32(d +  40, snd_rd_f000);
  snd_put32(d +  44, snd_rd_fffc);
  snd_put32(d +  48, snd_rd_fffa);
  snd_put32(d +  52, snd_rd_fffe);
  snd_put32(d +  56, snd_n_applied);
  snd_put32(d +  60, sn_q_head);
  snd_put32(d +  64, sn_q_tail);
  snd_put32(d +  68, (uint32_t)sn_last_reg);
  for (int i = 0; i < 8; i++) {
    d[72 + 2 * i] = (uint8_t)sn_reg[i];
    d[73 + 2 * i] = (uint8_t)(sn_reg[i] >> 8);
  }
  for (int i = 0; i < 4; i++) snd_put32(d +  88 + 4 * i, (uint32_t)sn_period[i]);
  for (int i = 0; i < 4; i++) snd_put32(d + 104 + 4 * i, (uint32_t)sn_vol[i]);
  snd_put32(d + 120, (uint32_t)BUS_PIN_MASK);   // so the decoder needs no
  snd_put32(d + 124, (uint32_t)RW_PIN_MASK);    // knowledge of which board
  snd_put32(d + 128, (uint32_t)A15_PIN_MASK);   // wrote the file
  snd_put32(d + 132, (uint32_t)D0_PIN);
  snd_put32(d + 136, SND_ENTRIES);
  snd_put32(d + 140, SND_BANKW);
  snd_put32(d + 144, SND_APPLIED);
  for (uint32_t i = 0; i < SND_ENTRIES; i++) {
    snd_put32(d + 160 + 12 * i,     snd_entry[i].first);
    snd_put32(d + 160 + 12 * i + 4, snd_entry[i].last);
    snd_put32(d + 160 + 12 * i + 8, snd_entry[i].meta);
  }
  for (uint32_t i = 0; i < SND_BANKW; i++) {
    snd_put32(d + 1600 + 8 * i,     snd_bankw[i][0]);
    snd_put32(d + 1600 + 8 * i + 4, snd_bankw[i][1]);
  }
  memcpy(d + 1856, snd_applied, SND_APPLIED);
  nv_write_slot(NV_SLOT_HSC, snd_dump, sizeof(snd_dump));
}
#endif // SN_DIAG

// Core 0 audio loop for an SN cart. Never returns, exactly like pokey_run().
static inline void sn76489_run(void) {
  // Core 1 may still be changing the system clock when we get here, and
  // configuring PWM against a clock that is about to move is what came back
  // from hardware as "no audio at all" for POKEY on 2026-08-24. Same bounded
  // wait, same flag.
  uint32_t wait_t0 = time_us_32();
  while (!emu_clock_ready && (uint32_t)(time_us_32() - wait_t0) < 200000u)
    tight_loop_contents();

  sn76489_reset_state();
  sn76489_audio_init();

  // 31.25us per sample, carried as whole microseconds plus a 16-bit fraction
  const uint32_t period_us_int  = 31u;
  const uint32_t period_us_frac = 16384u;
  uint32_t next_us = time_us_32();
  uint32_t frac_us = 0;
#if SN_DIAG
  const uint32_t snd_t0 = time_us_32();
  uint32_t snd_done = 0;
#endif
  while (1) {
    uint32_t now = time_us_32();
    if ((int32_t)(now - next_us) >= 0) {
      pwm_set_gpio_level(SN_AUDIO_PIN, sn76489_next_sample());
      frac_us += period_us_frac;
      next_us += period_us_int + (frac_us >> 16);
      frac_us &= 0xFFFFu;
    }
#if SN_DIAG
    if (!snd_done && (uint32_t)(now - snd_t0) >= SND_DUMP_AFTER_US) {
      snd_done = 1u;
      snd_frozen = 1u;
      __dmb();
      snd_write_dump((uint32_t)(now - snd_t0) / 1000u);
      next_us = time_us_32();   // the flash write stalls this loop: resync
      frac_us = 0;
    }
#endif
  }
}

#endif // SN76489_HOST_TEST

#endif // SN76489_H
