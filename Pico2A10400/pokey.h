// pokey.h - minimal POKEY audio synthesis for Pico2A10400
//
// !!! TA PLYTKA NIE MA PODLACZONEJ LINII AUDIO !!!
//
// Odczytane ze schematu Pico2A10400.pdf: plytka uzywa STANDARDOWEJ stopki
// Raspberry Pi Pico (symbol U3), ktora wyprowadza tylko GPIO 0-22 i 26-28 - razem
// 26 pinow. Magistrala 7800 zjada je co do jednego: A0-A15 (16) + D0-D7 (8) + RW +
// CLK = 26. Etykieta sieci "ExtAudio" istnieje przy pinie 18 zlacza kartridzowego
// (U2), ale NIE MA odpowiednika po stronie Pico - siec konczy sie w powietrzu.
//
// Dla porownania PicoA10400 uzywa "purpurowego klona", ktory wyprowadza dodatkowo
// GP23/24/25/29, i tam pin 18 jest poprowadzony do GP29 (patrz
// pokey_feasibility/README.md) - dlatego dzwiek dziala na tamtej plytce.
//
// Kod jest tu przeniesiony w calosci, zeby oba szkice pozostaly zgodne (CLAUDE.md)
// i zeby przechwytywanie rejestrow POKEY dzialalo identycznie. Synteza tez sie
// wykonuje. Zeby ja USLYSZEC, trzeba fizycznie polaczyc POKEY_AUDIO_PIN z pinem 18
// zlacza - domyslnie ustawiony na GP25, bo to jedyny pin dostepny programowo, ktory
// nie obsluguje magistrali (dioda LED na plytce Pico). Bez takiego przewodu ta
// czesc kodu nic nie robi poza miganiem dioda w rytm dzwieku.
//
#ifndef POKEY_H
#define POKEY_H

#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define POKEY_AUDIO_PIN   25        // LED na plytce - pin 18 NIE jest tu poprowadzony
#define POKEY_SAMPLE_RATE 32000
#define POKEY_PWM_WRAP    255       // 8-bit PWM; carrier = clk_sys / 256

// Register file, written by core 1 from the bus, read by core 0. Plain volatile
// bytes are enough: these are state, not events, so a late or reordered update is
// at worst one sample stale. No queue, no locking, nothing core 1 has to wait for -
// its hot loop must not block.
volatile uint8_t pokey_regs[16] = {0};
volatile uint8_t pokey_enabled  = 0;   // set by identify_cartridge() from the header
// $4000 (byte54 bit0) or $0450 (bit6). 0xFFFF when disabled - NOT 0, because
// (addr & 0xFFF0)==0 matches the TIA registers at $0000-$000F, which every cart
// hits constantly; that made non-POKEY carts run the capture path too.
volatile uint16_t pokey_base    = 0xFFFF;
// Address mask for the window test. 0xFFF0 = 16 bytes, which is what MAME and
// test7800 decode for $4000/$0450/$0440. For $0800 MAME installs the handler over
// the WHOLE $0800-$0FFF range and decodes it with "offset & 0x0f" (rom.h:218), so
// the chip is mirrored every 16 bytes across 2KB - that is why "POKEY Tester (810)"
// was silent while "(800)" played: it addresses $0810, one mirror up.
volatile uint16_t pokey_mask    = 0xFFF0;

// --- AUDC bits ---
#define POKEY_NOTPOLY5    0x80
#define POKEY_POLY4       0x40
#define POKEY_PURE        0x20
#define POKEY_VOLUME_ONLY 0x10
#define POKEY_VOLUME_MASK 0x0f
// --- AUDCTL bits ---
#define POKEY_POLY9       0x80
#define POKEY_CH1_179     0x40
#define POKEY_CH3_179     0x20
#define POKEY_CH1_CH2     0x10
#define POKEY_CH3_CH4     0x08
#define POKEY_CLOCK_15    0x01

#define POKEY_CLK_BASE    1787520u          // POKEY clock in the 7800
#define POKEY_DIV_64      28u               // -> ~63.8 kHz
#define POKEY_DIV_15      114u              // -> ~15.7 kHz

static uint32_t pk_phase[4] = {0,0,0,0};    // 16.16 fixed point phase
static uint8_t  pk_out[4]   = {0,0,0,0};    // current output bit per channel
static uint32_t pk_lfsr17   = 1;
static uint8_t  pk_lfsr4    = 1;
static uint8_t  pk_lfsr5    = 1;

static inline void pokey_audio_init(void) {
  gpio_set_function(POKEY_AUDIO_PIN, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(POKEY_AUDIO_PIN);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, POKEY_PWM_WRAP);   // no clock divider: carrier as high
  pwm_init(slice, &cfg, true);                 // as possible, ~977kHz at 250MHz
  pwm_set_gpio_level(POKEY_AUDIO_PIN, 0);
}

// Capture one POKEY register write off the bus. Call ONLY from a branch where the
// cartridge is not driving the data lines - it never drives, it only listens, so a
// mis-read of R/W can at worst store one spurious byte instead of fighting the CPU.
// 'addr' must be the full bus address of the current cycle.
// --- bus-side service, called from core 1 -----------------------------------
//
// A real POKEY answers READS too, and that turns out to matter: a game typically
// probes for the chip by reading a register (classically RANDOM at offset $0A) and
// stays silent if the value never changes. Capturing writes alone is why 7800 XMAS
// declared POKEY yet produced nothing.
//
// always_inline is NOT decoration: called from several hot loops, GCC otherwise
// outlines this into one copy in FLASH reached through a RAM veneer - the defect
// patches 0.09/0.13 removed. check_hotpath.sh catches it if this is ever dropped.

static uint32_t pk_rand = 0x1234;      // RANDOM generator, core 1 only

static inline __attribute__((always_inline)) uint8_t pokey_read_reg(uint32_t reg) {
  if (reg == 0x0A) {                   // RANDOM - the register games probe
    pk_rand = (pk_rand >> 1) | ((((pk_rand) ^ (pk_rand >> 5)) & 1u) << 16);
    return (uint8_t)(pk_rand & 0xFF);
  }
  return 0xFF;                         // ALLPOT/IRQST/SKSTAT: idle reads as high
}

// Service one bus cycle that landed inside the POKEY register window.
// Non-blocking by design: the SuperGame loops never wait anywhere, and dropping a
// bounded wait into them cost MARIA its data (visible glitches on XMAS/Arkanoid).
// No wait is needed - the loop re-enters this branch repeatedly while the address
// sits in the window, so on a write the LAST store before the address changes is
// the one that survives, which is end-of-cycle capture for free.
static inline __attribute__((always_inline))
void pokey_window_service(uint32_t addr, uint8_t *rom_in_use) {
  // A YM2151 cart reuses this whole mechanism: identify_cartridge() points
  // pokey_base/pokey_mask at $0460/$0461 and sets pokey_enabled, so every
  // emulate_*_pokey() loop already routes the window here without a single
  // change at the call sites. Only one aux chip is emulated, so this is a
  // choice, not a merge - see ym2151.h.
  if (ym_enabled) { ym_window_service(addr, rom_in_use); return; }
  uint32_t g = gpio_get_all();
  if (g & RW_PIN_MASK) {                                   // read cycle
    sio_hw->gpio_out = (uint32_t)pokey_read_reg(addr & 0x0F) << D0_PIN;
    if (!*rom_in_use) { SET_DATA_MODE_OUT; *rom_in_use = 1; }
  } else {                                                 // write cycle
    if (*rom_in_use) { SET_DATA_MODE_IN; *rom_in_use = 0; }
    pokey_regs[addr & 0x0F] = (uint8_t)((g >> D0_PIN) & 0xFF);
  }
}

// One channel's output frequency, in Hz, or 0 when it is silent/DC.
static inline uint32_t pk_channel_freq(int ch, uint8_t audctl) {
  uint32_t clk = (audctl & POKEY_CLOCK_15) ? (POKEY_CLK_BASE / POKEY_DIV_15)
                                           : (POKEY_CLK_BASE / POKEY_DIV_64);
  uint32_t div;

  // 16-bit pairs: ch2 is the high byte of ch1, ch4 the high byte of ch3. The pair
  // is clocked by the LOW channel's clock source, and only the low channel makes
  // sound - matching how ProSystem routes the borrow (Pokey.js: CH1_CH2/CH3_CH4).
  if (ch == 0 && (audctl & POKEY_CH1_CH2)) {
    if (audctl & POKEY_CH1_179) clk = POKEY_CLK_BASE;
    div = ((uint32_t)pokey_regs[2] << 8 | pokey_regs[0]) + 1u;
  } else if (ch == 2 && (audctl & POKEY_CH3_CH4)) {
    if (audctl & POKEY_CH3_179) clk = POKEY_CLK_BASE;
    div = ((uint32_t)pokey_regs[6] << 8 | pokey_regs[4]) + 1u;
  } else if (ch == 1 && (audctl & POKEY_CH1_CH2)) {
    return 0;                                   // high half of a pair: no output
  } else if (ch == 3 && (audctl & POKEY_CH3_CH4)) {
    return 0;
  } else {
    if (ch == 0 && (audctl & POKEY_CH1_179)) clk = POKEY_CLK_BASE;
    if (ch == 2 && (audctl & POKEY_CH3_179)) clk = POKEY_CLK_BASE;
    div = (uint32_t)pokey_regs[ch * 2] + 1u;
  }
  return clk / (2u * div);                      // square wave => half the divider
}

// Produce one 8-bit sample. Called POKEY_SAMPLE_RATE times per second.
static inline uint8_t pokey_next_sample(void) {
  uint8_t audctl = pokey_regs[8];
  int32_t mix = 0;

  for (int ch = 0; ch < 4; ch++) {
    uint8_t audc = pokey_regs[ch * 2 + 1];
    uint8_t vol  = audc & POKEY_VOLUME_MASK;
    if (vol == 0) continue;

    if (audc & POKEY_VOLUME_ONLY) {             // DC: used for sampled playback
      mix += vol;
      continue;
    }

    uint32_t f = pk_channel_freq(ch, audctl);
    if (f == 0) continue;

    // Advance phase; every wrap is one half-period of the divider output, i.e.
    // exactly where real POKEY would take a borrow and clock its waveform.
    uint32_t inc = (uint32_t)(((uint64_t)f << 16) / POKEY_SAMPLE_RATE);
    uint32_t before = pk_phase[ch];
    pk_phase[ch] = before + inc;
    if ((pk_phase[ch] ^ before) & 0x10000u) {   // crossed a half period
      pk_lfsr4 = (uint8_t)(((pk_lfsr4 >> 1) | (((pk_lfsr4 ^ (pk_lfsr4 >> 1)) & 1) << 3)) & 0x0f);
      pk_lfsr5 = (uint8_t)(((pk_lfsr5 >> 1) | (((pk_lfsr5 ^ (pk_lfsr5 >> 2)) & 1) << 4)) & 0x1f);
      pk_lfsr17 = (pk_lfsr17 >> 1) | ((((pk_lfsr17) ^ (pk_lfsr17 >> 5)) & 1u) << 16);

      // The 5-bit poly gates every non-pure mode (Pokey.js process_channel()).
      if ((audc & POKEY_NOTPOLY5) || (pk_lfsr5 & 1)) {
        if (audc & POKEY_PURE)        pk_out[ch] ^= 1;
        else if (audc & POKEY_POLY4)  pk_out[ch] = pk_lfsr4 & 1;
        else                          pk_out[ch] = pk_lfsr17 & 1;
      }
    }
    if (pk_out[ch]) mix += vol;
  }

  // 4 channels x volume 15 = 60 max. Scale to 8 bits with headroom rather than
  // clipping: the console's input is being driven straight from a 3.3V pin, so
  // there is no reason to run it any hotter than necessary.
  mix = (mix * 255) / 60;
  if (mix > 255) mix = 255;
  return (uint8_t)mix;
}

// Core 0 audio loop. Runs only while a POKEY cart is playing; returns never.
static inline void pokey_run(void) {
  pokey_audio_init();
  const uint32_t period_us = 1000000u / POKEY_SAMPLE_RATE;   // 31us at 32kHz
  uint32_t next = time_us_32();
  while (1) {
    uint32_t now = time_us_32();
    if ((int32_t)(now - next) >= 0) {
      next += period_us;
      pwm_set_gpio_level(POKEY_AUDIO_PIN, pokey_next_sample());
    }
  }
}

#endif // POKEY_H
