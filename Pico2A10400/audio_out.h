// audio_out.h - the pin(s) that carry the synthesised audio (0.82)
//
// POKEY, YM2151 and SN76489 all end in the same place: a PWM output whose
// average, filtered by the console, is the sound on cartridge pin 18. Until
// 0.81 each synthesiser drove one fixed pin (PicoA10400 GPIO29, Pico2A10400
// GPIO25). Now the pin set comes from audio_pin_setting (config_file.h,
// AUDIO_PIN in /CONFIG.TXT):
//
//   AUTO  AUDIO_AUTO_PINS, a board fact the sketch defines before including
//         this header:
//           PicoA10400   GPIO29 - wired to cartridge pin 18 on the board
//           Pico2A10400  GPIO25 + GPIO29 - the board has no audio wiring of its
//                        own; the bodge goes from GPIO25 (LED pad, any Pico 2)
//                        or from header pin 35 ("A3" = GPIO29 on the black
//                        RP2350A clone) to pin 18, and driving both makes
//                        either one work with no setting at all
//   GP25, GP29, BOTH, OFF  exactly that
//
// Choosing the pin by identifying the board was considered and rejected: the
// only marker (the VSYS/3 divider a real Pico / Pico 2 has on GPIO29, read by
// the ADC) reads the console's audio input bias instead once GPIO29 IS the
// bodge, and the flash size does not identify the layout (the clone also comes
// with 4 MB). Driving a pin nobody wired is harmless on every board this runs
// on: GPIO25 is the on-board LED or a free pad; GPIO29 is a free header pin on
// the clone and the internal VSYS/3 divider (~200k/100k) on a real Pico 2 -
// tens of microamps either way.
//
// IN STEP. GPIO25 and GPIO29 sit on two PWM slices (4 B and 6 B). Started one
// after the other, the two counters would run at an arbitrary phase to each
// other, and two pins wired to one line would drive against each other on
// every edge. audio_out_start() stops and zeroes every slice it uses and then
// starts them all with ONE write to the EN register: the counters run in step
// and the pins switch on the same clock edge.
//
// Core 0 only. Same text in both sketches.

#ifndef AUDIO_OUT_H
#define AUDIO_OUT_H

#include "hardware/pwm.h"
#include "hardware/gpio.h"

#ifndef AUDIO_AUTO_PINS
#error "define AUDIO_AUTO_PINS (bit mask of GPIO25 / GPIO29) before including audio_out.h"
#endif

#define AUDIO_GP25 (1u << 25)
#define AUDIO_GP29 (1u << 29)

static uint32_t audio_pins = 0;      // the set in use, AUDIO_GP25 | AUDIO_GP29

static inline uint32_t audio_pins_for(uint8_t setting) {
  switch (setting) {
    case AUDIO_PIN_GP25: return AUDIO_GP25;
    case AUDIO_PIN_GP29: return AUDIO_GP29;
    case AUDIO_PIN_BOTH: return AUDIO_GP25 | AUDIO_GP29;
    case AUDIO_PIN_OFF:  return 0;
    default:             return (uint32_t)(AUDIO_AUTO_PINS);
  }
}

static inline void audio_out_select(void) {
  audio_pins = audio_pins_for(audio_pin_setting);
}

static uint32_t audio_out_prepare(uint pin, pwm_config *cfg, uint16_t level) {
  uint slice = pwm_gpio_to_slice_num(pin);
  pwm_init(slice, cfg, false);              // stopped, counter at 0
  pwm_set_gpio_level(pin, level);
  gpio_set_function(pin, GPIO_FUNC_PWM);
  return 1u << slice;
}

// Every selected pin gets the same slice configuration and level; all their
// slices start together.
static void audio_out_start(pwm_config *cfg, uint16_t level) {
  uint32_t slices = 0;
  if (audio_pins & AUDIO_GP25) slices |= audio_out_prepare(25, cfg, level);
  if (audio_pins & AUDIO_GP29) slices |= audio_out_prepare(29, cfg, level);
  if (slices) hw_set_bits(&pwm_hw->en, slices);
}

// What pokey_audio_init(), ym_audio_init() and sn76489_audio_init() used to do
// on one pin: no clock divider, TOP = wrap, the given starting level.
static void audio_out_init(uint16_t wrap, uint16_t level) {
  audio_out_select();
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, wrap);
  audio_out_start(&cfg, level);
}

// One sample, every selected pin.
static inline void audio_out_level(uint16_t level) {
  if (audio_pins & AUDIO_GP25) pwm_set_gpio_level(25, level);
  if (audio_pins & AUDIO_GP29) pwm_set_gpio_level(29, level);
}

#endif // AUDIO_OUT_H
