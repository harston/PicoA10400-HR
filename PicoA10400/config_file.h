// config_file.h - /CONFIG.TXT, the user's settings file (0.82)
//
// A plain text file in the ROOT of the cartridge's drive, read once at power-on:
// setup() calls config_load() right after the file system is mounted and BEFORE
// plus_scan(), so a /PLUS/ autostart game gets the settings too. The menu hides
// the file (is_firmware_own_file()).
//
// Format: one setting per line, "KEY = VALUE". Case does not matter, spaces and
// tabs around both are ignored, '#' or ';' starts a comment, CR/LF and a UTF-8
// BOM are accepted. Unknown keys and unknown values are ignored - a missing,
// empty or broken file simply means the defaults. Lines longer than
// CFG_LINE_MAX - 1 characters are cut; the first 4 KB of the file are read.
//
//   AUDIO_PIN = AUTO | GP25 | GP29 | BOTH | OFF        (default AUTO)
//       Which pin(s) carry the POKEY / YM2151 / SN76489 audio - see
//       audio_out.h. AUTO is the board's own choice (AUDIO_AUTO_PINS in the
//       sketch). "25"/"GPIO25" and "29"/"GPIO29" are accepted too, and NONE
//       for OFF.
//
// The parser takes the text from a buffer and knows nothing about files, so
// tools/config_selftest compiles it on the host (CONFIG_HOST_TEST) and checks it.
// Same text in both sketches.

#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

enum { AUDIO_PIN_AUTO = 0, AUDIO_PIN_GP25, AUDIO_PIN_GP29, AUDIO_PIN_BOTH, AUDIO_PIN_OFF };

static uint8_t audio_pin_setting = AUDIO_PIN_AUTO;   // AUDIO_PIN

#define CONFIG_FILE_NAME "CONFIG.TXT"
#define CFG_LINE_MAX     80
#define CFG_READ_MAX     4096

// strcmp() == 0 ignoring the case of ASCII letters.
static bool cfg_ieq(const char *a, const char *b) {
  for (;; a++, b++) {
    int ca = (unsigned char)*a, cb = (unsigned char)*b;
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return false;
    if (!ca) return true;
  }
}

static char *cfg_trim(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  size_t n = strlen(s);
  while (n && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r')) s[--n] = 0;
  return s;
}

static bool cfg_audio_pin_value(const char *v, uint8_t *out) {
  if (cfg_ieq(v, "AUTO"))                                             { *out = AUDIO_PIN_AUTO; return true; }
  if (cfg_ieq(v, "GP25") || cfg_ieq(v, "GPIO25") || cfg_ieq(v, "25")) { *out = AUDIO_PIN_GP25; return true; }
  if (cfg_ieq(v, "GP29") || cfg_ieq(v, "GPIO29") || cfg_ieq(v, "29")) { *out = AUDIO_PIN_GP29; return true; }
  if (cfg_ieq(v, "BOTH"))                                             { *out = AUDIO_PIN_BOTH; return true; }
  if (cfg_ieq(v, "OFF") || cfg_ieq(v, "NONE"))                        { *out = AUDIO_PIN_OFF;  return true; }
  return false;
}

// Line accumulator: the file is fed in chunks of any size, a line may straddle
// two of them. Lives on the caller's stack - no RAM of its own.
typedef struct {
  char    line[CFG_LINE_MAX];
  uint8_t len;
  uint8_t applied;    // settings taken
  uint8_t rejected;   // known key, unknown value
} cfg_reader_t;

static void cfg_line_done(cfg_reader_t *r) {
  r->line[r->len] = 0;
  r->len = 0;
  char *s = r->line;
  if ((uint8_t)s[0] == 0xEF && (uint8_t)s[1] == 0xBB && (uint8_t)s[2] == 0xBF) s += 3;   // UTF-8 BOM
  for (char *c = s; *c; c++)
    if (*c == '#' || *c == ';') { *c = 0; break; }
  char *eq = strchr(s, '=');
  if (!eq) return;
  *eq = 0;
  const char *key = cfg_trim(s), *val = cfg_trim(eq + 1);
  if (cfg_ieq(key, "AUDIO_PIN")) {
    uint8_t m;
    if (cfg_audio_pin_value(val, &m)) { audio_pin_setting = m; r->applied++; }
    else r->rejected++;
  }
}

static void cfg_feed(cfg_reader_t *r, const char *p, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (p[i] == '\n') cfg_line_done(r);
    else if (r->len < CFG_LINE_MAX - 1) r->line[r->len++] = p[i];
  }
}

static void cfg_finish(cfg_reader_t *r) {
  if (r->len) cfg_line_done(r);
}

#ifndef CONFIG_HOST_TEST
// Core 0, setup(), right after fatfs.begin(). One open, at most 4 KB read.
static void config_load(void) {
  FatFile f;
  if (!f.open("/" CONFIG_FILE_NAME, O_RDONLY)) return;   // no file: defaults
  cfg_reader_t r;
  memset(&r, 0, sizeof(r));
  char chunk[64];
  uint32_t total = 0;
  int got;
  while (total < CFG_READ_MAX && (got = f.read(chunk, sizeof(chunk))) > 0) {
    cfg_feed(&r, chunk, (size_t)got);
    total += (uint32_t)got;
  }
  cfg_finish(&r);
  f.close();
}
#endif

#endif // CONFIG_FILE_H
