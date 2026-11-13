// menu32.h - the 32-column menu page, built by the Pico for the Atari to run.
//
// This is a port of PlusCart's createMenuForAtari()
// (ORIG/United-Carts-of-Atari/source/STM32firmware/PlusCart/Src/cartridge_firmware.c,
// GPL-3) to our firmware, driven by our own file list. It emits an F4-style
// 32KB image into rom_table:
//
//   bank 0          menu32_rom[] (menu32_rom/menu32_hr.asm), $FF fill, the
//                   command byte at $1FE6, seven status bytes at $1FE7-$1FED and
//                   the end-of-bank stub with the vectors at $1FEE-$1FFF
//   banks 1-3       the page as drawn in even frames
//   banks 4-6       the same page as drawn in odd frames
//   bank 7          spare
//
// WHY THE PAGE IS CODE AND NOT DATA. A 2600 cannot read a character cell from
// memory and draw it in the same scanline - our old 12-column kernel spends the
// whole line on LDA (zp),Y. PlusCart's trick is to have no load at all: every
// scanline of every row is its own unrolled code with the glyph bytes sitting in
// it as `LDA #` operands, and two mid-line RESP0 strobes that let the same
// player draw again. Eight GRP writes per line, two glyphs of 4 pixels per byte,
// A and B lines interleaved and swapped every frame: 32 characters per row.
// The price is that the "page" is ~700 bytes of code per row, which is why it
// lives in rom_table (144KB, and a loaded game overwrites it anyway) and why
// changing pages means rebuilding it.
//
// The row layout is fixed by the kernel in bank 0; what this file decides is the
// content: how many rows a page has (MENU32_SPACING), the typeface, the colours
// and the text.
#ifndef MENU32_H
#define MENU32_H

#include "menu32_rom.h"   // bank 0 image + the addresses inside it (build_menu32_rom.py)
#include "menu32_font.h"  // the four typefaces (dump of PlusCart's font.c)

// --- what the menu looks like -----------------------------------------------
// Typeface and density were chosen on the user's TV from TEST_ROMS_MENU32 (see
// MENU_UPGRADE.md, section H): image "06" - Glacier Belle, 10 entries per page.
// Going back to image "04" (14 entries) is MENU32_SPACING 0; nothing else has to
// change, the kernel in bank 0 is the same.
// Both can be overridden from the build line (build.sh -MENU32-<font>-<spacing>
// and tools/menu32/verify_port.py use that to check every combination).
#ifndef MENU32_FONT
#define MENU32_FONT       3   // 0 Small Caps, 1 Trichotomic-12, 2 Captain Morgan, 3 Glacier Belle
#endif
#ifndef MENU32_SPACING
#define MENU32_SPACING    2   // 0 dense = 14 rows, 1 regular = 12, 2 sparse = 10
#endif

#define MENU32_COLS       32
#define MENU32_MAX_ROWS   15  // header + at most 14 entries
#define MENU32_IMAGE_SIZE (32 * 1024)

// MENU32_COMPAT 1 drops everything we add on top of PlusCart (the footer, our
// colours, the "/" on directories) so that tools/menu32/verify_port.py can keep
// proving that the page generator itself is still PlusCart's, byte for byte.
#ifndef MENU32_COMPAT
#define MENU32_COMPAT 0
#endif

// Colours. Backgrounds and the text colour are PlusCart's; the rest was picked
// on the user's own PAL TV (hue 5 is green there, hue 2 yellow, $66 the closest
// to red - the palette measurement in CLAUDE.md):
//   DIR     light green, so a directory is recognisable without the cursor
//   CURSOR  yellow, patched into the kernel's `lda #SEL_COL` (SetActiveItem)
//   OVER    file larger than rom_table, or an ARM cartridge (0.76). PAL $64,
//           picked on the TV from TEST_ROMS_KOLORY page 04 (2026-11-13: "best on
//           the blue background"); until 0.78 it was $68, the 12-column menu's red
//           made brighter, which the user found too weak
// A row under the cursor keeps its MEANING and changes only its shade: on the
// bright yellow bar a colour of similar luminance would vanish (that is what
// happened to the red of an oversized file), so each kind has a dark variant of
// its own hue for the highlighted row. Painting that row black instead would
// read well but would throw the information away - the colour is what says
// "directory" or "this file does not fit".
// Every one of them can be overridden from the build line, which is how
// tools/build_test_roms_colours.py puts whole palettes side by side on the TV
// instead of anyone guessing what a hue looks like after RF, a converter and an
// LCD (MENU_COLOURS.md).
#ifdef MENU32_NTSC
 #ifndef MENU32_COL_BACK
  #define MENU32_COL_BACK     0x92
 #endif
 #ifndef MENU32_COL_TEXT
  #define MENU32_COL_TEXT     0x0A
 #endif
 #ifndef MENU32_COL_DIR
  #define MENU32_COL_DIR      0xC6
 #endif
 #ifndef MENU32_COL_OVER
  #define MENU32_COL_OVER     0x3A
 #endif
 #ifndef MENU32_COL_CURSOR
  #define MENU32_COL_CURSOR   0x1A
 #endif
 #ifndef MENU32_COL_CUR_TEXT
  #define MENU32_COL_CUR_TEXT 0x00   // ordinary file, highlighted
 #endif
 #ifndef MENU32_COL_CUR_DIR
  #define MENU32_COL_CUR_DIR  0xC4   // directory, highlighted
 #endif
 #ifndef MENU32_COL_CUR_OVER
  #define MENU32_COL_CUR_OVER 0x34   // oversized, highlighted
 #endif
 #ifndef MENU32_COL_FOOT_BG
  #define MENU32_COL_FOOT_BG  0x90
 #endif
#else
 #ifndef MENU32_COL_BACK
  #define MENU32_COL_BACK     0xD2
 #endif
 #ifndef MENU32_COL_TEXT
  #define MENU32_COL_TEXT     0x0A
 #endif
 #ifndef MENU32_COL_DIR
  #define MENU32_COL_DIR      0x56
 #endif
 #ifndef MENU32_COL_OVER
  #define MENU32_COL_OVER     0x64
 #endif
 #ifndef MENU32_COL_CURSOR
  #define MENU32_COL_CURSOR   0x2A
 #endif
 #ifndef MENU32_COL_CUR_TEXT
  #define MENU32_COL_CUR_TEXT 0x00   // ordinary file, highlighted
 #endif
 #ifndef MENU32_COL_CUR_DIR
  #define MENU32_COL_CUR_DIR  0x54   // directory, highlighted
 #endif
 #ifndef MENU32_COL_CUR_OVER
  #define MENU32_COL_CUR_OVER 0x64   // oversized, highlighted
 #endif
 #ifndef MENU32_COL_FOOT_BG
  #define MENU32_COL_FOOT_BG  0xB0
 #endif
#endif
#if MENU32_COMPAT
 #undef  MENU32_COL_DIR
 #define MENU32_COL_DIR     0x2A
#endif

// The footer row: page number and how much of the directory is on screen. It
// costs one entry, not one row - the page keeps the eleven rows whose spacing
// was chosen on the TV, and the kernel's own line budget is untouched.
#define MENU32_FOOTER (MENU32_COMPAT ? 0 : 1)

// Entry kinds our list has; they only pick a colour.
#define MENU32_KIND_FILE  0
#define MENU32_KIND_DIR   1
#define MENU32_KIND_UP    2
#define MENU32_KIND_OVER  3   // file larger than rom_table: loaded truncated
#define MENU32_KIND_ARM   4   // ARM-coprocessor cartridge: refused (arm_cart.h); drawn as OVER

// Comms, as the kernel uses them (menu32_rom/menu32_hr.asm).
#define MENU32_CMD_BYTE      0x1FE6  // the Atari WRITES a command byte here
#define MENU32_STATUS_BASE   0x1FE7  // seven status bytes the Atari reads
#define MENU32_MARQUEE_BASE  0x1FC0  // the Atari reads $1FC0+row once per frame
#define MENU32_BANK_BASE     0x1FF4  // F4 hotspots, $1FF4+n selects bank n

// Commands (values the kernel writes to $1FE6)
#define MENU32_CMD_FIRSTBOOT 0xFF
#define MENU32_CMD_PAGE_PREV 0x10
#define MENU32_CMD_PAGE_NEXT 0x20
#define MENU32_CMD_START     0x30
// 0x00-0x0D: the entry selected on the current page

// Status byte indices (offsets from MENU32_STATUS_BASE)
#define MENU32_ST_REBOOT     0
#define MENU32_ST_CURPAGE    1
#define MENU32_ST_MAXPAGE    2
#define MENU32_ST_ITEMS      3
#define MENU32_ST_PAGETYPE   4

typedef struct {
  const char *name;      // shown truncated to 32 characters
  unsigned char kind;    // MENU32_KIND_*
} menu32_entry;

// Where each row's code sits, so one row can be redrawn on its own (marquee).
typedef struct {
  unsigned short row_off[2][MENU32_MAX_ROWS];  // [frame parity][row], 0 = not present
  unsigned short col_off[2][MENU32_MAX_ROWS];  // the row's text-colour byte, for the cursor
  unsigned char rows;                          // header + entries actually emitted
  unsigned char items;                         // entries on this page, header not counted
} menu32_page_map;

// ---------------------------------------------------------------------------
// The byte templates. These are PlusCart's, with the three addresses that point
// into bank 0 taken from menu32_rom.h instead of being written out by hand:
// PlusCart's own firmware still carries `jmp $106B` for DoStart, which moved to
// $1077 in the revision we assemble.
// ---------------------------------------------------------------------------
#define MENU32_LO(a) ((unsigned char)((a) & 0xFF))
#define MENU32_HI(a) ((unsigned char)((a) >> 8))

static const unsigned char menu32_t_start_bank[10] = {
  0xD8,                                                   // cld
  0x8D, 0xF4, 0xFF,                                       // sta $FFF4 (select bank 0)
  0x4C, MENU32_LO(MENU32_ROM_CONTDRAWSCREEN), MENU32_HI(MENU32_ROM_CONTDRAWSCREEN),
  0x8D, 0xF5, 0xFF                                        // sta $FFF5,x - low byte += bank
};

static const unsigned char menu32_t_end_bank[18] = {
  0x8D, 0xF4, 0xFF,                                       // sta $FFF4
  0x4C, MENU32_LO(MENU32_ROM_DOSTART), MENU32_HI(MENU32_ROM_DOSTART),
  0, 0, 0, 0, 0, 0, 0, 0,                                 // the bank hotspots themselves
  MENU32_LO(MENU32_ROM_FIRSTSTART), MENU32_HI(MENU32_ROM_FIRSTSTART),
  MENU32_LO(MENU32_ROM_FIRSTSTART), MENU32_HI(MENU32_ROM_FIRSTSTART)
};

static const unsigned char menu32_t_switch_bank[3] = {
  0x4C, MENU32_LO(MENU32_ROM_SWITCHBANK), MENU32_HI(MENU32_ROM_SWITCHBANK)
};

static const unsigned char menu32_t_prefix_a_first[7] = { 0xEA, 0xEA, 0xEA, 0xEA, 0xEA, 0xEA, 0xEA };
static const unsigned char menu32_t_prefix_a[7]       = { 0x04, 0x00, 0xEA, 0xEA, 0xEA, 0xEA, 0xEA };
static const unsigned char menu32_t_prefix_b[9]       = { 0x85, 0x2A, 0x85, 0x10, 0xEA, 0xEA, 0xEA, 0xEA, 0xEA };

// One scanline of text, "A" phase: 00--00--11--11--11----00--00--00--00
static const unsigned char menu32_t_kernel_a[48] = {
  0xA2, 0x00,        // ldx #cell   (patch 1)
  0xA9, 0x00,        // lda #cell   (patch 3)
  0x85, 0x1C,        // sta GRP1
  0xA9, 0x00,        // lda #cell   (patch 7)
  0x85, 0x1B,        // sta GRP0
  0xA0, 0x00,        // ldy #cell   (patch 11)
  0x8E, 0x1B, 0x00,  // stx.w GRP0
  0xEA,              // nop
  0xA2, 0x00,        // ldx #cell   (patch 17)
  0xA9, 0x00,        // lda #cell   (patch 19)
  0x85, 0x1C,        // sta GRP1
  0xA9, 0x00,        // lda #cell   (patch 23)
  0x8D, 0x1C, 0x00,  // sta.w GRP1
  0x85, 0x10,        // sta RESP0   - the same player draws again
  0x84, 0x1B,        // sty GRP0
  0x85, 0x10,        // sta RESP0
  0x8E, 0x1B, 0x00,  // stx.w GRP0
  0xA9, 0x00,        // lda #cell   (patch 37)
  0x8D, 0x1B, 0x00,  // sta.w GRP0
  0xA2, 0x80,        // ldx #$80
  0x86, 0x21,        // stx HMP1
  0xEA,              // nop
  0x85, 0x10         // sta RESP0
};
#define MENU32_PA_1 1
#define MENU32_PA_2 3
#define MENU32_PA_3 7
#define MENU32_PA_4 11
#define MENU32_PA_5 17
#define MENU32_PA_6 19
#define MENU32_PA_7 23
#define MENU32_PA_8 37

// The other phase: --00--00--11--11--1100--00--00--
static const unsigned char menu32_t_kernel_b[48] = {
  0xA0, 0x00,        // ldy #cell   (patch 1)
  0xA9, 0x00,        // lda #cell   (patch 3)
  0x85, 0x1C,        // sta GRP1
  0xA2, 0x00,        // ldx #cell   (patch 7)
  0xA9, 0x00,        // lda #cell   (patch 9)
  0x85, 0x1B,        // sta GRP0
  0xA9, 0x00,        // lda #cell   (patch 13)
  0x85, 0x1B,        // sta GRP0
  0xA9, 0x00,        // lda #cell   (patch 17)
  0x8D, 0x1B, 0x00,  // sta.w GRP0
  0x86, 0x1C,        // stx GRP1
  0x85, 0x10,        // sta RESP0
  0x84, 0x1C,        // sty GRP1
  0x85, 0x10,        // sta RESP0
  0xA9, 0x00,        // lda #cell   (patch 30)
  0x8D, 0x1B, 0x00,  // sta.w GRP0
  0xA9, 0x00,        // lda #cell   (patch 35)
  0x85, 0x1B,        // sta GRP0
  0xA2, 0x00,        // ldx #0
  0x86, 0x21,        // stx HMP1
  0x8D, 0x2A, 0x00,  // sta.w HMOVE
  0x8D, 0x10, 0x00   // sta.w RESP0
};
#define MENU32_PB_1 1
#define MENU32_PB_2 3
#define MENU32_PB_3 7
#define MENU32_PB_4 9
#define MENU32_PB_5 13
#define MENU32_PB_6 17
#define MENU32_PB_7 30
#define MENU32_PB_8 35

static const unsigned char menu32_t_header_bottom[12] = {
  0xA9, 0x00, 0x85, 0x1B, 0x85, 0x1C,   // lda #0 / sta GRP0 / sta GRP1
  0x85, 0x02, 0x85, 0x02,               // sta WSYNC x2
  0xA9, 0x00                            // lda #background (patch 11)
};
static const unsigned char menu32_t_header_bottom2[2] = { 0x85, 0x09 };  // sta COLUBK

static const unsigned char menu32_t_normal_bottom[10] = {
  0xA9, 0x00, 0x85, 0x1B, 0x85, 0x1C,   // blank both players
  0xE6, 0x82,                           // inc lineCounter
  0xA5, 0x00                            // lda LineBackColor+row+1 (patch 9)
};
static const unsigned char menu32_t_normal_bottom2[2] = { 0x85, 0x09 };  // sta COLUBK
static const unsigned char menu32_t_wsync[2] = { 0x85, 0x02 };

static const unsigned char menu32_t_text_colour[12] = {
  0xA9, 0x00,        // lda #colour (patch 1)
  0x85, 0x06,        // sta COLUP0
  0x85, 0x07,        // sta COLUP1
  0xA9, 0x00,        // lda #0
  0x85, 0x82,        // sta lineCounter
  0x85, 0x02         // sta WSYNC
};
static const unsigned char menu32_t_normal_top[6] = {
  0xA9, 0x00,        // lda #colour (patch 1)
  0x85, 0x06, 0x85, 0x07
};
static const unsigned char menu32_t_exit_kernel[7] = {
  0xA9, 0x00,        // lda #background (patch 1)
  0x85, 0x09,
  0x4C, MENU32_LO(MENU32_ROM_EXITKERNEL), MENU32_HI(MENU32_ROM_EXITKERNEL)
};
static const unsigned char menu32_t_zero_sprites[6] = { 0xA2, 0x00, 0x86, 0x1B, 0x86, 0x1C };

// Zero-page addresses inside the kernel (PlusCart.asm: lineCounter, LineBackColor)
#define MENU32_ZP_LINECOUNTER  0x82
#define MENU32_ZP_LINEBACKCOL  0x84

// ---------------------------------------------------------------------------
// Emitting
// ---------------------------------------------------------------------------
typedef struct {
  unsigned char *image;   // the 32KB image being built
  unsigned char *p;       // where the next byte goes
  unsigned char font;
  unsigned char spacing;
} menu32_emit;

static void menu32_put(menu32_emit *e, const unsigned char *src, unsigned len) {
  unsigned i;
  for (i = 0; i < len; i++) e->p[i] = src[i];
  e->p += len;
}

static unsigned char menu32_glyph_row(unsigned char font, unsigned char ch, unsigned char row) {
  unsigned short g;
  if (ch < MENU32_FIRST_ASCII || ch >= MENU32_FIRST_ASCII + MENU32_ASCII_SLOTS) ch = ' ';
  g = menu32_ascii_to_glyph[font][ch - MENU32_FIRST_ASCII];
  return menu32_glyphs[g * MENU32_CHAR_HEIGHT + row];
}

// One cell = two characters: left glyph in the high nibble, right in the low one.
static unsigned char menu32_cell(unsigned char font, const char *text, int left, unsigned char row) {
  return (unsigned char)((menu32_glyph_row(font, (unsigned char)text[left], row) << 4) |
                         menu32_glyph_row(font, (unsigned char)text[left + 1], row));
}

static void menu32_line_a(menu32_emit *e, const char *text, unsigned char row) {
  unsigned char *k = e->p;
  menu32_put(e, menu32_t_kernel_a, sizeof menu32_t_kernel_a);
  k[MENU32_PA_1] = menu32_cell(e->font, text, 4, row);
  k[MENU32_PA_2] = menu32_cell(e->font, text, 8, row);
  k[MENU32_PA_3] = menu32_cell(e->font, text, 0, row);
  k[MENU32_PA_4] = (unsigned char)(menu32_cell(e->font, text, 22, row) << 1);
  k[MENU32_PA_5] = (unsigned char)(menu32_cell(e->font, text, 26, row) << 1);
  k[MENU32_PA_6] = menu32_cell(e->font, text, 12, row);
  k[MENU32_PA_7] = menu32_cell(e->font, text, 16, row);
  k[MENU32_PA_8] = (unsigned char)(menu32_cell(e->font, text, 30, row) << 1);
}

static void menu32_line_b(menu32_emit *e, const char *text, unsigned char row) {
  unsigned char *k = e->p;
  menu32_put(e, menu32_t_kernel_b, sizeof menu32_t_kernel_b);
  k[MENU32_PB_1] = menu32_cell(e->font, text, 18, row);
  k[MENU32_PB_2] = menu32_cell(e->font, text, 10, row);
  k[MENU32_PB_3] = menu32_cell(e->font, text, 14, row);
  k[MENU32_PB_4] = menu32_cell(e->font, text, 2, row);
  k[MENU32_PB_5] = menu32_cell(e->font, text, 6, row);
  k[MENU32_PB_6] = menu32_cell(e->font, text, 20, row);
  k[MENU32_PB_7] = menu32_cell(e->font, text, 24, row);
  k[MENU32_PB_8] = menu32_cell(e->font, text, 28, row);
}

static void menu32_space_lines(menu32_emit *e, int extra) {
  int lines = extra + (e->spacing == 1 ? 1 : (e->spacing == 2 ? 2 : 0));
  while (lines-- > 0) menu32_put(e, menu32_t_wsync, sizeof menu32_t_wsync);
}

// The twelve scanlines of one row, in the phase this bank is drawn in.
static void menu32_row_lines(menu32_emit *e, const char *text, int parity) {
  unsigned char sc;
  for (sc = 0; sc < MENU32_CHAR_HEIGHT; sc++) {
    if ((parity + sc) & 1) {
      menu32_put(e, menu32_t_prefix_b, sizeof menu32_t_prefix_b);
      menu32_line_b(e, text, sc);
    } else {
      menu32_put(e, sc == 0 ? menu32_t_prefix_a_first : menu32_t_prefix_a, sizeof menu32_t_prefix_a);
      menu32_line_a(e, text, sc);
    }
  }
}

// ---------------------------------------------------------------------------
// A whole page
// ---------------------------------------------------------------------------
static const unsigned char menu32_items_per_page[3] = { 14, 12, 10 };

// Selectable entries on a page: the row budget of the kernel, less the footer.
#define MENU32_ITEMS (menu32_items_per_page[MENU32_SPACING] - MENU32_FOOTER)

static void menu32_text32(char *dst, const char *src) {
  int i = 0;
  if (src) {
    for (; i < MENU32_COLS && src[i]; i++) {
      unsigned char c = (unsigned char)src[i];
      // The typefaces cover ASCII 32..127 plus PlusCart's own icons; anything
      // else (a national character in a file name, a control byte) becomes a
      // space rather than a random glyph.
      dst[i] = (c < MENU32_FIRST_ASCII || c > 127) ? ' ' : (char)c;
    }
  }
  for (; i < MENU32_COLS; i++) dst[i] = ' ';
  dst[MENU32_COLS] = 0;
  dst[MENU32_COLS + 1] = 0;   // menu32_cell() reads pairs; never past the end
}

static unsigned char menu32_kind_colour(unsigned char kind) {
  switch (kind) {
    case MENU32_KIND_DIR:  return MENU32_COL_DIR;
    case MENU32_KIND_UP:   return MENU32_COL_DIR;
    case MENU32_KIND_OVER: return MENU32_COL_OVER;
    case MENU32_KIND_ARM:  return MENU32_COL_OVER;
    default:               return MENU32_COL_TEXT;
  }
}

// The same kind, as it is drawn on the cursor bar.
static unsigned char menu32_kind_cursor_colour(unsigned char kind) {
  switch (kind) {
    case MENU32_KIND_DIR:  return MENU32_COL_CUR_DIR;
    case MENU32_KIND_UP:   return MENU32_COL_CUR_DIR;
    case MENU32_KIND_OVER: return MENU32_COL_CUR_OVER;
    case MENU32_KIND_ARM:  return MENU32_COL_CUR_OVER;
    default:               return MENU32_COL_CUR_TEXT;
  }
}

static void menu32_start_bank(menu32_emit *e, int bank) {
  e->p = e->image + bank * 4096;
  menu32_put(e, menu32_t_start_bank, sizeof menu32_t_start_bank);
  // "sta $FFF5,x" -> the hotspot of the NEXT bank (low byte of the operand)
  e->p[-2] = (unsigned char)(0xF5 + bank);
}

static void menu32_end_bank(menu32_emit *e, int bank) {
  menu32_put(e, menu32_t_switch_bank, sizeof menu32_t_switch_bank);
  e->p = e->image + (bank + 1) * 4096 - (int)sizeof menu32_t_end_bank;
  menu32_put(e, menu32_t_end_bank, sizeof menu32_t_end_bank);
}

static void menu32_header_bottom(menu32_emit *e) {
  unsigned char *t = e->p;
  menu32_put(e, menu32_t_header_bottom, sizeof menu32_t_header_bottom);
  t[11] = MENU32_COL_BACK;
  menu32_put(e, menu32_t_header_bottom2, sizeof menu32_t_header_bottom2);
  menu32_space_lines(e, e->spacing == 0 ? 2 : (e->spacing == 1 ? 3 : 4));
}

// Ends the last entry row and paints the footer's own background behind the row
// that follows it. Same template as the header's bottom - in this kernel a row's
// background is always set by the row above it.
static void menu32_footer_bottom(menu32_emit *e) {
  unsigned char *t = e->p;
  menu32_put(e, menu32_t_header_bottom, sizeof menu32_t_header_bottom);
  t[11] = MENU32_COL_FOOT_BG;
  menu32_put(e, menu32_t_header_bottom2, sizeof menu32_t_header_bottom2);
  menu32_space_lines(e, 0);
}

static void menu32_normal_bottom(menu32_emit *e, int row) {
  unsigned char *t = e->p;
  menu32_put(e, menu32_t_normal_bottom, sizeof menu32_t_normal_bottom);
  t[9] = (unsigned char)(MENU32_ZP_LINEBACKCOL + row + 1);   // the row's own background
  menu32_space_lines(e, (e->font == 0 && e->spacing == 1) ? 1 : 0);
  menu32_put(e, menu32_t_normal_bottom2, sizeof menu32_t_normal_bottom2);
}

// Returns the offset of the colour byte it just wrote, so the cursor can change
// a row's text colour later without rebuilding the page.
static unsigned short menu32_normal_top(menu32_emit *e, unsigned char colour) {
  unsigned char *t = e->p;
  menu32_put(e, menu32_t_normal_top, sizeof menu32_t_normal_top);
  t[1] = colour;
  menu32_space_lines(e, (e->font == 0 && e->spacing == 1) ? 0 : 1);
  return (unsigned short)((t - e->image) + 1);
}

static void menu32_text_colour(menu32_emit *e, unsigned char colour) {
  unsigned char *t = e->p;
  menu32_put(e, menu32_t_text_colour, sizeof menu32_t_text_colour);
  t[1] = colour;
}

static void menu32_exit_kernel(menu32_emit *e) {
  unsigned char *t;
  menu32_put(e, menu32_t_zero_sprites, sizeof menu32_t_zero_sprites);
  menu32_space_lines(e, 0);
  t = e->p;
  menu32_put(e, menu32_t_exit_kernel, sizeof menu32_t_exit_kernel);
  t[1] = MENU32_COL_BACK;
}

// What the wait routine draws while the cartridge is busy: NOTHING. The kernel
// animates a sprite there (PlusCart puts its own logo in it, and a spinner of
// ours was no better received), but a page is rebuilt in milliseconds, so the
// mark is on screen just long enough to look like a glitch. All four frames are
// blank; put a pattern back here if an indicator is ever wanted again.
static const unsigned char menu32_busy_logo[32] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Bank 0: the kernel, the command byte, the status bytes and the vectors.
static void menu32_build_bank0(unsigned char *image, int page, int max_page, int items) {
  unsigned i;
  for (i = 0; i < sizeof menu32_rom; i++) image[i] = menu32_rom[i];
  for (i = (unsigned)sizeof menu32_rom; i < 0xFE6; i++) image[i] = 0xFF;
  image[0xFE6] = 0x00;                       // command byte: written by the Atari, never read
  image[0xFE7 + MENU32_ST_REBOOT]   = 0;     // set to 1 by the firmware once a game is loaded
  image[0xFE7 + MENU32_ST_CURPAGE]  = (unsigned char)page;
  image[0xFE7 + MENU32_ST_MAXPAGE]  = (unsigned char)max_page;
  image[0xFE7 + MENU32_ST_ITEMS]    = (unsigned char)items;
  image[0xFE7 + MENU32_ST_PAGETYPE] = 0;     // directory
  image[0xFE7 + 5] = 0;
  image[0xFE7 + 6] = 0;
  for (i = 0; i < sizeof menu32_t_end_bank; i++) image[0xFEE + i] = menu32_t_end_bank[i];
#if !MENU32_COMPAT
  // Two bytes of the kernel are ours to set, and both are patched here rather
  // than assembled in, so changing them is a rebuild of the sketch and not of
  // the ROM: the cursor colour, and the animation the wait routine shows while
  // the cartridge is busy - PlusCart draws its own logo there, which has no
  // business being on this menu.
  image[MENU32_ROM_SELCOL_OFF]   = MENU32_COL_CURSOR;
  // The kernel paints the row backgrounds from constants of its own, so the
  // palette is only half ours unless those are patched too: the entry rows
  // (SetActiveItem refills them every frame), the header row, and the screen the
  // wait routine draws while the cartridge is busy.
  image[MENU32_ROM_BACKCOL_OFF]  = MENU32_COL_BACK;
  image[MENU32_ROM_BACKCOL2_OFF] = MENU32_COL_BACK;
  image[MENU32_ROM_HEADCOL_OFF]  = MENU32_COL_FOOT_BG;
  image[MENU32_ROM_HEADCOL2_OFF] = MENU32_COL_FOOT_BG;
  for (i = 0; i < MENU32_ROM_LOGO_SIZE; i++) image[MENU32_ROM_LOGO_OFF + i] = menu32_busy_logo[i];
#endif
}

// Build one page of the menu. `items`/`count` is what goes on THIS page (at most
// MENU32_ITEMS entries); the caller does the paging. `map` comes back with the
// offset of every row in both frame phases, for menu32_update_row().
static void menu32_build_page(unsigned char *image,
                              const menu32_entry *items, int count,
                              int page, int max_page,
                              const char *header, const char *footer,
                              menu32_page_map *map) {
  const int per = MENU32_ITEMS;            // selectable rows
  const int last = per + MENU32_FOOTER;    // last row drawn, footer included
  menu32_emit e;
  char text[MENU32_COLS + 2];
  int parity, entry, bank = 1, i;

  e.image = image;
  e.p = image;
  e.font = MENU32_FONT;
  e.spacing = MENU32_SPACING;

  menu32_build_bank0(image, page, max_page, count);
  for (i = 4096; i < MENU32_IMAGE_SIZE; i++) image[i] = 0xFF;

  map->rows = (unsigned char)(last + 1);
  map->items = (unsigned char)count;
  for (parity = 0; parity < 2; parity++)
    for (entry = 0; entry < MENU32_MAX_ROWS; entry++)
      map->row_off[parity][entry] = map->col_off[parity][entry] = 0;

  menu32_start_bank(&e, bank);
  for (parity = 0; parity < 2; parity++) {
    for (entry = 0; entry <= last; entry++) {
      if (entry == 0) {
        menu32_text_colour(&e, MENU32_COL_TEXT);
        menu32_text32(text, header);
      } else if (entry <= per) {
        const menu32_entry *it = (entry - 1 < count) ? &items[entry - 1] : 0;
        map->col_off[parity][entry] =
          menu32_normal_top(&e, it ? menu32_kind_colour(it->kind) : MENU32_COL_TEXT);
        menu32_text32(text, it ? it->name : 0);
      } else {
        menu32_normal_top(&e, MENU32_COL_TEXT);     // the footer row
        menu32_text32(text, footer);
      }
      map->row_off[parity][entry] = (unsigned short)(e.p - image);
      menu32_row_lines(&e, text, parity);
      if (entry == 0) {
        menu32_header_bottom(&e);
        menu32_normal_bottom(&e, 0);
      } else {
        if (entry < per)                          menu32_normal_bottom(&e, entry);
        else if (entry == per && MENU32_FOOTER)   menu32_footer_bottom(&e);
        else                                      menu32_exit_kernel(&e);
        // Five rows is what fits in a 4KB bank, so a page takes three banks per
        // frame phase; the last row of a phase has to end its bank too, or the
        // other phase would start writing into the same one.
        if (entry == 4 || entry == 9 || entry == last) {
          menu32_end_bank(&e, bank);
          bank++;
          menu32_start_bank(&e, bank);
        }
      }
    }
  }
  menu32_end_bank(&e, bank);
}

// The text colour of ONE row, changed in place. Used for the cursor: the
// highlighted row is drawn dark on the yellow bar and put back to its own colour
// when the cursor moves on. Written into the phase the console is not drawing,
// exactly like the marquee.
static void menu32_set_row_colour(unsigned char *image, const menu32_page_map *map,
                                  int parity, int row, unsigned char colour) {
  if (row < 0 || row >= map->rows || !map->col_off[parity][row]) return;
  image[map->col_off[parity][row]] = colour;
}

// Redraw the text of ONE row - this is the marquee. Only the twelve scanlines
// are rewritten, so the colours and the row's place in the page stay as they
// are. The caller passes the phase the console is NOT drawing right now (the
// firmware knows it from which bank the kernel started this frame), so the
// 6502 never reads a row while it is being rewritten.
static void menu32_update_row(unsigned char *image, const menu32_page_map *map,
                              int parity, int row, const char *name) {
  menu32_emit e;
  char text[MENU32_COLS + 2];
  if (row < 0 || row >= map->rows || !map->row_off[parity][row]) return;
  e.image = image;
  e.p = image + map->row_off[parity][row];
  e.font = MENU32_FONT;
  e.spacing = MENU32_SPACING;
  menu32_text32(text, name);
  menu32_row_lines(&e, text, parity);
}

#endif  // MENU32_H

