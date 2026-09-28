#!/usr/bin/env bash
# Builds PicoA10400 and/or Pico2A10400 sketches and installs the resulting
# .uf2 files at the repo root, backing up whatever was there first.
#
# Deliberately does NOT pass -b/--fqbn on the command line: board options
# (including the flash= split that reserves space for the ROM filesystem)
# live in each sketch's sketch.yaml instead. A bare --fqbn rp2040:rp2040:rpipico
# reserves zero bytes for that filesystem and silently produces a firmware
# with an empty USB drive - see CLAUDE.md, "Build and flash".
#
# sketch.yaml is NOT tracked by git (checked: absent from the whole history),
# so a fresh checkout of these sources has neither file and arduino-cli fails
# with "Missing FQBN". This script is the single source of truth for the two
# FQBN strings below and writes sketch.yaml on demand if it is missing, so it
# works the same whether or not that file happens to exist already.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

# Known-good FQBNs, byte-for-byte what used to live only in the untracked
# sketch.yaml files. boot2=boot2_w25q128jvxq_4_padded_checksum is verified
# identical to the boot2 stage in upstream's published PicoA10400.ino.uf2
# (commit e0f8c66c) - do not change it without re-running that comparison.
PICOA10400_FQBN="rp2040:rp2040:generic:flash=16777216_15728640,usbstack=tinyusb,boot2=boot2_w25q128jvxq_4_padded_checksum"
PICO2A10400_FQBN="rp2040:rp2040:rpipico2:flash=4194304_3145728,usbstack=tinyusb"
# Same sketch on an RP2350A clone that carries a 16MB flash chip instead of the
# Pico 2's 4MB. The "rpipico2" board profile only offers 4MB options, so the
# 16MB filesystem needs the Generic RP2350 profile. The program region is
# IDENTICAL to the 4MB profile's (_FS_start = 0x100FE000, 1040384 bytes for the
# sketch in both); only _FS_end moves, 0x103FE000 -> 0x10FFE000. Verified on the
# built .elf: not one instruction differs, only data-pointer literals (the USB
# product string is shorter) and the seven _FS_end constants.
PICO2A10400_16MB_FQBN="rp2040:rp2040:generic_rp2350:flash=16777216_15728640,variantchip=RP2350A,freq=150,psramcs=GPIOnone,usbstack=tinyusb"

# Writes sketch.yaml only if it does not already exist - never overwrites a
# file that might carry a deliberate local customization.
ensure_sketch_yaml() {
  local dir="$1"
  local fqbn="$2"
  local yaml="$REPO_ROOT/$dir/sketch.yaml"
  if [[ -f "$yaml" ]]; then
    return
  fi
  echo "  sketch.yaml missing for ${dir} - creating it (default_fqbn: ${fqbn})"
  printf 'default_fqbn: %s\n' "$fqbn" > "$yaml"
}

usage() {
  cat <<'EOF'
Usage: ./build.sh <target>

Builds one or both PicoA10400 sketches and installs the resulting .uf2
files at the repo root (backing up whatever was there first).

Targets:
  PicoA10400        Build only the RP2040 "purple clone" board
  Pico2A10400       Build only the RP2350 (Pico 2) board
  all               Build both boards
  Pico2A10400-16MB  Build Pico2A10400 for an RP2350A clone with 16MB flash
                    (15MB of ROM storage instead of 3MB). Installs as
                    Pico2A10400.ino_16MB.uf2 - do NOT flash it to a real Pico 2.
  Pico2A10400-16MB-NTSC
                    The same board with the NTSC menu kernel. Installs as
                    Pico2A10400.ino_16MB_NTSC.uf2. The Pico 2 board therefore
                    has four images: PAL and NTSC, each 4MB and 16MB.
  PicoA10400-NTSC   Build the NTSC-menu-kernel variant of PicoA10400
  Pico2A10400-NTSC  Build the NTSC-menu-kernel variant of Pico2A10400
  all-NTSC          Build both boards' NTSC variant
  PicoA10400-E3     Diagnostic: synthesis runs, audio pin never driven (SPENT)
  PicoA10400-E4     Diagnostic: core 0 never synthesises, bus side untouched
  PicoA10400-E5     Diagnostic: bus side runs the plain (non-POKEY) loops
  PicoA10400-E6     Diagnostic: flat loop holds OE across reads (FAILED, see docs)
  PicoA10400-E7     Diagnostic: flat loop needs THREE matching address samples
  PicoA10400-E9     Diagnostic (0.47): the SuperGame/RAM loop's POKEY window
                    captures with the same bounded end-of-cycle scan the flat
                    loop uses, instead of one early sample. Aimed at the noise
                    reported on "LZSS Player" and "Portal" - the only two dual
                    carts that run that loop. WATCH THE PICTURE TOO: a bounded
                    wait dropped into these loops once cost MARIA its data.
  PicoA10400-E10    Diagnostic (0.47): render chip 1 ($0450) only, chip 0
                    ($0440) muted - 0.46's audio with 0.47's bus side. Says
                    whether the noise is what chip 0 receives.
  PicoA10400-E11    Diagnostic (0.47): the mirror of E10 - chip 0 ($0440) only,
                    chip 1 muted. On "LZSS Player" this separates "chip 0's
                    data is broken" from "both chips' data is broken", because
                    both read their registers out of the same on-cart RAM.
  PicoA10400-E12    Diagnostic (0.47): emulate_supercart_ram_pokey() confirms
                    the address with a SECOND matching sample before decoding
                    anything below $8000. That loop has always used one sample;
                    the flat loop has needed two since 0.16, and a mis-sampled
                    address serves the wrong ram_table byte - which on "LZSS
                    Player" IS a POKEY register.
  PicoA10400-E13    Diagnostic (0.47): pokey_window_service() becomes
                    LISTEN-ONLY for a POKEY cart - never drives the data bus,
                    ignores R/W, captures with the flat loop's end-of-cycle
                    scan. Makes "LZSS Player" and "Portal" behave exactly like
                    the seven dual carts that are clean. YM2151 carts are
                    untouched (they must answer reads).
                    (E14 - confirming the window address - is PRODUCTION
                    since 0.48, so it has no switch. E15 was E13+E14 and came
                    back WORSE than E14 alone; with the guard shipped, -E13 now
                    reproduces exactly that. See pokey.h.)
  PicoA10400-E8-2MA / -4MA / -8MA / -12MA
                    Diagnostic: data-line drive strength. 4mA is the RP2040
                    default, i.e. electrically what production already does, so
                    that build is the control. 2mA was tried first and made
                    everything worse, which is why the higher ones exist.
  PicoA10400-BSHI   Diagnostic: Bankset with the HALT sense INVERTED. Production
                    assumes /HALT is asserted LOW (Maria pulls it down to take
                    the bus). If every Bankset title comes up dead on hardware,
                    this build is the other half of that coin - see
                    TEST_ROMS_BANKSET/README.md. The Pico2A10400-BSHI target
                    exists for symmetry only: that board has BANKSET_HAS_HALT 0,
                    so the polarity is never read and the binary comes out
                    identical to production.
  PicoA10400-BSSA   Diagnostic: Bankset with the HALT line IGNORED - every board
                    mapped correctly but always served from Sally's half, which
                    is what the Pico 2 gets permanently. Together with -BSHI it
                    tells "HALT is inverted" apart from "HALT is dead".
  PicoA10400-BSCH   Diagnostic: Bankset with the HALT selection applied ONLY in
                    $E000-$E7FF, the character generator the official demo draws
                    from. The CPU always gets Sally's half, so the demo is
                    guaranteed to run and paint its purple background; whether
                    the TEXT appears answers "is HALT usable at all?".
  PicoA10400-BSTR   Candidate fix: keep the data output LIVE for the whole bus
                    cycle, re-reading HALT each turn, the way the real
                    combinational board does - instead of sampling HALT once and
                    holding the byte.
  PicoA10400-BSK<n> (any n, e.g. -BSK2 -BSK4 -BSK8 -BSK16)
                    Sweep of the HALT hysteresis depth. N=8 is the production
                    default since 2026-09-02 (N=2 hung, N=8 worked on the demos
                    AND on StoneAge); these targets exist to map the band and to
                    bisect if a future title misbehaves. Rule: trust HALT only after it has stayed
                    low for N consecutive accesses (N = 2 and 8). BSCH proved
                    HALT reads correctly while MARIA fetches, and BSTR proved
                    the problem is not WHEN inside the cycle we sample - so what
                    is left is Sally still using the bus at the START of a HALT
                    period, which only a duration rule can exclude.
  PicoA10400-BSAU<n>  Diagnostic: after the n-th captured POKEY write, force an
                    audible tone and stop capturing. The threshold is the value,
                    so it can be swept. Pick it well above the cart's init
                    routine: "Bankset Test 2x32K RAM Pokey800" writes exactly 29
                    on startup, so -BSAU64 sits only 35 above that and a handful
                    of stray captures already clears it. -BSAU4096 can only be
                    reached by something writing continuously, i.e. a player.
  PicoA10400-BSP1   Candidate fix: in the banked-RAM loop, handle the aux-chip
                    window on the FIRST bus sample instead of waiting for two
                    matching address samples - which is what the pre-reshape
                    loop did, and that loop produced sound.
  PicoA10400-BSA2   Diagnostic: force an audible tone the first time a write to
                    an AUDCx volume register arrives with a non-zero volume -
                    the only kind of write that can make a sound at all. Tone
                    means the player's volume commands do reach the register
                    file; silence means only zeroes are getting there.
  PicoA10400-BSA3   Diagnostic: force a tone on the first captured write whose
                    DATA BYTE is non-zero, whatever register it targets. The
                    cart's init routine writes nothing but zeroes, so it cannot
                    trigger it. Silence means every byte we sample off the bus
                    is $00 - i.e. we sample before the CPU drives the data.
  PicoA10400-BSA5   Diagnostic: force a tone on the first captured write to an
                    AUDCx register carrying ANY non-zero byte - a weaker test
                    than -BSA2, which asked for a non-zero VOLUME nibble.
  PicoA10400-BSNA   Cross-test: run the banked-RAM Bankset cart through the
                    ordinary emulate_normala78_pokey() - the loop that plays
                    "Black Lamp Music Demo (800)" - with romLen halved. The
                    picture will be wrong; judge the sound only.
  PicoA10400-BSA6   Diagnostic: force a tone after 4096 captures whose DATA BYTE
                    is non-zero. -BSA3 asked the same with a threshold of 1 and
                    so fired on the init routine, which writes SKCTL=$03; 4096
                    is out of that routine's reach (29 writes).
  PicoA10400-HSCP   Diagnostic (0.56): writes the whole High Score Cart decision
                    chain INTO /.save_hsc.data - once at game start and then
                    every 5 seconds with the live count of NVRAM writes core 1
                    has seen. Answers "why is nothing happening" without a
                    serial port, which is what a cartridge in an Atari has.
                    NOT for playing: one flash erase every 5 seconds, capped at
                    200, and the record overwrites the last 32 bytes of the
                    table.
  PicoA10400-NVSF   Diagnostic (0.60): replaces the version string in the menu
                    footer with the save-slot decision, so it can be read off
                    the TV instead of by carrying the cartridge to a PC.
                    "SAVE F10 H10" is the good answer - both save files
                    accepted, backend 1 (file), fail code 0. The first digit of
                    each pair is the backend (0 none, 1 file, 2 reserved
                    sector), the second is the step that refused it (0 accepted,
                    1 open, 2 size, 3 contiguous, 4 bounds, 5 mapping). 0.58 and
                    0.59 on the console showed "SAVE F20 H00". Otherwise a
                    completely normal build - it writes nothing and plays.
  PicoA10400-HSCN   Diagnostic (0.52): serves both High Score Cart windows and
                    counts the NVRAM writes, but NEVER touches flash. Splits "the
                    two new bus windows broke the game or the picture" from
                    "writing flash under a live bus broke it" - the same split
                    FA2N draws for FA2. Scores work inside one session and are
                    gone after a power cycle, by design.
  PicoA10400-FA2T   Diagnostic (0.50): puts the FA2 flash-write measurement in the
                    menu footer instead of the version string - "E045P007 N03"
                    means 45 ms to erase, 7 ms to program, 3 saves so far, worst
                    times seen. This is the number TODO.md phase 0 asked for, and
                    it can only be read off the TV, because the cartridge is in
                    the Atari and not in a PC while it is measured.
  PicoA10400-FA2N   Diagnostic (0.50): the FA2 handshake and the full busy delay,
                    but NO flash access at all. Splits the two suspects this
                    feature introduces - "the new bus branch or the core-0
                    handover broke the picture" versus "erasing flash under a
                    live bus broke it". Behaves like 0.45 (nothing persists) with
                    0.50's timing. If the picture is clean here and broken in
                    production, the flash primitive is the culprit and nothing
                    else is.
  PicoA10400-DIGI   Feature build: timestamped queue for sample ("digi") playback
  PicoA10400-AUDX   Diagnostic (menu hum on a 7800, MENU_FIX.md 2 / TODO.md 9):
                    rotates the idle state of cartridge pin 18 every 4s while
                    the menu is on screen and names the phase in the footer -
                    "AUD0 INPUT" (the pin as every build so far leaves it, i.e.
                    the control), "AUD1 LOW" (driven to 0V, what a silent
                    POKEY/SN presents), "AUD2 PWM MID" (mid rail, what the YM
                    calls silence). Listen for which phase goes quiet; that
                    names the fix. One flash answers what -AUD0/-AUD1/-AUD2
                    would need three for.
  PicoA10400-AUD0/-AUD1/-AUD2
                    The same three states, each nailed down for a whole build,
                    for confirming the AUDX result or for testing inside a game
                    (the rotation only runs while the MENU is up).
                    On Pico2A10400 all four are a NO-OP by construction:
                    AUDIO_PIN_ROUTED is 0 there because connector pin 18 reaches
                    no Pico pad at all (patch 0.31), so the image comes out
                    identical to production. The targets exist only so the two
                    sketches stay in step.
  Pico2A10400-E3..E7, -E8-2MA/-4MA/-8MA/-12MA, -DIGI   The same for the Pico 2

  PicoA10400-MENU-<tag> / Pico2A10400-MENU-<tag> / all-MENU-<tag>
                    Menu-kernel variant: rom.h is generated for the compile from
                    menu_rom_analysis/menu_kernel_<tag>.bin (make_rom_h.py, tag
                    in lower case) and restored afterward, exactly like -NTSC.
                    Installs as <Sketch>.ino_MENU_<TAG>.uf2. For testing a patched
                    menu on hardware before it goes into rom.h - e.g. -MENU-HMOVE
                    (TODO.md 10, patch_hmove_every_line.py).

The -NTSC targets swap in <sketch>/rom_ntsc.h for the compile (rom.h is always
restored afterward, even on failure) and install as <Sketch>.ino_NTSC.uf2,
next to (not instead of) the PAL .uf2. See menu_rom_analysis/README.md for
what makes the NTSC kernel different from the PAL one - it is not a simple
recompile flag, the menu cartridge image itself differs.

The -E3..-E7 targets are DIAGNOSTIC builds for the POKEY picture-artifact
investigation. They define POKEY_DIAG_E3..E7 (all default to 0, so an
ordinary build is unaffected) and install as <Sketch>.ino_<E>.uf2 - next to,
never instead of, the production .uf2. What each one changes and how to read
its result is in PicoA10400/pokey.h under "DIAGNOSTIC BUILD SWITCHES"; the ROM
sets and decision tables are in TEST_ROMS_E3/, TEST_ROMS_PASEK/ (E4/E5) and
TEST_ROMS_E6/. There is no -E2: that experiment succeeded and became production
(POKEY_CLOCK_KHZ in pokey.h); TEST_ROMS_E2/ keeps its record. These are
experiments, not releases - none belongs in patches/ as a version.

E3, E4 and E5 produce NO SOUND by design. E6 is the first diagnostic build that
does NOT go silent: it changes only when the flat loop drives the data lines,
not what it serves, so a POKEY cart still plays - which makes "the music still
works" part of its pass criteria. E3 on Pico2A10400 is a no-op (that board's
audio pin is the on-board LED and pin 18 is not routed, patch 0.31); E4, E5 and
E6 are meaningful there.

Options:
  -h, --help    Show this help and exit

Examples:
  ./build.sh all
  ./build.sh PicoA10400
  ./build.sh Pico2A10400
  ./build.sh all-NTSC
EOF
}

if [[ $# -eq 0 ]]; then
  usage
  exit 1
fi

case "$1" in
  -h|--help)
    usage
    exit 0
    ;;
esac

if [[ $# -ne 1 ]]; then
  echo "ERROR: expected exactly one target, got $#: $*" >&2
  echo >&2
  usage >&2
  exit 1
fi

TARGET_LOWER="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
NTSC=0
PICO2_16MB=0 # 1 = build Pico2A10400 for a 16MB RP2350A clone, not a 4MB Pico 2
DIAG=""      # "", or "E3".."E7"/"E9".."E13" - see the notes in usage()
# <board>-menu-<tag>: build against menu_rom_analysis/menu_kernel_<tag>.bin
if [[ "$TARGET_LOWER" =~ ^(picoa10400|pico2a10400|all)-menu-([a-z0-9_]+)$ ]]; then
  case "${BASH_REMATCH[1]}" in
    picoa10400)  BUILD_PICOA10400=1; BUILD_PICO2A10400=0 ;;
    pico2a10400) BUILD_PICOA10400=0; BUILD_PICO2A10400=1 ;;
    all)         BUILD_PICOA10400=1; BUILD_PICO2A10400=1 ;;
  esac
  DIAG="MENU_$(printf '%s' "${BASH_REMATCH[2]}" | tr '[:lower:]' '[:upper:]')"
  TARGET_LOWER="menu-variant"
fi
case "$TARGET_LOWER" in
  menu-variant)     ;;
  picoa10400)       BUILD_PICOA10400=1;  BUILD_PICO2A10400=0 ;;
  pico2a10400)      BUILD_PICOA10400=0;  BUILD_PICO2A10400=1 ;;
  pico2a10400-16mb) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; PICO2_16MB=1 ;;
  pico2a10400-16mb-ntsc) BUILD_PICOA10400=0; BUILD_PICO2A10400=1; PICO2_16MB=1; NTSC=1 ;;
  all)              BUILD_PICOA10400=1;  BUILD_PICO2A10400=1 ;;
  picoa10400-menu32)       BUILD_PICOA10400=1; BUILD_PICO2A10400=0; DIAG="MENU32" ;;
  pico2a10400-menu32)      BUILD_PICOA10400=0; BUILD_PICO2A10400=1; DIAG="MENU32" ;;
  all-menu32)              BUILD_PICOA10400=1; BUILD_PICO2A10400=1; DIAG="MENU32" ;;
  picoa10400-menu32-ntsc)  BUILD_PICOA10400=1; BUILD_PICO2A10400=0; DIAG="MENU32_NTSC" ;;
  pico2a10400-menu32-ntsc) BUILD_PICOA10400=0; BUILD_PICO2A10400=1; DIAG="MENU32_NTSC" ;;
  all-menu32-ntsc)         BUILD_PICOA10400=1; BUILD_PICO2A10400=1; DIAG="MENU32_NTSC" ;;
  picoa10400-ntsc)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; NTSC=1 ;;
  pico2a10400-ntsc) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; NTSC=1 ;;
  all-ntsc)         BUILD_PICOA10400=1;  BUILD_PICO2A10400=1; NTSC=1 ;;
  picoa10400-e3)    BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E3" ;;
  picoa10400-e4)    BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E4" ;;
  picoa10400-e5)    BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E5" ;;
  picoa10400-e6)    BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E6" ;;
  picoa10400-e7)    BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E7" ;;
  picoa10400-e9)    BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E9" ;;
  picoa10400-e10)   BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E10" ;;
  picoa10400-e11)   BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E11" ;;
  picoa10400-e12)   BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E12" ;;
  picoa10400-e13)   BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="E13" ;;
  picoa10400-e8-2ma)  BUILD_PICOA10400=1; BUILD_PICO2A10400=0; DIAG="E8_2MA" ;;
  picoa10400-e8-4ma)  BUILD_PICOA10400=1; BUILD_PICO2A10400=0; DIAG="E8_4MA" ;;
  picoa10400-e8-8ma)  BUILD_PICOA10400=1; BUILD_PICO2A10400=0; DIAG="E8_8MA" ;;
  picoa10400-e8-12ma) BUILD_PICOA10400=1; BUILD_PICO2A10400=0; DIAG="E8_12MA" ;;
  picoa10400-bshi)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSHI" ;;
  picoa10400-aud0)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="AUD0" ;;
  picoa10400-aud1)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="AUD1" ;;
  picoa10400-aud2)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="AUD2" ;;
  picoa10400-audx)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="AUDX" ;;
  picoa10400-bssa)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSSA" ;;
  picoa10400-bsau)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSAU64" ;;
  picoa10400-bsau[0-9]*) BUILD_PICOA10400=1; BUILD_PICO2A10400=0
                         DIAG="BSAU${TARGET_LOWER#picoa10400-bsau}" ;;
  picoa10400-bsa2)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSA2" ;;
  picoa10400-bsa3)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSA3" ;;
  picoa10400-bsa5)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSA5" ;;
  picoa10400-bsna)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSNA" ;;
  picoa10400-bsa6)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSA6" ;;
  picoa10400-bsp1)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSP1" ;;
  picoa10400-bsch)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSCH" ;;
  picoa10400-bstr)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="BSTR" ;;
  picoa10400-bsk[0-9]*)  BUILD_PICOA10400=1; BUILD_PICO2A10400=0
                         DIAG="BSK${TARGET_LOWER#picoa10400-bsk}" ;;
  pico2a10400-bsk[0-9]*) BUILD_PICOA10400=0; BUILD_PICO2A10400=1
                         DIAG="BSK${TARGET_LOWER#pico2a10400-bsk}" ;;
  picoa10400-digi)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="DIGI" ;;
  picoa10400-hscp)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="HSCP" ;;
  pico2a10400-hscp) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="HSCP" ;;
  picoa10400-hscn)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="HSCN" ;;
  pico2a10400-hscn) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="HSCN" ;;
  picoa10400-nvsf)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="NVSF" ;;
  pico2a10400-nvsf) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="NVSF" ;;
  picoa10400-fa2t)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="FA2T" ;;
  picoa10400-fa2n)  BUILD_PICOA10400=1;  BUILD_PICO2A10400=0; DIAG="FA2N" ;;
  pico2a10400-fa2t) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="FA2T" ;;
  pico2a10400-fa2n) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="FA2N" ;;
  pico2a10400-e3)   BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E3" ;;
  pico2a10400-e4)   BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E4" ;;
  pico2a10400-e5)   BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E5" ;;
  pico2a10400-e6)   BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E6" ;;
  pico2a10400-e7)   BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E7" ;;
  pico2a10400-e9)   BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E9" ;;
  pico2a10400-e10)  BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E10" ;;
  pico2a10400-e11)  BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E11" ;;
  pico2a10400-e12)  BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E12" ;;
  pico2a10400-e13)  BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="E13" ;;
  pico2a10400-e8-2ma)  BUILD_PICOA10400=0; BUILD_PICO2A10400=1; DIAG="E8_2MA" ;;
  pico2a10400-e8-4ma)  BUILD_PICOA10400=0; BUILD_PICO2A10400=1; DIAG="E8_4MA" ;;
  pico2a10400-e8-8ma)  BUILD_PICOA10400=0; BUILD_PICO2A10400=1; DIAG="E8_8MA" ;;
  pico2a10400-e8-12ma) BUILD_PICOA10400=0; BUILD_PICO2A10400=1; DIAG="E8_12MA" ;;
  pico2a10400-bshi) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="BSHI" ;;
  pico2a10400-aud0) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="AUD0" ;;
  pico2a10400-aud1) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="AUD1" ;;
  pico2a10400-aud2) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="AUD2" ;;
  pico2a10400-audx) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="AUDX" ;;
  pico2a10400-bssa) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="BSSA" ;;
  pico2a10400-digi) BUILD_PICOA10400=0;  BUILD_PICO2A10400=1; DIAG="DIGI" ;;
  *)
    echo "ERROR: unknown target '$1'" >&2
    echo >&2
    usage >&2
    exit 1
    ;;
esac

TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP_DIR="$REPO_ROOT/.uf2-backup"

# Expected "Maximum is N bytes" program-storage figure for PicoA10400 (see
# CLAUDE.md). If this changes, the 15MB filesystem partition was not reserved
# as expected and the resulting .uf2 would boot with an empty file list.
PICOA10400_EXPECTED_MAX=1044480
# The Pico2 sketch reports the same figure under BOTH flash profiles (4MB/3MB
# and 16MB/15MB reserve exactly 1040384 bytes for the program), so one constant
# guards both - and catches a profile that forgot to reserve the filesystem.
PICO2A10400_EXPECTED_MAX=1040384

# One file swapped in for the duration of a build, and put back afterwards -
# through an EXIT trap, so a failed build (set -e aborts the script) cannot leave
# the sketch directory holding the wrong file for the next one. More than one
# file can be swapped at a time: an NTSC build of the 32-column menu needs BOTH
# rom.h (the old kernel, still the initialiser of rom_table) and menu32_rom.h.
# They are kept in a stack, because a single pair of variables is exactly how the
# first version of this lost a file: "all-menu32-ntsc" swapped for board one,
# overwrote the bookkeeping for board two, and board one's PAL header stayed NTSC.
ROMH_SWAP_TARGETS=()
ROMH_SWAP_BACKUPS=()

swap_file_for_build() {
  local target="$1" replacement="$2" what="$3"
  if [[ ! -f "$replacement" ]]; then
    echo "ERROR: $replacement not found - $what" >&2
    exit 1
  fi
  local backup
  backup="$(mktemp)"
  cp "$target" "$backup"
  cp "$replacement" "$target"
  ROMH_SWAP_TARGETS+=("$target")
  ROMH_SWAP_BACKUPS+=("$backup")
  trap 'restore_swapped_files' EXIT
  echo "  Swapped in $(basename "$replacement") as $(basename "$target") for this build."
}

restore_swapped_files() {
  local i
  for ((i = ${#ROMH_SWAP_TARGETS[@]} - 1; i >= 0; i--)); do
    cp "${ROMH_SWAP_BACKUPS[i]}" "${ROMH_SWAP_TARGETS[i]}"
    rm -f "${ROMH_SWAP_BACKUPS[i]}"
    echo "  Restored $(basename "${ROMH_SWAP_TARGETS[i]}")."
  done
  ROMH_SWAP_TARGETS=()
  ROMH_SWAP_BACKUPS=()
  trap - EXIT
}

# An NTSC build: the NTSC revision of both menu kernels. menu32_rom.h is only
# there from 0.70 on, so it is swapped when it exists - a tree without it still
# builds the 12-column menu the old way.
swap_in_ntsc_romh() {
  local sketch_dir="$1"
  swap_file_for_build "$REPO_ROOT/$sketch_dir/rom.h" "$REPO_ROOT/$sketch_dir/rom_ntsc.h" \
    "regenerate it (see menu_rom_analysis/README.md)"
  if [[ -f "$REPO_ROOT/$sketch_dir/menu32_rom.h" ]]; then
    swap_file_for_build "$REPO_ROOT/$sketch_dir/menu32_rom.h" \
      "$REPO_ROOT/$sketch_dir/menu32_rom_ntsc.h" "run menu32_rom/build_menu32_rom.py"
  fi
}

# The 32-column kernel alone, for the -MENU32-NTSC diagnostic target.
swap_in_menu32_ntsc_rom() {
  local sketch_dir="$1"
  swap_file_for_build "$REPO_ROOT/$sketch_dir/menu32_rom.h" \
    "$REPO_ROOT/$sketch_dir/menu32_rom_ntsc.h" "run menu32_rom/build_menu32_rom.py"
}

# The same swap for a menu-kernel variant: rom.h is regenerated from a .bin with
# make_rom_h.py (formatting preserved) and put back by the same trap.
swap_in_menu_kernel() {
  local sketch_dir="$1" tag="$2"
  local romh="$REPO_ROOT/$sketch_dir/rom.h"
  local bin="$REPO_ROOT/menu_rom_analysis/menu_kernel_${tag}.bin"
  if [[ ! -f "$bin" ]]; then
    echo "ERROR: menu_rom_analysis/menu_kernel_${tag}.bin not found." >&2
    exit 1
  fi
  local backup
  backup="$(mktemp)"
  cp "$romh" "$backup"
  ROMH_SWAP_TARGETS+=("$romh")
  ROMH_SWAP_BACKUPS+=("$backup")
  trap 'restore_swapped_files' EXIT
  # The array size is NOT the same on both boards since 0.73: PicoA10400 keeps
  # 144KB (an RP2040 has 264KB of SRAM in total) and Pico2A10400 runs 384KB.
  # make_rom_h.py copies the template's head verbatim, and that template carries
  # 144 - so without this argument a -MENU-<tag> build on the Pico 2 would
  # quietly hand the board a 144KB game buffer and every ROM above that would
  # start being truncated again, with nothing in the build output saying so.
  local array_kb=144
  [[ "$sketch_dir" == "Pico2A10400" ]] && array_kb=384
  python3 "$REPO_ROOT/menu_rom_analysis/make_rom_h.py" "$bin" --array-kb "$array_kb" > "$romh"
  echo "  Swapped in menu_kernel_${tag}.bin as rom.h for this build (rom_table[1024*${array_kb}], rom.h restored afterward)."
}

restore_romh_after_ntsc() {
  restore_swapped_files
}

build_sketch() {
  local sketch_dir="$1" uf2_name="$2" build_subdir="$3" fqbn="$4" expected_max="${5:-}" use_ntsc="${6:-0}" diag="${7:-}" name_suffix="${8:-}" fqbn_override="${9:-}"

  ensure_sketch_yaml "$sketch_dir" "$fqbn"

  if [[ "$use_ntsc" -eq 1 ]]; then
    swap_in_ntsc_romh "$sketch_dir"
  elif [[ "$diag" == "MENU32_NTSC" ]]; then
    swap_in_menu32_ntsc_rom "$sketch_dir"
  elif [[ "$diag" == MENU_* ]]; then
    local tag="${diag#MENU_}"
    swap_in_menu_kernel "$sketch_dir" "$(printf '%s' "$tag" | tr '[:upper:]' '[:lower:]')"
  fi

  # A diagnostic build is selected by a -D on the compile line, not by editing
  # the sources: every POKEY_DIAG_* defaults to 0 in pokey.h, so the switches sit
  # in the tree permanently (documenting the experiment and letting it be
  # re-run) while an ordinary build stays byte-identical to the released one.
  # compiler.cpp.extra_flags is the right property here and build.extra_flags
  # is not: the latter carries board configuration from the platform, and
  # overriding it would silently drop that.
  local -a extra_args=()
  # Normally NOTHING is passed here and the board options come from sketch.yaml
  # (see the header of this file). A -16MB target is the one exception: it uses a
  # different board profile than the sketch's own, so it has to say so - as a
  # COMPLETE fqbn with the flash option, never a bare one. The expected_max
  # sanity check below is what proves the filesystem was still reserved.
  if [[ -n "$fqbn_override" ]]; then
    extra_args+=(--fqbn "$fqbn_override")
  fi
  # macOS 27 no longer runs x86_64 binaries (no Rosetta 2), and the ctags that
  # arduino-cli installs for its prototype step - builtin:ctags 5.8-arduino11 -
  # exists only as x86_64, so every compile stopped with "bad CPU type in
  # executable". tools/build_native_ctags.sh builds the very same tag natively
  # into tools/.cache/ctags/; when it is there, arduino-cli is pointed at it.
  # Verified harmless before first use: the 0.73 tree compiled this way came
  # out byte-identical to the shipped PicoA10400.ino.uf2 and Pico2A10400.ino.uf2.
  if [[ -x "$REPO_ROOT/tools/.cache/ctags/ctags" ]]; then
    extra_args+=(--build-property "tools.ctags.path=$REPO_ROOT/tools/.cache/ctags")
  fi
  if [[ -n "$diag" ]]; then
    if [[ "$diag" == "MENU32" || "$diag" == "MENU32_NTSC" ]]; then
      # The 32-column menu (TODO.md 11): PlusCart's kernel in menu32_rom.h, the
      # page built by menu32.h. -DMENU32=1 is the whole switch; MENU32_NTSC also
      # swaps the NTSC revision of the kernel in, above.
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DMENU32=1")
    elif [[ "$diag" == MENU_* ]]; then
      :   # no -D: the difference is the rom.h swapped in above
    elif [[ "$diag" == "DIGI" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DPOKEY_DIGI_QUEUE=1")
    elif [[ "$diag" == BSK* ]]; then
      # Hysteresis depth is the VALUE, the way E8 carried its drive strength.
      n="${diag#BSK}"
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_STICKY_N=${n}")
    elif [[ "$diag" == "HSCP" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DHSC_DIAG_PROBE=1")
    elif [[ "$diag" == "HSCN" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DHSC_DIAG_NOSAVE=1")
    elif [[ "$diag" == "NVSF" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DNV_DIAG_FOOTER=1")
    elif [[ "$diag" == "FA2T" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DFA2_DIAG_TIMING=1")
    elif [[ "$diag" == "FA2N" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DFA2_DIAG_NOWRITE=1")
    elif [[ "$diag" == "BSP1" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_PKFIRST=1")
    elif [[ "$diag" == "BSA6" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_TONE5=4096")
    elif [[ "$diag" == "BSNA" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_NORMALLOOP=1")
    elif [[ "$diag" == "BSA5" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_TONE4=1")
    elif [[ "$diag" == "BSA3" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_TONE3=1")
    elif [[ "$diag" == "BSA2" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_TONE2=1")
    elif [[ "$diag" == BSAU* ]]; then
      # The threshold is the VALUE, so it can be swept: -BSAU64, -BSAU4096, ...
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_TONE=${diag#BSAU}")
    elif [[ "$diag" == "BSCH" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_DIAG_CHARWIN=1")
    elif [[ "$diag" == "BSTR" ]]; then
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_TRACK_HALT=1")
    elif [[ "$diag" == "BSSA" ]]; then
      # Bankset with HALT ignored - the sketch guards its own #define with
      # #ifndef, so this -D wins without touching the sources.
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_HAS_HALT=0")
    elif [[ "$diag" == "BSHI" ]]; then
      # Bankset HALT sense. The sketch guards its own #define with #ifndef, so
      # this -D wins without touching the sources - the same discipline as the
      # POKEY_DIAG_* switches.
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DBANKSET_HALT_ACTIVE_LOW=0")
    elif [[ "$diag" == AUD[0-2] ]]; then
      # Idle state of the 7800 audio line (MENU_FIX.md 2.5). The mode is the
      # VALUE, exactly as E8 carries its drive strength; the sketch guards its
      # own #define with #ifndef, so this -D wins without touching the sources.
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DAUDIO_IDLE_MODE=${diag#AUD}")
    elif [[ "$diag" == "AUDX" ]]; then
      # Mode 3: rotate the three idle states in the menu, one flash instead of
      # three. AUD0/AUD1/AUD2 stay available for a single fixed state.
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DAUDIO_IDLE_MODE=3")
    elif [[ "$diag" == E8_*MA ]]; then
      # E8 is enabled by its VALUE - the drive strength in mA. ONE -D on purpose:
      # two of them inside a single --build-property break arduino-cli's library
      # discovery (the sketch stops finding SdFat.h).
      ma="${diag#E8_}"; ma="${ma%MA}"
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DPOKEY_DIAG_E8=${ma}")
    else
      extra_args+=(--build-property "compiler.cpp.extra_flags=-DPOKEY_DIAG_${diag}=1")
    fi
  fi

  local label="${sketch_dir}"
  [[ "$use_ntsc" -eq 1 ]] && label="${sketch_dir} (NTSC)"
  [[ -n "$diag" ]] && label="${sketch_dir} (diagnostic ${diag})"
  echo "==> Building ${label}"
  local log
  log="$(mktemp)"
  (cd "$REPO_ROOT/$sketch_dir" && arduino-cli compile --export-binaries "${extra_args[@]}" .) | tee "$log"

  local built_uf2="$REPO_ROOT/$sketch_dir/build/$build_subdir/$uf2_name"
  if [[ ! -f "$built_uf2" ]]; then
    echo "ERROR: expected build output not found: $built_uf2" >&2
    rm -f "$log"
    exit 1
  fi

  if [[ -n "$expected_max" ]]; then
    local max_line max_bytes
    max_line="$(grep -oE 'Maximum is [0-9]+ bytes' "$log" | head -1 || true)"
    max_bytes="$(grep -oE '[0-9]+' <<<"$max_line" || true)"
    if [[ "$max_bytes" != "$expected_max" ]]; then
      echo "ERROR: ${sketch_dir} program storage sanity check failed." >&2
      echo "  Expected: Maximum is ${expected_max} bytes" >&2
      echo "  Got:      ${max_line:-<not found in build output>}" >&2
      echo "  The filesystem partition was probably not reserved as expected -" >&2
      echo "  see CLAUDE.md, 'Build and flash'. Refusing to install this .uf2." >&2
      rm -f "$log"
      exit 1
    fi
    echo "  Sanity check OK: ${max_line}"
  fi
  rm -f "$log"

  # arduino-cli always names the build artifact after the .ino sketch,
  # regardless of which rom.h went into it - the NTSC variant gets a
  # different name only when installed at the repo root, following
  # upstream's own "<Sketch>.ino_NTSC.uf2" convention (not "_NTSC.ino.uf2").
  # The region (PAL/NTSC) and the board variant (e.g. 16MB) are INDEPENDENT
  # axes, so the name has to carry both - "<Sketch>.ino_16MB_NTSC.uf2", the same
  # order the existing _MENU32_NTSC images use. Treating NTSC as the winner,
  # which is what the first version of this did, makes a 16MB NTSC build install
  # itself over the 4MB NTSC image under that image's own name.
  local variant_tag=""
  [[ -n "$diag" ]] && variant_tag="$diag"
  [[ -z "$variant_tag" && -n "$name_suffix" ]] && variant_tag="$name_suffix"
  local install_tag="$variant_tag"
  if [[ "$use_ntsc" -eq 1 ]]; then
    install_tag="${install_tag:+${install_tag}_}NTSC"
  fi

  local install_name="$uf2_name"
  if [[ -n "$install_tag" ]]; then
    install_name="${uf2_name%.ino.uf2}.ino_${install_tag}.uf2"
  fi

  local target="$REPO_ROOT/$install_name"
  if [[ -f "$target" ]]; then
    mkdir -p "$BACKUP_DIR"
    local backup="$BACKUP_DIR/${install_name%.uf2}.${TIMESTAMP}.uf2"
    cp "$target" "$backup"
    echo "  Backed up existing $install_name -> ${backup#$REPO_ROOT/}"
  fi

  cp "$built_uf2" "$target"
  echo "  Installed $install_name"

  if [[ "$use_ntsc" -eq 1 ]]; then
    # arduino-cli names the artifact after the SKETCH, not after the rom.h that
    # went into it, so an NTSC build leaves an NTSC image sitting in the build
    # directory under the PAL name - and "all" followed by "all-NTSC" therefore
    # ends with the PAL artifact overwritten by the NTSC one. Flashing from the
    # build directory then silently gets you the wrong region: on a PAL console
    # the NTSC menu kernel paints the background with $80 instead of $B0, i.e.
    # red instead of blue. Rename it so the name always matches the content.
    mv -f "$built_uf2" "${built_uf2%.ino.uf2}.ino_${install_tag}.uf2"
    echo "  Renamed the build artifact to $(basename "${built_uf2%.ino.uf2}.ino_${install_tag}.uf2") (it is NOT the PAL image)"
    restore_romh_after_ntsc
  elif [[ -n "$diag" ]]; then
    # Same trap as the NTSC case above and the same fix: arduino-cli names the
    # artifact after the SKETCH, so a diagnostic build leaves a diagnostic image
    # sitting in the build directory under the production name. Flashing from
    # there would then silently get you E4 or E5 - both SILENT BY DESIGN, which
    # is a miserable thing to debug by accident.
    #
    # The .elf/.bin/.map go with it, and that half is not cosmetic: the default
    # argument of PicoA10400_tune/tools/check_hotpath.sh is exactly this .elf
    # path, so leaving a diagnostic .elf under the production name means the
    # next hot-path audit silently inspects the wrong binary. That happened
    # once while 0.33 was being prepared - the audit reported the POKEY bus
    # loops missing, which was true of E5 and not of production.
    for ext in uf2 elf bin map; do
      local src="${built_uf2%.uf2}.${ext}"
      [[ -f "$src" ]] && mv -f "$src" "${built_uf2%.ino.uf2}.ino_${diag}.${ext}"
    done
    echo "  Renamed the build artifacts (uf2/elf/bin/map) to *.ino_${diag}.* (they are NOT the production image)"
    # MENU32_NTSC swapped menu32_rom.h, not rom.h, but the same helper puts it
    # back - and it MUST be called per board: with "all-menu32-ntsc" the first
    # board's header would otherwise stay swapped for good, and the next PAL
    # build would silently carry the NTSC kernel. That happened once.
    if [[ "$diag" == MENU_* || "$diag" == "MENU32_NTSC" ]]; then
      restore_romh_after_ntsc
    fi
  fi
}

if [[ "$BUILD_PICOA10400" -eq 1 ]]; then
  build_sketch "PicoA10400" "PicoA10400.ino.uf2" "rp2040.rp2040.generic" "$PICOA10400_FQBN" "$PICOA10400_EXPECTED_MAX" "$NTSC" "$DIAG"
fi
if [[ "$BUILD_PICO2A10400" -eq 1 ]]; then
  if [[ "$PICO2_16MB" -eq 1 ]]; then
    build_sketch "Pico2A10400" "Pico2A10400.ino.uf2" "rp2040.rp2040.generic_rp2350" \
                 "$PICO2A10400_FQBN" "$PICO2A10400_EXPECTED_MAX" "$NTSC" "$DIAG" \
                 "16MB" "$PICO2A10400_16MB_FQBN"
  else
    build_sketch "Pico2A10400" "Pico2A10400.ino.uf2" "rp2040.rp2040.rpipico2" \
                 "$PICO2A10400_FQBN" "$PICO2A10400_EXPECTED_MAX" "$NTSC" "$DIAG"
  fi
fi

echo "==> Done."
