// pokey.h - POKEY audio synthesis for PicoA10400, event-driven core (v2)
//
// HISTORY: the first version of this file (see patches/ for the diff) answered
// one question on real hardware - "do POKEY games produce sound through
// cartridge pin 18?" - with a per-output-sample phase accumulator. It shipped,
// it worked on 24/24 test carts, and its own header said plainly that it was
// "deliberately NOT a faithful POKEY". This version is the fidelity pass that
// header always pointed at, written after POKEY_IMPROVEMENT.md's analysis
// found that the accumulator had TWO real bugs, not just approximations:
//   - every channel played a full OCTAVE low (the accumulator halved a
//     frequency that was already the borrow rate, not the square-wave rate);
//   - every 16-bit channel pair (AUDCTL CH1_CH2/CH3_CH4, 122/434 library
//     files) was SILENT, because the low half of the pair was given the
//     divider and the high half the output, when real POKEY does the reverse.
// Both are fixed here, along with every other simplification the old header
// listed (high-pass filters, 9-bit poly, the 5-bit poly gate's phase, two-tone
// mode, RANDOM already existed) except the deliberately-kept ones documented
// below.
//
// MODEL: per-BORROW, not per-output-sample. A real POKEY channel is an 8-bit
// down-counter that reloads and toggles on underflow ("borrow"); the four
// borrows happen at whatever rate AUDF/AUDCTL select, and everything else -
// the waveform, the noise, the high-pass filters, RANDOM - is defined in terms
// of those borrow instants. This file tracks each channel's NEXT borrow as an
// absolute tick of a virtual 1,787,520 Hz clock and jumps straight to the
// nearest one (pk_run_to()), instead of ticking that clock 1.79 million times
// a second. A typical voicing produces on the order of 3-16 THOUSAND borrows a
// second, not 1.79 million - see POKEY_IMPROVEMENT.md 4 for the measurement
// (QEMU, cycle-accurate) that a per-clock model does NOT fit the RP2040's
// budget (159 cycles/POKEY-clock vs 140 available) while this one costs under
// 3% of a core even in typical use, because the cost tracks the CONTENT, not
// the clock. Ron Fries said as much in 1997 (pokey.txt): "the routine only
// calculates new output values when a change is sensed".
//
// Landing on an EXACT borrow instant, rather than sampling at a fixed 32kHz
// grid, is also what fixes aliasing: the output sample is the time-weighted
// AVERAGE of the level over its whole window (pk_mix / elapsed), the same
// rectangular low-pass every other POKEY emulator in ORIG/ uses (JS7800's
// Pokey.js sums sub-samples and divides; test7800 keeps sampleSum/sampleSumCt)
// - it just gets there without visiting every one of the ~56 POKEY clocks a
// 32kHz sample spans.
//
// SOURCE AND FIDELITY: transcribed from
//   ORIG/MAME-A7800/src/devices/sound/pokey.cpp   (4.9, a7800 fork)
// which POKEY_IMPROVEMENT.md 6 picked over ProSystem/JS7800's Pokey.js after
// checking all four polynomial sequences bit-for-bit: JS7800 still carries the
// OLDER Ron Fries poly4/poly5 tables that MAME's own changelog says were
// replaced ("4.8: poly4/5 init routine replaced with one based on Altirra's
// implementation, which resolved [a] pitch shift issue"). tools/pokey_selftest/
// compiles that same, unmodified pokey.cpp on a host and compares step_one_
// clock()'s packed 4-channel output against this file's engine, clock for
// clock, exactly the method tools/ym2151_selftest/ used for the FM core - see
// its README for what it covers and what it deliberately does not.
//
// HARDWARE PATH (see pokey_feasibility/README.md for how this was established):
//   Atari 7800 cartridge connector pin 18 "AUD IN"  ->  net ExtAudio  ->  J2 pin 8
//   ->  Pico GPIO 29, which the firmware otherwise never uses. There is NO passive
//   conditioning anywhere on the PCB - the board carries zero resistors - so this
//   drives the console's audio input directly with PWM. The carrier is now 9-bit
//   (488kHz @250MHz) rather than the original 8-bit (977kHz): the averaged output
//   of this model has far more than 61 distinct levels (POKEY_IMPROVEMENT.md
//   5.3), and 9 bits is the same carrier YM2151 already ships on this exact pin
//   with no reported picture interference. If that changes, an RC low-pass
//   (1k + 10nF) is the fix, not a code change.
//
// KNOWN SIMPLIFICATIONS, kept deliberately because no file in the library (2600+
// 7800 ROMs scanned, see POKEY_IMPROVEMENT.md 1) is known to need them and
// reproducing them exactly costs the O(1) event-jump design its speed:
//   - AUDCTL's clock-source bits (CH1_179/CH3_179/CH1_CH2/CH3_CH4/CLOCK_15) are
//     re-read only when a channel (or channel pair) naturally rearms, not
//     applied the instant they change mid-countdown. Real POKEY applies a
//     clock-source change on the very next 1.79MHz tick; catching that would
//     mean falling back to per-clock stepping for the transition. AUDF/AUDC are
//     latched-at-reload on real hardware too (ProSystem's own comment: "current
//     divCounter continues as normal even though we've changed the frequency"),
//     so those were never a gap.
//   - A borrow already "in flight" (the +4/+7 propagation delay past a wrap) is
//     not separately tracked, so it does not survive an SKCTL write that holds
//     the chip in reset - it is simply cancelled, same as every other pending
//     borrow. Real POKEY (and MAME's step_one_clock()) lets an in-flight borrow
//     complete even while frozen. This is a one-1.79MHz-clock-wide corner that
//     tools/pokey_selftest/ deliberately does not fuzz across (see its README).
//   - RANDOM ($X00A) is still the free-running LFSR the first version used, not
//     a read of the same poly17 table the synthesis engine uses. Investigating
//     this found MAME's RANDOM read is a MULTI-BIT slice of the raw 17-bit
//     shift register word at the sampled instant (pokey.cpp read(), RANDOM_C:
//     "(m_poly17[m_p17] >> 8) & 0xff"), not the single output bit the engine
//     needs - matching it exactly would mean a ~393KB table (17 bits x 131071
//     entries) instead of the 16KB one below. No cartridge in the library uses
//     POKEY's RANDOM for anything but "does this value change" (chip presence
//     detection), which the existing LFSR already satisfies.
//
//   - Two-tone mode (SKCTL bit 3) resets CHAN1's own timing correctly, but if
//     CHAN1 is ALSO the low half of an active 16-bit join at that instant, the
//     high half's closed-form schedule (pk_period_pair_total(), computed once
//     when the pair was armed and assuming CHAN1 completes its free-run
//     periods on an undisturbed schedule) is not recomputed to account for the
//     interruption. tools/pokey_selftest/ found this by fuzzing SKCTL freely
//     on every pass, which enables two-tone incidentally even on passes not
//     named for it - two of its twenty configurations only pass with SK_TWOTONE
//     masked out of that fuzzing, both of them a joined pair; see its README.
//     Fixing this properly means tracking how many of the low half's free-run
//     periods have already counted toward the high half's threshold as
//     explicit state, which is exactly the per-clock bookkeeping the event-jump
//     design exists to avoid. No cartridge in the library uses two-tone mode
//     (a serial-port feature repurposed for audio) together with a 16-bit pair
//     on the same channels, so this is deliberately left as the one case this
//     rewrite does not chase.
//   - A residual, low-probability (~0.01% of clocks in adversarial fuzzing,
//     never seen outside it) mismatch remains when an SKCTL freeze/release
//     cycle lands on a channel that is the low half of an active join AND
//     using a non-PURE, non-NOTPOLY5 distortion mode at a very small AUDF -
//     tools/pokey_selftest/'s "CH1+CH2 joined, 64kHz" pass is the one
//     configuration of twenty that still shows it. Not tracked down further:
//     pk_freeze_snapshot() rescales the low half's remaining distance
//     correctly in isolation (verified separately), so the gap is specifically
//     in some interaction between that rescale and the join/gate machinery
//     that a rarer combination of conditions is needed to trigger.
//
// Everything else the original header listed as a simplification is fixed:
// high-pass filters (AUDCTL bits 2/1), the 9-bit poly (bit 7), the 5-bit poly
// gate now reads a globally free-running phase instead of one sampled per
// channel period, and two-tone mode (SKCTL bit 3, 45/434 library files) except
// for the join interaction just above.

#ifndef POKEY_H
#define POKEY_H

#include <stdint.h>
#ifndef POKEY_HOST_TEST
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#endif
#include "pokey_tables.h"

#define POKEY_AUDIO_PIN   29        // ExtAudio -> 7800 connector pin 18
#define POKEY_SAMPLE_RATE 32000
#define POKEY_PWM_WRAP    511       // 9-bit PWM; carrier = clk_sys / 512

// POKEY_CLOCK_KHZ - the core clock used while a POKEY cart plays. Lives here,
// next to the reason for it, exactly as YM_CLOCK_KHZ lives in ym2151.h.
//
// MEASURED ON HARDWARE, not assumed (2026-08-27, experiment E2 below). POKEY
// carts showed thin flickering lines that YM2151 carts on this same pin, these
// same emulate_*() loops and this same console did not - and the raised clock
// was one of only two things YM did differently. Giving POKEY carts the same
// 300MHz removed those lines completely on all three banked-mapper titles that
// showed them (Bentley Bear, Donkey Kong PK-XM, Commando). The attribution is
// clean: all three are cart types >=35, so they were ALREADY running at 1.30V
// before this change - the clock was the only variable that moved for them.
//
// This is NOT the "raise the clock and hope" that 2026-08-24 got wrong (see
// EMU_CLOCK_KHZ's comment in the .ino): it is the same 300MHz YM has been
// shipping on this board since 0.29, and the artifact it removes was observed
// present at 250MHz (both the 0.32 build and the E3 build) and absent at
// 300MHz, on the same three files.
#define POKEY_CLOCK_KHZ   300000

// --- DIAGNOSTIC BUILD SWITCHES ----------------------------------------------
// All default to 0, so an ordinary build never contains any of them. Each one
// removes exactly ONE variable from the picture-artifact investigation, so a
// symptom that survives it cannot be caused by the thing it removed.
//
// WHAT THE FIRST TWO ALREADY SETTLED (hardware, 2026-08-27):
//
//   E2  POKEY carts get YM's 300MHz/1.30V. RESULT: the thin flickering LINES
//       vanished; the left-edge BAND did not. ADOPTED AS PRODUCTION above, so
//       there is no POKEY_DIAG_E2 switch - it would be a no-op. To re-run that
//       comparison against a future baseline, put POKEY_CLOCK_KHZ back to
//       250000 and rebuild; that is the whole of what E2 was.
//   E3  Synthesis runs at full rate but the audio pin is never driven, leaving
//       it the plain input a non-POKEY cart leaves it. RESULT: every artifact
//       class survived - lines (Bentley, still at 250MHz in this build), band
//       (LZSS), flicker (R-Type). THE AUDIO SIGNAL IS EXONERATED: the artifacts
//       are not the raw PWM on pin 18, so no RC filter and no carrier change is
//       called for. (R-Type still made sound here. Not a leak from this path -
//       it writes 16 TIA audio registers, i.e. the console's OWN sound chip,
//       which nothing in a cartridge can switch off.)
//
// WHAT IS STILL OPEN, AND WHAT E4/E5 ARE FOR:
//
//   The left-edge band survives BOTH the clock change and a dead audio pin, so
//   it is neither. Two candidates remain, and E2/E3 cannot separate them
//   because both of those builds leave BOTH candidates running:
//     (a) CORE 0 - a POKEY cart is the only case where core 0 synthesises
//         continuously, contending with core 1's rom_table reads for SRAM and
//         pulling 16KB of poly tables through the XIP cache. This is the
//         unknown POKEY_IMPROVEMENT.md 9 and YM2151.md 9 both flagged and
//         neither measured.
//     (b) THE BUS-SIDE WINDOW - the POKEY address decode and write capture
//         inside the emulate_*_pokey() loops.
//
//   E4  Core 0 never enters pokey_run(). Bus side untouched.
//   E5  The mirror image: core 0 keeps synthesising, but core 1 runs the plain
//       emulate_*() loop, so the bus side is byte-identical to a cart with no
//       POKEY.
//
//   RESULT (hardware, 2026-08-27): THE BAND SURVIVED BOTH. The pair was meant
//   to say which of (a)/(b) it was; instead it ruled out the whole question.
//   pokey_enabled gates exactly three things in this firmware - the clock
//   branch, POKEY_BUS_ON, and the pokey_run() call - and the band outlived all
//   three (the clock at both 250 and 300MHz, the window under E5, the
//   synthesis under E4). There is no fourth. So NO POKEY-SPECIFIC BEHAVIOUR
//   EXPLAINS IT.
//
//   The sharpest form of that is E5: there, Bloodfighter is served by
//   emulate_normala78() - the same machine code, byte for byte, that serves
//   the non-POKEY carts whose picture is clean. Under identical firmware code
//   one file bands and another does not, so the difference is IN THE ROM, not
//   in this code. The open question is therefore no longer "which POKEY
//   behaviour causes the band" but "is the band about POKEY at all", which
//   needs no new switch: TEST_ROMS_PASEK/2_KONTROLA runs matched non-POKEY
//   titles on the PRODUCTION build. See that README.
//
//   Both switches are SILENT by design (E4 computes nothing, E5 never sees a
//   register write), so they judge picture only. Kept, though spent, so the
//   pair can be re-run against a future baseline.
//
// E6 - WHAT THE CYCLE BUDGET POINTED AT
//
//   With POKEY excluded and the demos themselves exonerated, the band was
//   measured off the console photographs: the first ~47 of 320 pixels on every
//   line carry the PREVIOUS line's content, the step is exactly one scanline,
//   and it sits at the same place at 250MHz and at 300MHz
//   (TEST_ROMS_PASEK/pomiar_zalamania/).
//
//   The cycle budget then ruled out the obvious reading of that
//   (TEST_ROMS_PASEK/budzet_cyklowy/): against MARIA's tightest 282ns fetch
//   interval we answer in 97-176ns - 1.6-2.3x of margin, and FASTER than the
//   150-250ns mask ROMs this console was designed around. We are not too slow,
//   which is independently why the 250->300MHz move did not shift the band by
//   a pixel, and why shortening this path further would buy nothing.
//
//   What the same disassembly did show is that emulate_normala78() clears OE
//   after EVERY fetch and then needs two matching address reads before it can
//   drive again - roughly 37 core cycles with the data lines undriven, on every
//   fetch of a DMA burst. The SuperGame loops do not do this: rom_in_use holds
//   OE across consecutive reads.
//
//   E6 rebuilds the flat loop in that same, already-hardware-proven shape:
//   non-blocking, one bus sample per pass, byte put out by address, and R/W
//   used only to decide whether the drivers are enabled. It changes WHEN we
//   drive, never WHICH byte we serve.
//
//   The 0.18 regression this deliberately avoids: the note on the POKEY branch
//   in the .ino warns that "blocking ROM path + R/W gating" is what broke
//   3D Worldrunner. E6 gates on R/W but does NOT block anywhere - the same
//   combination emulate_supercart_ram() has shipped since 0.15.
//
//   RESULT (hardware, 2026-08-28): FAILED, AND MADE THINGS WORSE. The band
//   stayed on all three flat titles, and the SOUND broke on every one of them
//   (Ballblazer, clean and audible before, came back distorted; White Lamp went
//   silent). Do not ship this shape.
//
//   The cause was a design error of mine, not a property of the console:
//   holding OE meant the loop could no longer block, and dropping the blocking
//   wait ALSO dropped the wait for a stable address. The POKEY window test
//   ((addr & pkmask) == pkbase) then ran on a RAW sample, so an address caught
//   mid-transition could land in the window and inject a garbage byte into
//   pokey_regs[]. White Lamp suffered most because its $0800 window is 2KB
//   wide rather than 16 bytes - by far the biggest target. E6's picture result
//   is therefore weakened too: it removed one candidate cause and introduced
//   another, since ROM was also being served off an unstable address.
//
// E7 - THE ONE THING E6 ACCIDENTALLY PROVED
//
//   E6's failure is itself evidence: removing the two-matching-samples filter
//   was enough to corrupt the register file, which means the address lines
//   really do glitch and our sampling of them really does matter. The shipping
//   loop has that filter, but it is weak - two samples, about 27ns apart at
//   300MHz. A glitch that survives two samples lands not only in the POKEY
//   window but in the rom_table INDEX, i.e. in the picture.
//
//   E7 therefore turns exactly ONE dial and nothing else: THREE matching
//   samples instead of two. The blocking wait stays, the OE release after each
//   fetch stays, the POKEY branch stays - the diff is one extra confirming
//   read. Budget allows it: the third sample costs ~8 core cycles (~27ns at
//   300MHz) against 1.6-2.3x of headroom we already measured.
//
//   Band gone -> address glitching was the cause, and the fix is to carry a
//   stronger filter into production (and into the SuperGame loops, which have
//   NO stability check at all - they sample once per pass).
//   Band unchanged -> the cartridge side is exhausted and the next question is
//   about MARIA's own bus behaviour, which we do not model.
//   Sound must stay clean on Ballblazer and Camouflage: unlike E6, E7 does not
//   touch how the POKEY window is reached, so a repeat of E6's distortion would
//   mean the extra sample itself is harmful.
//   LZSS and R-Type are SuperGame carts, so neither E6 nor E7 touches them:
//   they are the built-in negative control and MUST look exactly as today.
//
// Build with ./build.sh PicoA10400-E3 (or -E4 / -E5).
#ifndef POKEY_DIAG_E3
#define POKEY_DIAG_E3 0
#endif
#ifndef POKEY_DIAG_E4
#define POKEY_DIAG_E4 0
#endif
#ifndef POKEY_DIAG_E5
#define POKEY_DIAG_E5 0
#endif
#ifndef POKEY_DIAG_E6
#define POKEY_DIAG_E6 0
#endif
#ifndef POKEY_DIAG_E7
#define POKEY_DIAG_E7 0
#endif
#ifndef POKEY_DIAG_E8
#define POKEY_DIAG_E8 0
#endif

// E9 / E10 - the 0.47 dual-POKEY follow-up. Hardware reported NOISE ON TOP OF
// THE MUSIC on exactly two of the nine dual carts, "LZSS Player" and "Portal",
// and those two are not an arbitrary pair: they are the ONLY two whose header
// byte54 is 0x46 rather than 0x40, i.e. the only two that declare SuperGame
// plus RAM at $4000. That routes them to emulate_supercart_ram_pokey() while
// the other seven run emulate_normala78_pokey(), and those two loops capture a
// POKEY write in genuinely different ways:
//
//   flat loop            no R/W gating, but a bounded 64-turn end-of-cycle scan
//                        that keeps the LAST byte seen while the address was
//                        still valid - i.e. after the 6502 has driven the data.
//   SuperGame/RAM loop   pokey_window_service(): R/W gated, drives the bus on a
//                        read, and takes the data byte from ONE early sample,
//                        relying on the loop re-entering to converge.
//
// The .ino documents this exact hazard twice on that very loop ("Krok 19", the
// bank register and the on-cart RAM window both got an end-of-cycle scan for
// it: "the byte is sometimes good and sometimes garbage"). The POKEY window on
// that loop never did. Before 0.47 that only fed the chip at $0450; now it also
// feeds $0440, so there is twice as much of it and it is audible on both.
//
// These two switches separate the two candidate explanations. They are
// diagnostics, not fixes: pokey.h's own note above records that a bounded wait
// dropped into these loops once cost MARIA its data (XMAS/Arkanoid glitches),
// which is precisely why E9 has to be MEASURED on a console rather than
// adopted.
//
//   E9   pokey_window_service() captures with the same bounded end-of-cycle
//        scan the flat loop uses. Noise gone -> the capture path is the cause.
//        Watch the PICTURE as carefully as the sound on this one.
//   E10  chip 1 ($0450) only; chip 0 ($0440) is never rendered. The bus side is
//        untouched, so this is 0.46's AUDIO with 0.47's everything else. Noise
//        gone -> what chip 0 receives is the problem, not the synthesis.
#ifndef POKEY_DIAG_E9
#define POKEY_DIAG_E9 0
#endif
#ifndef POKEY_DIAG_E10
#define POKEY_DIAG_E10 0
#endif

// E11 / E12 - the second round, after E9/E10 came back from hardware:
//
//   0.47   LZSS: noise           Portal: buzz
//   E10    LZSS: NO noise, but   Portal: buzz STILL THERE
//          the sound is distorted
//   E9     LZSS: noise           Portal: buzz, lighter
//
// Two separate faults, not one:
//
//   LZSS   the noise IS chip 0's rendered output (E10 removes it) and it is NOT
//          a capture-timing artefact (E9 does not). So chip 0 is being fed bad
//          DATA. And the data has a source: LZSS reads its POKEY register
//          stream straight out of ON-CART RAM - eighteen stores of the shape
//          "LDA $6000,X / STA $0440+n" and "LDA $7000,X / STA $0450+n", nine
//          registers per chip. $6000 and $7000 are inside $4000-$7FFF, the
//          SuperGame RAM window this firmware serves from ram_table. No other
//          dual demo does this: measured over all ten, LZSS is the only one
//          with a single such store. A byte lost or mis-addressed in that RAM
//          therefore lands DIRECTLY in an AUDF/AUDC register - which is what
//          noise sounds like. It also explains E10's "distorted": chip 1's
//          table lives in the same RAM, so muting chip 0 removes the noise but
//          leaves chip 1 playing from the same damaged data.
//
//   Portal the buzz survives chip 0 being muted, so it is NOT the second chip
//          and NOT a 0.47 regression - E10 is 0.46's audio. Portal does not
//          read POKEY data from cart RAM at all (0 such stores); it drives
//          chip 1 mostly through IRQEN/STIMER/SKCTL, i.e. it uses POKEY as an
//          interrupt timer, and takes chip 0's data from CONSOLE RAM ($23A0).
//          Separate problem, tracked separately.
//
//   E11  chip 0 ($0440) only, chip 1 muted - the mirror of E10. If LZSS's
//        chip 0 alone is ALSO distorted music rather than pure noise, then both
//        chips are equally damaged and the fault is upstream of POKEY entirely,
//        i.e. the RAM. If chip 0 alone is pure noise while chip 1 alone was
//        music, the damage is specific to the $6000 table.
//   E12  the SuperGame+RAM loop confirms the address with a SECOND matching
//        sample before it decodes anything below $8000. That loop takes the
//        address from ONE sample, unlike emulate_normala78_pokey(), which has
//        required two matching samples since 0.16 - and a mis-sampled address
//        serves the WRONG ram_table byte. For a graphics cart that is one
//        wrong pixel; for LZSS it is a wrong POKEY register. Non-blocking: a
//        mismatch just re-enters the loop, it never waits.
#ifndef POKEY_DIAG_E11
#define POKEY_DIAG_E11 0
#endif
#ifndef POKEY_DIAG_E12
#define POKEY_DIAG_E12 0
#endif

// E13 - "listen-only" for the aux window on the SuperGame/RAM loops. TRIED AND
// REJECTED; the switch is kept only so the negative result can be reproduced.
//
// The full sequence, and how it landed (hardware, 2026-09-06):
//
//   build  rendered     bus change                    LZSS            Portal
//   0.47   chip0+chip1  -                             noise           buzz
//   E9     chip0+chip1  end-of-cycle capture          noise           buzz, lighter
//   E10    chip1 only   -                             no noise*       buzz
//   E11    chip0 only   -                             noise, music ok NO BUZZ
//   E12    chip0+chip1  RAM address = 2 samples       noise           buzz
//   E13    chip0+chip1  listen-only + end-of-cycle    noise           NO BUZZ, clicks
//   E14    chip0+chip1  CONFIRM THE WINDOW ADDRESS    CLEAN           CLEAN
//   E15    chip0+chip1  E13 + E14                     noise           clean
//
//   (*) E10's "distorted" was half an arrangement, not a defect - E11 showed
//       chip 0 alone plays it clean.
//
// E14 IS NOW PRODUCTION (see the guard in pokey_window_service() above), so
// there is no POKEY_DIAG_E14 switch - it would be a no-op, exactly as E2's
// clock change has had none since 0.33. POKEY_DIAG_E15 is gone with it: with
// the guard shipped, E13 alone now *is* what E15 was.
//
// AND THAT IS THE POINT OF KEEPING E13. E15 - listen-only WITH the address
// guard - came back WORSE than the guard alone: LZSS was noisy again. So the
// listen-only idea is not merely unnecessary, it is harmful on these loops, and
// the switch is here so that can be re-checked rather than re-argued. Note that
// E13 changes TWO things at once (it drops the R/W gate AND stops driving), so
// its result is not attributable to either one on its own; since E14 alone
// fixes both files there was no reason to split it further.
#ifndef POKEY_DIAG_E13
#define POKEY_DIAG_E13 0
#endif

// --- POKEY_DIGI_QUEUE: sub-sample timing for sample ("digi") playback -------
// POKEY_IMPROVEMENT.md 5.4 predicted this and deliberately deferred it: "digi
// streams through VOLUME_ONLY are limited by our sample rate - writes faster
// than 32kHz are lost. A timestamped queue, like YM2151's, would be the cure,
// but YM needed one for a different reason ($08 = KEY ON/OFF is an event).
// POKEY is state. Note it and do not build it until some title forces it."
//
// A title has now forced it. "R-Type - Deep Mix" is a sample-mixing demo and
// its sound comes back distorted/incomplete on hardware; Bloodfighter (covox)
// and LZSS Player stream samples the same way. The defect is not only lost
// writes: even when a write survives, the flat register file makes the engine
// see it at the START of the next 32kHz sample instead of when it happened, so
// every transition is quantised by up to 31us. That is audible as distortion.
//
// WHAT THIS CHANGES. Registers 0-7 (AUDF1..AUDC4 - the ones a sample player
// hammers) stop going into pokey_regs[] from core 1 and travel as timestamped
// events instead. Core 0 replays them at their exact instant inside the sample
// window, which the event-driven engine already handles natively: it advances
// to an arbitrary tick with pk_run_to() and its output is the time-weighted
// average over the window, so a transition placed correctly inside the window
// is integrated correctly rather than aliased.
//
// WHAT IT DELIBERATELY DOES NOT CHANGE. AUDCTL (8), STIMER (9) and SKCTL ($0F)
// keep the existing once-per-sample snapshot path. They are control registers
// written rarely, the resync/freeze machinery hangs off them, and giving them
// sub-sample placement would mean per-event freeze bookkeeping for no audible
// gain. If the queue is FULL the write falls back to the old direct path, so a
// burst can lose timing precision but never loses the write itself.
//
// NOT COVERED BY tools/pokey_selftest/: that test drives the engine directly
// and POKEY_DIGI_QUEUE is forced off under POKEY_HOST_TEST, so the queue's
// timing logic is verified by listening, not by the differential test. The
// engine itself is untouched.
#ifndef POKEY_DIGI_QUEUE
#define POKEY_DIGI_QUEUE 0
#endif
#ifdef POKEY_HOST_TEST
#undef POKEY_DIGI_QUEUE
#define POKEY_DIGI_QUEUE 0
#endif
// --- BUS_DRIVE_STRENGTH: how hard the eight data lines are driven ----------
// MEASURED ON HARDWARE, not reasoned about. This is the third time in this
// project that a value had to be measured rather than argued (see the PAL
// palette and POKEY_CLOCK_KHZ) - and the reasoned guess was wrong again.
//
// WHAT THIS FIXES - AND WHAT IT DOES NOT. The firmware had never configured
// pad electrics at all: every pin sat on the RP2040 default of 4mA since the
// project started. Four builds were measured, one instruction apart:
//    2mA   sound distorted in 3 titles, LZSS Player would not boot
//    4mA   LZSS Player would not boot (this is what production shipped)
//    8mA   every title runs correctly, LZSS Player boots
//   12mA   same as 8mA, no further improvement
// So the bus is CAPACITIVE (edge connector, cable, MARIA inputs) and 4mA was
// marginal: enough for most titles, not enough for the one that streams data
// hardest. The first hypothesis - ringing on an unterminated line, cured by
// WEAKER edges - was backwards, which 2mA settled immediately.
//
// IT DOES NOT FIX THE SMEAR BAND. The vertical band down the left edge of the
// picture (~47 of 320 pixels, showing the previous scanline's content) is
// UNCHANGED at 2, 4, 8 and 12mA. Drive strength is now on the same list as
// clock speed in both directions, the audio pin, core 0 synthesis, the bus-side
// POKEY window, holding OE between fetches and address sample strength (E2-E7):
// measured, and ruled out. Nothing left in the firmware plausibly explains it.
//
// WHY 8 AND NOT 12. 8mA is the lowest level at which everything works, and
// there is no reason to drive harder than the load needs: the board has no
// series resistors, emulate_normala78() does not test R/W so it drives during
// write cycles too, and 12mA would roughly triple both that contention current
// and the ground bounce from eight lines switching at once. Nothing about 12mA
// looked bad on hardware - it is simply unnecessary.
//
// SCOPE. Data lines only (D0..D7). The address lines are inputs, where drive
// strength does nothing, and the audio pin was not part of the measurement, so
// it is deliberately left alone.
#define BUS_DRIVE_STRENGTH GPIO_DRIVE_STRENGTH_8MA

// E8 diagnostic: override the production level to re-run the measurement.
// Switched on by its VALUE, not by a separate level flag: 0 = production,
// otherwise the drive strength in mA (2, 4, 8 or 12). ONE macro rather than
// two because arduino-cli's library discovery breaks when a single
// --build-property carries two -D options separated by a space - the sketch
// then fails to find SdFat.h, which is a confusing way to learn that.
// Building with POKEY_DIAG_E8=8 must reproduce the production binary exactly;
// POKEY_DIAG_E8=4 reproduces the pre-0.34 behaviour for an A/B comparison.
#if   POKEY_DIAG_E8 == 0
/* the production value above stands */
#elif POKEY_DIAG_E8 == 2
#undef  BUS_DRIVE_STRENGTH
#define BUS_DRIVE_STRENGTH GPIO_DRIVE_STRENGTH_2MA
#elif POKEY_DIAG_E8 == 4
#undef  BUS_DRIVE_STRENGTH
#define BUS_DRIVE_STRENGTH GPIO_DRIVE_STRENGTH_4MA
#elif POKEY_DIAG_E8 == 8
#undef  BUS_DRIVE_STRENGTH
#define BUS_DRIVE_STRENGTH GPIO_DRIVE_STRENGTH_8MA
#elif POKEY_DIAG_E8 == 12
#undef  BUS_DRIVE_STRENGTH
#define BUS_DRIVE_STRENGTH GPIO_DRIVE_STRENGTH_12MA
#else
#error "POKEY_DIAG_E8 must be 0 (production) or a drive strength: 2, 4, 8 or 12"
#endif

// --- bus-side state, unchanged from the first version -----------------------
// Register file, written by core 1 from the bus, read by core 0. Plain volatile
// bytes are enough: these are state, not events, so a late or reordered update is
// at worst one sample stale. No queue, no locking, nothing core 1 has to wait for -
// its hot loop must not block. The ONE exception is STIMER (offset 9, see
// pokey_stimer_seq below): it is a strobe, and "last write wins" cannot see a
// second strobe of the SAME value.
volatile uint8_t pokey_regs[16] = {0};
// SECOND CHIP. Nine files in the library declare two POKEYs at once (bit 10 =
// $0440 together with bit 6 = $0450 in the a78 header's 16-bit mapper field);
// one more, "White Lamp Music Demo (Dual POKEY 800 810)", uses $0800 and $0810
// without any header bit able to say so. Both pairs are SIXTEEN BYTES APART, so
// the two windows are one 32-byte window with address bit 4 choosing the chip -
// which is why the hot loops need no second compare (see pokey_reg_mask).
volatile uint8_t pokey2_regs[16] = {0};
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

// Which address bits carry the register index INSIDE that window. 0x0F for a
// single chip - exactly the "addr & 0x0F" every capture site used to spell out
// literally. 0x1F when two chips share one 32-byte window: bit 4 then selects
// the chip and bits 3-0 the register, so pokey_capture_write() can split them
// without the bus loop testing a second base address. Nothing else changes -
// a single-POKEY cart gets 0x0F here and behaves exactly as before.
volatile uint8_t pokey_reg_mask = 0x0F;

// Has the bus ever written this chip? Set by core 1 on every capture, read by
// core 0 once per output sample. Two jobs, both of which need "written", not
// "declared": chip 1's synthesis is skipped entirely until it exists (so the
// ~424 single-POKEY files in the library pay literally nothing for this
// feature), and the output scale divides by 2 only when BOTH chips are really
// playing - so "POKEY Tester (810)", a lone chip sitting in the second slot,
// stays as loud as "POKEY Tester (800)" rather than half as loud.
volatile uint8_t pokey_live[2]  = {0, 0};

// STIMER (offset 9) reloads all four channel counters and forces their outputs
// to a known state - it is an EVENT, not a value, and two STIMER writes in a
// row (same or different data, the byte written is ignored by real POKEY too)
// must both be seen. A plain register-file capture would only notice the
// SECOND write if its value differed from the first. This counter increments
// on every write to offset 9 regardless of value; core 0 reacts to a CHANGE in
// it, which a "last write wins" byte cannot represent for a repeated strobe.
volatile uint32_t pokey_stimer_seq[2] = {0, 0};

#if POKEY_DIGI_QUEUE
#include "hardware/timer.h"
#define POKEY_Q_SIZE 128                      /* power of two */
typedef struct { uint32_t t_us; uint8_t reg; uint8_t val; } pk_ev_t;
// One queue PER CHIP. A shared queue cannot work once there are two renderers:
// each drains until it reaches an event later than its own window, so whichever
// ran first would consume the other's events. Costs nothing while the feature
// is off, which is its default.
static volatile pk_ev_t pokey_queue[2][POKEY_Q_SIZE];
static volatile uint32_t pk_q_head[2] = {0, 0};   // written by core 1 only
static volatile uint32_t pk_q_tail[2] = {0, 0};   // written by core 0 only
#endif

// Capture one POKEY register write off the bus. Single point so the queue and
// the plain register file cannot drift apart. always_inline for the same reason
// everything else on this path is: see the note above pokey_read_reg().
static inline __attribute__((always_inline))
void pokey_capture_write(uint32_t reg, uint8_t val) {
  // 'reg' arrives masked with pokey_reg_mask, so bit 4 is the chip select for a
  // dual cart and is always 0 for every other one - no branch, no second window
  // test, and the single-chip path is the same three instructions it was.
  const uint32_t chip = (reg >> 4) & 1u;
  reg &= 0x0Fu;
#if POKEY_DIGI_QUEUE
  if (reg < 8) {                              // AUDF/AUDC - the digi registers
    uint32_t h = pk_q_head[chip], n = h + 1u;
    if ((uint32_t)(n - pk_q_tail[chip]) <= POKEY_Q_SIZE) {
      uint32_t i = h & (POKEY_Q_SIZE - 1u);
      pokey_queue[chip][i].t_us = timer_hw->timerawl;
      pokey_queue[chip][i].reg  = (uint8_t)reg;
      pokey_queue[chip][i].val  = val;
      pk_q_head[chip] = n;
      pokey_live[chip] = 1;
      return;                                 // timing channel ONLY - core 0
    }                                         // applies it into pokey_regs[]
    // queue full: fall through and take the old path rather than lose the write
  }
#endif
  // A lookup table of the two register files would be the obvious way to select
  // one, and it is the wrong one HERE: a static array of pointers lands in
  // .rodata, i.e. FLASH, and this line runs on core 1 inside a
  // __time_critical_func() loop. Reaching flash from a bus loop is the class of
  // defect patches 0.09/0.13 removed. A ternary between two .bss arrays keeps
  // both addresses in the RAM-resident literal pool of the loop itself -
  // checked in the .elf, there is no 0x10xxxxxx load on this path.
  volatile uint8_t *rf = chip ? pokey2_regs : pokey_regs;
  rf[reg] = val;
  if (reg == 0x09) pokey_stimer_seq[chip] = pokey_stimer_seq[chip] + 1;   // strobe
  pokey_live[chip] = 1;
}

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
#define POKEY_CH1_FILTER  0x04
#define POKEY_CH2_FILTER  0x02
#define POKEY_CLOCK_15    0x01
// --- SKCTL bits (only the two this file needs) ---
#define POKEY_SK_TWOTONE  0x08
#define POKEY_SK_RESET    0x03     // either bit set = chip released ("running")

#define POKEY_CLK_BASE    1787520u          // POKEY clock in the 7800 (Ron Fries'
                                             // FREQ_17_APPROX; see pokey.txt and
                                             // POKEY_IMPROVEMENT.md 6 for why this
                                             // stays: it makes the 64/15kHz divisors
                                             // exact integers, and the resulting
                                             // -2.2/+13.7 cent offset from real
                                             // NTSC/PAL is below what a 6502
                                             // sequencer's own rounding contributes)
#define POKEY_DIV_64      28u               // -> ~63.8 kHz
#define POKEY_DIV_15      114u              // -> ~15.7 kHz

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

static uint32_t pk_rand[2] = {0x1234, 0x1234};   // RANDOM generator, core 1 only -
                                        // see the "KNOWN SIMPLIFICATIONS" note
                                        // above for why this stays independent
                                        // of the synthesis engine's poly17
                                        // table. One per chip: two real POKEYs
                                        // are two independent shift registers,
                                        // and a game probing the second one has
                                        // to see it change on its own.

// 'reg' is masked with pokey_reg_mask, so bit 4 is the chip - same convention as
// pokey_capture_write(), and 0 for every single-POKEY cart.
static inline __attribute__((always_inline)) uint8_t pokey_read_reg(uint32_t reg) {
  if ((reg & 0x0Fu) == 0x0A) {         // RANDOM - the register games probe
    uint32_t chip = (reg >> 4) & 1u;
    uint32_t r = pk_rand[chip];
    r = (r >> 1) | ((((r) ^ (r >> 5)) & 1u) << 16);
    pk_rand[chip] = r;
    return (uint8_t)(r & 0xFF);
  }
  return 0xFF;                         // ALLPOT/IRQST/SKSTAT: idle reads as high
}

// Service one bus cycle that landed inside the POKEY register window.
// Non-blocking by design: the SuperGame loops never wait anywhere, and dropping a
// bounded wait into them cost MARIA its data (visible glitches on XMAS/Arkanoid).
// No wait is needed - the loop re-enters this branch repeatedly while the address
// sits in the window, so on a write the LAST store before the address changes is
// the one that survives, which is end-of-cycle capture for free.
// GPIO-dependent, so out of reach for tools/pokey_selftest/ (POKEY_HOST_TEST) -
// the DSP engine below never calls this, only the firmware's bus loops do.
#ifndef POKEY_HOST_TEST
static inline __attribute__((always_inline))
void pokey_window_service(uint32_t addr, uint8_t *rom_in_use) {
  // A YM2151 cart reuses this whole mechanism: identify_cartridge() points
  // pokey_base/pokey_mask at $0460/$0461 and sets pokey_enabled, so every
  // emulate_*_pokey() loop already routes the window here without a single
  // change at the call sites. Only one aux chip is emulated, so this is a
  // choice, not a merge - see ym2151.h.
  if (ym_enabled) { ym_window_service(addr, rom_in_use); return; }
  // CONFIRM THE ADDRESS BEFORE IT PICKS A REGISTER (0.48, measured - experiment
  // E14 below). The address handed in here is sampled at the TOP of the calling
  // loop and can be several instructions old; on emulate_supercart_ram_pokey()
  // it is the oldest of the three samples that pass takes. A sample caught while
  // the address lines are still settling - as the bus moves from $0450 to $0441,
  // say - can read as some OTHER address inside the window, and the captured
  // byte then lands in the wrong register of the wrong chip. On "LZSS Player"
  // that was audible as noise on top of otherwise correct music, and it is why
  // emulate_normala78_pokey() has stabilised its address with two matching
  // samples since 0.16 - the seven dual carts that run THAT loop were all clean.
  //
  // Giving up the cycle costs nothing: a real POKEY write holds its address for
  // the whole cycle, so the very next pass catches it. What is dropped is only
  // the settling artefact.
  //
  // Deliberately AFTER the ym_enabled test, so a YM2151 cart is untouched: 45
  // files have been through hardware on that path and nothing has been reported
  // against it. Same hazard in principle, no evidence and no reason to move it.
  if ((gpio_get_all() & BUS_PIN_MASK) != (addr & BUS_PIN_MASK)) {
    if (*rom_in_use) { SET_DATA_MODE_IN; *rom_in_use = 0; }
    return;
  }
  uint32_t g = gpio_get_all();
  // pokey_reg_mask is 0x0F for one chip and 0x1F for a dual cart, where bit 4
  // then names the chip - see its declaration. Read once here rather than in
  // the caller: this branch only runs on a window hit, never per bus cycle.
  const uint32_t rmask = (uint32_t)pokey_reg_mask;
#if POKEY_DIAG_E13
  // E13 DIAGNOSTIC: listen-only, exactly as emulate_normala78_pokey() does it.
  // Never drive, do not consult R/W, and take the LAST byte seen while the
  // address is still valid. This is the seven clean carts' handling, applied to
  // the two that are not.
  if (*rom_in_use) { SET_DATA_MODE_IN; *rom_in_use = 0; }
  {
    uint32_t last = g, cur;
    for (uint32_t n = 0; n < 64; n++) {
      cur = gpio_get_all();
      if ((cur & BUS_PIN_MASK) != (addr & BUS_PIN_MASK)) break;
      last = cur;
    }
    pokey_capture_write(addr & rmask, (uint8_t)((last >> D0_PIN) & 0xFF));
  }
  return;
#endif
  if (g & RW_PIN_MASK) {                                   // read cycle
    sio_hw->gpio_out = (uint32_t)pokey_read_reg(addr & rmask) << D0_PIN;
    if (!*rom_in_use) { SET_DATA_MODE_OUT; *rom_in_use = 1; }
  } else {                                                 // write cycle
    if (*rom_in_use) { SET_DATA_MODE_IN; *rom_in_use = 0; }
    uint32_t reg = addr & rmask;
#if POKEY_DIAG_E9
    // E9 DIAGNOSTIC: end-of-cycle capture, the same bounded 64-turn scan the
    // flat loop and this loop's own bank/RAM paths use. Keeps the LAST byte
    // seen while the address is still valid, i.e. after the 6502 has driven
    // it, instead of the single early sample production takes here.
    uint32_t last = g, cur;
    for (uint32_t n = 0; n < 64; n++) {
      cur = gpio_get_all();
      if ((cur & BUS_PIN_MASK) != (addr & BUS_PIN_MASK)) break;
      last = cur;
    }
    pokey_capture_write(reg, (uint8_t)((last >> D0_PIN) & 0xFF));
#else
    pokey_capture_write(reg, (uint8_t)((g >> D0_PIN) & 0xFF));
#endif
  }
}
#endif // POKEY_HOST_TEST

// =============================================================================
// SYNTHESIS ENGINE - per-borrow, table-driven. Builds standalone under
// POKEY_HOST_TEST (tools/pokey_selftest/) with none of the code above or the
// PWM/pacing code below.
// =============================================================================

typedef struct {
  uint32_t next;       // absolute POKEY-clock tick of this channel's own next borrow
  uint8_t  out;         // generator output bit, pre-filter
  uint8_t  filt;        // sample-and-hold comparand XORed with 'out' for the mix
  uint8_t  free_run;    // low half of an ACTIVE joined pair only: 0 = 'next' uses
                         // the AUDF-based first period, 1 = the fixed 256-pulse
                         // free-run period (see pk_arm_pair()/pk_fire_ch0|2())
} pk_chan_t;

// ONE EMULATED CHIP. Everything the engine used to keep in file-scope globals
// lives here instead, so the very same code drives either chip through a
// pointer: there is no second copy of the synthesis, only a second instance of
// its state. On Cortex-M0+ this is free - a struct member is a base register
// plus a small offset, which is exactly what a global already compiled to (a
// literal-pool base plus an offset).
typedef struct {
  volatile uint8_t *regs;   // this chip's register file, written by core 1
  uint8_t  idx;             // 0 or 1 - index into pokey_stimer_seq[]/pokey_live[]
  pk_chan_t ch[4];
  uint32_t now;             // POKEY's own clock; PAUSES while SK_RESET holds
  uint32_t epoch;           // 'now' value where poly phase 0 falls (reset pins
                            // this to 'now', so phase reads 0 the whole time
                            // the chip is held in reset)
  uint32_t mix;             // duration-weighted level accumulator
  uint8_t  level;           // current instantaneous summed level (0..60)
  // What used to be function-local statics inside pokey_next_sample(). A second
  // chip needs its own "have I been primed", "which AUDCTL did I last see" and
  // "which STIMER strobe have I already acted on"; leaving them shared would
  // make one chip's topology change silently resync the other.
  uint8_t  prev_audctl;
  uint8_t  primed;
  uint8_t  was_running;
  uint32_t seen_stimer_seq;
} pk_state_t;

// was_running starts 0 rather than the old global's 1, and that is not a
// behaviour change: the !primed branch in pk_render() assigns it from the
// current SKCTL before anything reads it, so the initial value was already dead.
static pk_state_t pk_st[2] = { { pokey_regs, 0 }, { pokey2_regs, 1 } };

// Mersenne fold: for mask == 2^shift - 1, 2^shift == 1 (mod mask), so summing
// the low 'shift' bits with everything above them is a modulo reduction. Loops
// at most 3 times for a full 32-bit input even at shift=4 (poly4's table), and
// the loop bound is why this is a function and not the unrolled two-step some
// of the earlier benchmarking used - a channel can be jumped forward by an
// arbitrary number of ticks in one call, not just a few dozen.
static inline uint32_t pk_fold(uint32_t x, uint32_t mask, uint32_t shift) {
  while (x > mask) x = (x & mask) + (x >> shift);
  return (x == mask) ? 0 : x;
}
static inline uint8_t pk_poly_bit(const uint8_t *tab, uint32_t idx) {
  return (tab[idx >> 3] >> (idx & 7)) & 1u;
}

// One channel's own borrow just happened at absolute tick 't'. Sample (or
// toggle) its generator exactly as pokey.cpp's process_channel() does - the
// 5-bit poly gates every non-pure mode, read at the CURRENT global phase, not
// one sampled per channel period (the bug POKEY_IMPROVEMENT.md 3.2 describes).
//
// The +1: pokey.cpp increments m_p4/5/9/17 at the TOP of step_one_clock(),
// before process_channel() ever runs, so the value a borrow on tick 't' reads
// has already counted t's own increment - i.e. it is the (t-epoch+1)-th
// value since the poly counters were last pinned to 0, not the (t-epoch)-th.
// Pure-tone testing never exercises this (PURE ignores the table, and
// NOTPOLY5 short-circuits the gate before it), which is how it went
// unnoticed until tools/pokey_selftest/ ran a mode that actually reads a
// table - see that tool's README for the trace that caught it.
static inline void pk_fire_generator(pk_state_t *pk, pk_chan_t *c, uint8_t audc, uint8_t audctl, uint32_t t) {
  uint32_t phase = t - pk->epoch + 1u;
  if ((audc & POKEY_NOTPOLY5) || pk_poly_bit(pk_poly5, pk_fold(phase, 31u, 5u))) {
    if (audc & POKEY_PURE) {
      c->out ^= 1;
    } else if (audc & POKEY_POLY4) {
      c->out = pk_poly_bit(pk_poly4, pk_fold(phase, 15u, 4u));
    } else if (audctl & POKEY_POLY9) {
      c->out = pk_poly_bit(pk_poly9, pk_fold(phase, 511u, 9u));
    } else {
      c->out = pk_poly_bit(pk_poly17, pk_fold(phase, 131071u, 17u));
    }
  }
}

// Recomputed from scratch after every event rather than patched incrementally:
// a channel's audible bit depends on ANOTHER channel's high-pass filter sample
// too (pk_fire_ch2()/pk_fire_ch3() below), so "only channel X changed" is not
// enough information. Four table-free lookups is not worth optimising against
// a budget this file uses under 3% of (POKEY_IMPROVEMENT.md 4.2).
static inline void pk_recompute_level(pk_state_t *pk) {
  uint32_t s = 0;
  for (int ch = 0; ch < 4; ch++) {
    uint8_t audc = pk->regs[ch * 2 + 1];
    uint8_t bit  = (pk->ch[ch].out ^ pk->ch[ch].filt) & 1u;
    if (bit || (audc & POKEY_VOLUME_ONLY)) s += (audc & POKEY_VOLUME_MASK);
  }
  pk->level = (uint8_t)s;
}

// The 28/114-tick base-clock divider is a single chip-wide free-running
// counter (pokey.cpp's m_clock_cnt[]) - it belongs to NO channel, so a channel
// (re)armed at an arbitrary tick does not get a clean (AUDF+1)*pulse period
// starting there: it has to wait out whatever is left of the CURRENT pulse
// first. Skipping this was a real bug tools/pokey_selftest/ caught - it only
// shows up right after an asynchronous rearm (STIMER, cold start, a topology
// resync, or a two-tone reset borrowed from another channel's schedule),
// because a NATURAL rearm (a channel firing on its own schedule) is already
// sitting exactly on a pulse boundary and (AUDF+1)*pulse lands on the next one
// automatically. pk->epoch doubles as this divider's phase reference: both it
// and the poly counters are pinned to 0 by the very same SKCTL-freeze branch
// in pokey.cpp (m_clock_cnt[0..2]=0 alongside m_p4/5/9/17=0), and neither is
// touched by STIMER, so one variable can serve both.
static inline uint32_t pk_ticks_to_pulse(pk_state_t *pk, uint32_t t, uint32_t pulse) {
  // A trigger clock is one where the divider REACHES 'pulse' after incrementing
  // - i.e. clock t itself triggers when (t - pk->epoch) % pulse == pulse-1, not
  // when the remainder is 0. Off by one here cost a full extra tick on every
  // asynchronous rearm; tools/pokey_selftest/ is what caught it (see its
  // README for the trace that pinned this down to the exact formula).
  uint32_t phase = (t - pk->epoch) % pulse;
  return (pulse - 1u) - phase;
}

// Period (in POKEY-clock ticks) from tick 't' to channel 'ch's own next
// borrow. Also the "first period" of the LOW half of a joined pair (pass
// pair=1) - the two differ only in the 1.79MHz propagation delay, +4 unlinked
// vs +7 joined (Altirra Hardware Reference Manual, cited verbatim in
// ORIG/test7800/hardware/pokey/channels.go). 'aligned' selects which case
// above applies: 1 for a natural, already-pulse-aligned rearm (the simple
// pulses_needed*pulse form), 0 for an asynchronous one (pk_ticks_to_pulse()
// covers the partial first pulse, then the rest are full-width).
//
// At 1.79MHz there IS no shared divider to be out of phase with - HICLK
// channels are driven directly off the master clock (clock_triggered[CLK_1]
// in pokey.cpp is unconditionally true every clock), not off m_clock_cnt[].
// 'aligned' still matters here, though, for a narrower reason: inc_chan() for
// a NATURAL rearm runs on the SAME tick that triggers the reload, but AFTER
// it - reset_channel() reloads the counter only once step_one_clock() reaches
// the check_borrow() section, well past that tick's own inc_chan() call - so
// the reloaded counter's first increment lands on the NEXT tick, and the full
// pulses_needed ticks after the firing tick is correct. An ASYNCHRONOUS reset
// (STIMER, cold start) has no such "already-consumed" tick: the very first
// tick processed after it IS the reload's first increment, one sooner than
// the natural case - hence pulses_needed-1. tools/pokey_selftest/ is what
// caught this (see its README): the pure-tone tick-by-tick trace that pinned
// it down was needed because PURE mode's XOR toggling can mask a one-tick
// FIRST-period error indefinitely once every later period is correct.
static inline uint32_t pk_period(pk_state_t *pk, int ch, uint8_t audf, uint8_t audctl, int pair, uint32_t t, int aligned) {
  int hiclk = (ch == 0 && (audctl & POKEY_CH1_179)) || (ch == 2 && (audctl & POKEY_CH3_179));
  uint32_t cycles = hiclk ? (uint32_t)(pair ? 7 : 4) : 1u;
  uint32_t pulses_needed = (uint32_t)audf + cycles;
  if (hiclk) return aligned ? pulses_needed : pulses_needed - 1u;
  uint32_t pulse = (audctl & POKEY_CLOCK_15) ? POKEY_DIV_15 : POKEY_DIV_64;
  if (aligned) return pulses_needed * pulse;
  return pk_ticks_to_pulse(pk, t, pulse) + (pulses_needed - 1u) * pulse;
}

// Whether channel 'ch's EFFECTIVE clock source is the 1.79MHz one with no
// shared divider - for the high half of a joined pair this follows the LOW
// half's own AUDCTL bit, not any bit of its own (channels 2/4 have none).
// Used only by pk_freeze_snapshot() below; pk_period()/pk_period_pair_total()
// never need it for ch 1/3 because they always reach a channel's timing
// through the pair's low half (L=0 or 2), never through the high half itself.
static inline int pk_is_hiclk(int ch, uint8_t audctl) {
  switch (ch) {
    case 0: return (audctl & POKEY_CH1_179) != 0;
    case 2: return (audctl & POKEY_CH3_179) != 0;
    case 1: return (audctl & POKEY_CH1_CH2) && (audctl & POKEY_CH1_179);
    default: return (audctl & POKEY_CH3_CH4) && (audctl & POKEY_CH3_179);
  }
}

// SKCTL write_internal()'s SK_RESET-assert branch does more than freeze the
// chip: it zeroes m_clock_cnt[0..2], the shared 28/114-tick divider, INSTANTLY
// discarding whatever fraction of the current pulse every slow-clock channel
// had already accumulated - while leaving each channel's OWN counter/borrow
// state untouched (reset_channel() is not called here). A channel that was,
// say, 7 ticks from its next borrow when the freeze hit is NOT 7 ticks from it
// after release: the divider it was waiting on no longer exists, and a fresh
// one starts from zero at the release instant. What DOES survive is the
// COUNT of borrows still owed - recovered here from the channel's current
// 'next' (still expressed in the OLD, about-to-be-invalid phase) and re-armed
// against the divider's fresh zero with the same pk_ticks_to_pulse() math an
// asynchronous rearm uses elsewhere in this file. Missing this was a second
// real bug tools/pokey_selftest/ caught, on top of the plain off-by-one in
// pk_ticks_to_pulse() itself - see that tool's README for the two traces.
//
// Called exactly once, on the EDGE where SK_RESET transitions from released to
// held (pokey_next_sample() tracks "was running" to detect it) - never while
// already frozen, and never on release, where nothing needs correcting.
static inline void pk_freeze_snapshot(pk_state_t *pk, uint32_t t, uint8_t audctl) {
  for (int ch = 0; ch < 4; ch++) {
    if (pk_is_hiclk(ch, audctl)) continue;         // no shared divider to lose phase on
    uint32_t pulse = (audctl & POKEY_CLOCK_15) ? POKEY_DIV_15 : POKEY_DIV_64;
    uint32_t remaining = pk->ch[ch].next - t;       // ticks left under the OLD phase
    uint32_t pulses_remaining = (remaining + pulse - 1u) / pulse;   // round up
    if (pulses_remaining == 0u) pulses_remaining = 1u;
    pk->ch[ch].next = t + pulses_remaining * pulse - 1u;
  }
}

// The LOW half of an active joined pair, after its first borrow, free-runs a
// full 256-tick 8-bit counter before borrowing again (it is not reloaded - see
// POKEY_IMPROVEMENT.md's "para 16-bit" note for the trace that established
// this against a from-scratch transcription of step_one_clock(): the naive
// assumption that every free-run wrap ALSO pays the propagation delay again is
// wrong, and pk_period_pair_total() below only matches MAME's actual output
// because this does not add 'pair's delay a second time here).
static inline uint32_t pk_period_freerun(int ch, uint8_t audctl) {
  int hiclk = (ch == 0 && (audctl & POKEY_CH1_179)) || (ch == 2 && (audctl & POKEY_CH3_179));
  if (hiclk) return 256u;
  uint32_t pulse = (audctl & POKEY_CLOCK_15) ? POKEY_DIV_15 : POKEY_DIV_64;
  return 256u * pulse;
}

// Absolute tick of the HIGH half's own borrow, computed directly rather than by
// stepping every intermediate low-half wrap: one first period plus AUDF_high
// free-run periods. Reduces to the textbook AUDF16+cycles only because the
// first period already carries the +cycles term and every free-run period
// carries none - see pk_period_freerun() above. 'aligned' passes straight
// through to the first period's pk_period() call, same meaning as there.
static inline uint32_t pk_period_pair_total(pk_state_t *pk, int L, uint8_t audf_lo, uint8_t audf_hi, uint8_t audctl, uint32_t t, int aligned) {
  return pk_period(pk, L, audf_lo, audctl, 1, t, aligned) + (uint32_t)audf_hi * pk_period_freerun(L, audctl);
}

// (Re)arm channel 'ch' as an independent, unlinked channel starting from tick
// 't'. Never touches out[]/filt[] - only STIMER and a filter-owning channel's
// own borrow do that. 'aligned': see pk_period() above.
static inline void pk_arm_single(pk_state_t *pk, int ch, uint32_t t, uint8_t audctl, int aligned) {
  pk->ch[ch].free_run = 0;
  pk->ch[ch].next = t + pk_period(pk, ch, pk->regs[ch * 2], audctl, 0, t, aligned);
}

// (Re)arm the pair whose low half is L (0 or 2, high half L+1) fresh from tick
// 't': the low half gets a first-period borrow, the high half's own borrow is
// scheduled directly via the closed form above. 'aligned': see pk_period().
static inline void pk_arm_pair(pk_state_t *pk, int L, uint32_t t, uint8_t audctl, int aligned) {
  int H = L + 1;
  pk->ch[L].free_run = 0;
  pk->ch[L].next = t + pk_period(pk, L, pk->regs[L * 2], audctl, 1, t, aligned);
  pk->ch[H].next = t + pk_period_pair_total(pk, L, pk->regs[L * 2], pk->regs[H * 2], audctl, t, aligned);
}

// Re-arm every channel/pair from tick 't' using the CURRENT AUDCTL. Always
// asynchronous (aligned=0 throughout - see pk_period()): called on the very
// first sample, on STIMER, and whenever AUDCTL changes, none of which have any
// relationship to the base-clock divider's current phase. See also the
// "KNOWN SIMPLIFICATIONS" note at the top of this file for what an AUDCTL-
// triggered resync deliberately does NOT try to preserve (exact phase across a
// live topology change) and why: no cartridge in the library changes
// CH1_CH2/CH3_CH4/HICLK mid-note, and preserving phase across it would need
// per-clock stepping for the transition, defeating the point of the
// event-jump design.
static inline void pk_resync_all(pk_state_t *pk, uint32_t t, uint8_t audctl) {
  if (audctl & POKEY_CH1_CH2) pk_arm_pair(pk, 0, t, audctl, 0);
  else { pk_arm_single(pk, 0, t, audctl, 0); pk_arm_single(pk, 1, t, audctl, 0); }
  if (audctl & POKEY_CH3_CH4) pk_arm_pair(pk, 2, t, audctl, 0);
  else { pk_arm_single(pk, 2, t, audctl, 0); pk_arm_single(pk, 3, t, audctl, 0); }
}

// STIMER (pokey.cpp write_internal(), STIMER_C): reloads every channel and
// forces a known output/filter state. Channels 1/2's filter idles HIGH when
// unfiltered, 3/4's idles LOW - not a typo, it is what real POKEY does (the
// XOR that implements the high-pass filter still runs even when "off").
static inline void pk_stimer_apply(pk_state_t *pk, uint32_t t, uint8_t audctl) {
  for (int c = 0; c < 4; c++) { pk->ch[c].out = 0; pk->ch[c].filt = (c < 2) ? 1 : 0; }
  pk_resync_all(pk, t, audctl);
  pk_recompute_level(pk);
}

// CHAN3's own borrow: toggle its generator, sample (or default) CHAN1's filter,
// then either extend the free-run (joined) or rearm as unlinked (a NATURAL
// rearm - aligned=1).
static inline void pk_fire_ch2(pk_state_t *pk, uint32_t t, uint8_t audctl) {
  pk_fire_generator(pk, &pk->ch[2], pk->regs[5], audctl, t);
  pk->ch[0].filt = (audctl & POKEY_CH1_FILTER) ? pk->ch[0].out : 1;
  if (audctl & POKEY_CH3_CH4) { pk->ch[2].free_run = 1; pk->ch[2].next = t + pk_period_freerun(2, audctl); }
  else pk_arm_single(pk, 2, t, audctl, 1);
}
// CHAN4's own borrow: toggle its generator, sample (or default) CHAN2's filter,
// then either rearm the whole pair fresh (its 16-bit cycle just completed) or
// rearm as unlinked (both NATURAL - aligned=1).
static inline void pk_fire_ch3(pk_state_t *pk, uint32_t t, uint8_t audctl) {
  pk_fire_generator(pk, &pk->ch[3], pk->regs[7], audctl, t);
  pk->ch[1].filt = (audctl & POKEY_CH2_FILTER) ? pk->ch[1].out : 1;
  if (audctl & POKEY_CH3_CH4) pk_arm_pair(pk, 2, t, audctl, 1);
  else pk_arm_single(pk, 3, t, audctl, 1);
}
// CHAN1's own borrow: toggle its generator, then either extend the free-run
// (joined) or rearm as unlinked (NATURAL - aligned=1). The two-tone reset of
// CHAN1 (SKCTL bit 3) is handled by the caller BEFORE this runs, exactly where
// MAME's step_one_clock() places it - see pk_run_to(). That reset is NOT
// natural for CHAN1 (it is borrowed from CHAN2's schedule, unrelated to
// CHAN1's own pulse phase), which is why pk_run_to() passes aligned=0 there.
static inline void pk_fire_ch0(pk_state_t *pk, uint32_t t, uint8_t audctl) {
  pk_fire_generator(pk, &pk->ch[0], pk->regs[1], audctl, t);
  if (audctl & POKEY_CH1_CH2) { pk->ch[0].free_run = 1; pk->ch[0].next = t + pk_period_freerun(0, audctl); }
  else pk_arm_single(pk, 0, t, audctl, 1);
}
// CHAN2's own borrow: toggle its generator, then either rearm the whole pair
// fresh or rearm as unlinked (both NATURAL - aligned=1).
static inline void pk_fire_ch1(pk_state_t *pk, uint32_t t, uint8_t audctl) {
  pk_fire_generator(pk, &pk->ch[1], pk->regs[3], audctl, t);
  if (audctl & POKEY_CH1_CH2) pk_arm_pair(pk, 0, t, audctl, 1);
  else pk_arm_single(pk, 1, t, audctl, 1);
}

// Advance the chip from pk->now to 'until', firing every borrow strictly before
// 'until' in between, then fold the final partial span into pk->mix. Processing
// order for ties (CHAN3, CHAN4, two-tone check, CHAN1, CHAN2) matches
// step_one_clock() exactly, including the obscure case that falls out of it
// for free: a two-tone reset of CHAN1 on the same tick CHAN1 would otherwise
// have independently borrowed CANCELS that borrow, because the reset already
// overwrote 'next' before the CHAN1 branch below is even reached.
static inline void pk_run_to(pk_state_t *pk, uint32_t until, uint8_t audctl, uint8_t skctl) {
  uint32_t mark = pk->now;
  for (;;) {
    uint32_t t = pk->ch[0].next;
    if (pk->ch[1].next < t) t = pk->ch[1].next;
    if (pk->ch[2].next < t) t = pk->ch[2].next;
    if (pk->ch[3].next < t) t = pk->ch[3].next;
    if (t >= until) break;

    pk->mix += (uint32_t)pk->level * (t - mark);
    mark = t;

    if (pk->ch[2].next == t) pk_fire_ch2(pk, t, audctl);
    if (pk->ch[3].next == t) pk_fire_ch3(pk, t, audctl);
    // aligned=1: CHAN1's own inc_chan() for tick t already ran (earlier in
    // step_one_clock(), before this check), same as a natural wrap - the reset
    // just discards its result. The NEW counter's first increment is still a
    // full tick away, exactly like a natural reload, which is what makes this
    // aligned rather than asynchronous like STIMER (whose reset lands BETWEEN
    // ticks, before that tick's own inc_chan() has run at all). Getting this
    // backwards only shows up as a one-tick error on channels using two-tone
    // together with a HICLK partner, which is why it slipped past the
    // TWOTONE-labelled passes in tools/pokey_selftest/ before the pass table
    // added SKCTL fuzzing that enables it incidentally on EVERY pass - see
    // that tool's README.
    if ((skctl & POKEY_SK_TWOTONE) && pk->ch[1].next == t) pk_arm_single(pk, 0, t, audctl, 1);
    if (pk->ch[0].next == t) pk_fire_ch0(pk, t, audctl);
    if (pk->ch[1].next == t) pk_fire_ch1(pk, t, audctl);

    pk_recompute_level(pk);
  }
  pk->mix += (uint32_t)pk->level * (until - mark);
  pk->now = until;
}

// Sample period as a whole number of POKEY-clock ticks plus a 16-bit fraction,
// carried separately - 1,787,520 / 32,000 is 55.86, and truncating to 55 would
// run the whole chip 1.5% sharp. Same technique ym2151.h uses for its own
// sample pacing, same reason: the fraction has to be carried by an accumulator,
// not folded into a wider integer, or it silently rounds to nothing.
#define POKEY_TICKS_INT   55u
#define POKEY_TICKS_FRAC  56360u    /* (1787520<<16)/32000 - (55<<16), i.e. 0.8600 */

// Advance ONE chip by 'elapsed' POKEY-clock ticks and return its
// duration-weighted level accumulator for that span (0 .. 60*elapsed). Reads
// AUDCTL/SKCTL once (matching the bus capture's own one-sample latency budget,
// unchanged from the first version) and resyncs channel scheduling on a STIMER
// strobe, a change in AUDCTL, or the very first call.
//
// Everything that used to be a function-local static here now lives in
// pk_state_t, which is what makes a second instance possible at all: two chips
// sharing one "prev_audctl" would each resync the other on every write.
#if POKEY_DIGI_QUEUE
static inline uint32_t pk_render(pk_state_t *pk, uint32_t elapsed,
                                 uint32_t now_us, uint32_t win0, uint32_t span_us) {
#else
static inline uint32_t pk_render(pk_state_t *pk, uint32_t elapsed) {
#endif
  uint8_t audctl = pk->regs[8];
  uint8_t skctl  = pk->regs[0x0F];
  uint32_t seq   = pokey_stimer_seq[pk->idx];   // volatile: one read
  uint8_t running = (skctl & POKEY_SK_RESET) != 0;

  if (!pk->primed) {
    pk_resync_all(pk, pk->now, audctl);
    pk_recompute_level(pk);
    pk->prev_audctl = audctl;
    pk->seen_stimer_seq = seq;
    pk->was_running = running;
    pk->primed = 1;
  } else if (seq != pk->seen_stimer_seq) {
    pk->seen_stimer_seq = seq;
    pk_stimer_apply(pk, pk->now, audctl);
    pk->prev_audctl = audctl;
  } else if (audctl != pk->prev_audctl) {
    pk_resync_all(pk, pk->now, audctl);
    pk_recompute_level(pk);
    pk->prev_audctl = audctl;
  }

  if (pk->was_running && !running) pk_freeze_snapshot(pk, pk->now, audctl);
  pk->was_running = running;

  if (running) {
#if POKEY_DIGI_QUEUE
    // Replay the register writes that happened DURING the window we are about
    // to render, each at its own instant, instead of pretending they all
    // happened at its start. The engine needs no help for this - pk_run_to()
    // already advances to an arbitrary tick, and because the sample it emits is
    // the time-weighted average over the whole window, a transition placed at
    // the right tick is integrated correctly instead of being aliased.
    //
    // Rendering is one period BEHIND real time on purpose: at the moment this
    // runs, only events already in the past exist, so the window that can be
    // reconstructed exactly is the one that has just elapsed. Costs 31us of
    // latency, which nothing here can hear.
    {
      const uint32_t target = pk->now + elapsed;
      const uint32_t q = pk->idx;
      // now_us / win0 / span_us are computed ONCE per output sample by the
      // caller and passed in, so both chips place their events on the same
      // window rather than each measuring a slightly different one.
      while (pk_q_tail[q] != pk_q_head[q]) {
        uint32_t i = pk_q_tail[q] & (POKEY_Q_SIZE - 1u);
        uint32_t t = pokey_queue[q][i].t_us;
        if ((int32_t)(t - now_us) >= 0) break;            // belongs to a later window
        uint32_t off = (int32_t)(t - win0) > 0 ? (t - win0) : 0u;
        if (off > span_us) off = span_us;
        // Proportional placement inside the window: using the window's ACTUAL
        // microsecond span rather than the nominal 31.25us keeps events in the
        // right place even when the pacing loop wakes up a little late.
        // 32-bit on purpose: off <= span_us <= 1000 and elapsed <= 56, so the
        // product is at most 56000 and fits easily. A uint64_t here would pull
        // in the 64-bit software divide instead of the 32-bit one - the exact
        // trap the note above this function's own division warns about.
        uint32_t tick = pk->now + (off * elapsed) / span_us;
        if ((int32_t)(tick - target) > 0) tick = target;
        if ((int32_t)(tick - pk->now) > 0) pk_run_to(pk, tick, audctl, skctl);
        pk->regs[pokey_queue[q][i].reg] = pokey_queue[q][i].val;
        pk_recompute_level(pk);      // volume/waveform may have changed mid-window
        pk_q_tail[q] = pk_q_tail[q] + 1u;
      }
    }
#endif
    pk_run_to(pk, pk->now + elapsed, audctl, skctl);
  } else {
    // Held in reset: no channel may borrow and the poly phase pins at 0 (see
    // pk->epoch), but the OUTPUT sample clock does not stop - the level is
    // simply constant over this whole span, so folding it in directly is
    // exact, not an approximation.
    pk->epoch = pk->now;
    pk->mix += (uint32_t)pk->level * elapsed;
  }

  uint32_t mix = pk->mix;
  pk->mix = 0;
  return mix;
}

// Produce one output sample. Called POKEY_SAMPLE_RATE times per second.
//
// COST OF THE SECOND CHIP, and why it is not paid by the ~424 single-POKEY
// files in the library: chip 1 is rendered only once a write has actually
// reached it (pokey_live[1]), which no cart but a dual one ever does. What a
// single-POKEY cart adds here is one byte load and one branch per sample -
// 32 000 times a second, against a core running at POKEY_CLOCK_KHZ.
//
// MIXING. Two real POKEYs both drive the console's single AUD IN line, so
// their outputs sum; linear summation is what MAME, JS7800 and test7800 all do
// for the 7800 (POKEY_IMPROVEMENT.md 5.3). The full-scale divisor therefore
// becomes 120 rather than 60 when BOTH chips are playing - each contributes
// half the swing, and a dual cart voicing the same part on both chips comes
// out at the same loudness as a single chip playing it alone, instead of
// clipping. The divisor stays at 60 when only one chip has ever been written,
// which keeps every existing POKEY cart bit-for-bit as loud as before AND
// covers "POKEY Tester (810)": a lone chip that happens to sit in the second
// slot must not be rendered at half volume just because the first slot exists.
static inline uint16_t pokey_next_sample(void) {
  static uint32_t frac = 0;
#if POKEY_DIGI_QUEUE
  static uint32_t pk_win_us = 0;     // real time at the END of the previous window
#endif

  frac += POKEY_TICKS_FRAC;
  uint32_t elapsed = POKEY_TICKS_INT + (frac >> 16);
  frac &= 0xFFFFu;

#if POKEY_DIGI_QUEUE
  // Measured once for BOTH chips - see pk_render()'s digi block.
  uint32_t now_us = time_us_32();
  uint32_t win0   = pk_win_us ? pk_win_us : now_us;   // first call: empty window
  pk_win_us = now_us;
  uint32_t span_us = now_us - win0;
  if (span_us == 0) span_us = 1;
  // Clamp so the multiply in pk_render() cannot overflow after a stall (a
  // hiccup could otherwise make span_us enormous). 1000us is ~32 sample
  // periods; anything beyond that is not a window worth reconstructing anyway.
  if (span_us > 1000u) span_us = 1000u;
#define PK_RENDER(p) pk_render((p), elapsed, now_us, win0, span_us)
#else
#define PK_RENDER(p) pk_render((p), elapsed)
#endif

#if POKEY_DIAG_E10
  // E10 DIAGNOSTIC: chip 1 only, chip 0 silent - 0.46's audio behaviour with
  // 0.47's bus side. See the switch's comment above.
  uint32_t mix   = pokey_live[1] ? PK_RENDER(&pk_st[1]) : 0u;
  uint32_t scale = 60u;
#elif POKEY_DIAG_E11
  // E11 DIAGNOSTIC: chip 0 only, chip 1 silent - the mirror of E10.
  uint32_t mix   = PK_RENDER(&pk_st[0]);
  uint32_t scale = 60u;
#else
  uint32_t mix   = PK_RENDER(&pk_st[0]);
  uint32_t scale = 60u;
  if (pokey_live[1]) {
    mix += PK_RENDER(&pk_st[1]);
    if (pokey_live[0]) scale = 120u;
  }
#endif
#undef PK_RENDER

  // 4 channels x volume 15 = 60 max per chip, elapsed <= 56 (POKEY_TICKS_INT
  // plus one carried tick) - mix never exceeds 2*60*56=6720, so mix*PWM_WRAP
  // fits a plain uint32_t with room to spare (max ~3.43M against a ~4.29G
  // ceiling). This matters on a core with no hardware divide: a needless
  // uint64_t here would pull in the 64-bit software division routine instead
  // of the 32-bit one, roughly doubling the one division this function cannot
  // avoid.
  uint32_t out = (mix * POKEY_PWM_WRAP) / (scale * elapsed);
  if (out > POKEY_PWM_WRAP) out = POKEY_PWM_WRAP;
  return (uint16_t)out;
}

#ifndef POKEY_HOST_TEST

static inline void pokey_audio_init(void) {
  gpio_set_function(POKEY_AUDIO_PIN, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(POKEY_AUDIO_PIN);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, POKEY_PWM_WRAP);   // no clock divider: carrier as high
  pwm_init(slice, &cfg, true);                 // as possible, ~488kHz at 250MHz
  pwm_set_gpio_level(POKEY_AUDIO_PIN, 0);
}

// Core 0 audio loop. Runs only while a POKEY cart is playing; returns never.
// Sample period is a whole number of microseconds plus a 16-bit fraction,
// tracked separately for the exact reason POKEY_TICKS_FRAC above is: plain
// integer microseconds (31 instead of 31.25) would run 0.8% sharp. This is the
// same bug 2026-08-24 found and fixed in ym2151.h's pacing - see that file's
// header for the "time_us_32() << 16 throws away the top 16 bits" trap this
// form avoids by keeping the fraction in ITS OWN accumulator.
#if POKEY_DIAG_E3
// E3 only: somewhere for the sample to go now that the pin does not get it.
// volatile so the compiler cannot work backwards from "nobody reads this" and
// delete the synthesis itself - which would turn E3 into a test of an idle
// core 0 rather than a test of a silent pin.
static volatile uint16_t pokey_diag_sink = 0;
#endif

static inline void pokey_run(void) {
  // Core 1 now performs a REAL PLL reconfiguration for a POKEY cart (250 ->
  // POKEY_CLOCK_KHZ), where before 0.33 it merely re-set the 250MHz already in
  // force - a no-op with nothing to race against. Core 0 can reach this
  // function before core 1 gets there, and configuring PWM against a clock
  // that is about to change underneath it is exactly what came back from
  // hardware on 2026-08-24 as "no audio at all". Same bounded wait, same
  // reason as ym_run() - see emu_clock_ready in ym2151.h. Bounded because a
  // flag that never arrives must cost a slightly wrong carrier, not silence
  // forever.
  uint32_t wait_t0 = time_us_32();
  while (!emu_clock_ready && (uint32_t)(time_us_32() - wait_t0) < 200000u)
    tight_loop_contents();
#if !POKEY_DIAG_E3
  pokey_audio_init();
#endif
  const uint32_t period_us_int  = 31u;
  const uint32_t period_us_frac = 16384u;   /* (1000000<<16)/32000 - (31<<16) */
  uint32_t next_us = time_us_32();
  uint32_t frac_us = 0;
  while (1) {
    uint32_t now = time_us_32();
    if ((int32_t)(now - next_us) >= 0) {
#if POKEY_DIAG_E3
      // Full-rate synthesis, pin untouched: pokey_audio_init() was skipped, so
      // GPIO stays the plain input it is under a cart with no POKEY at all.
      // That is the point - it makes a POKEY cart electrically identical, on
      // this one pin, to the carts whose picture is clean.
      pokey_diag_sink = pokey_next_sample();
#else
      pwm_set_gpio_level(POKEY_AUDIO_PIN, pokey_next_sample());
#endif
      frac_us += period_us_frac;
      next_us += period_us_int + (frac_us >> 16);
      frac_us &= 0xFFFFu;
    }
  }
}

#endif // POKEY_HOST_TEST

#endif // POKEY_H
