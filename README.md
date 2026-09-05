> ### ⚠️ HEADS UP, TRAVELER!
>
> This is a **fork** of the original repo by Andrea Ottaviani: https://github.com/aotta/PicoA10400
>
> Changes here are highly experimental, occasionally hand-crafted by an AI with more confidence than reading comprehension, and come with an industry-standard **zero guarantee that anything works**. Flash at your own risk, and maybe keep a fire extinguisher nearby (for the vibes, not because Pico boards actually catch fire... probably).
>
> If you still decide to use it, do the right thing and drop a ⭐ on aotta's original repo. He did the actual hard work.

# PicoA10400-HR — what this build gives you

- **Sound in the games that need a POKEY**
  _Titles built around the POKEY chip — **Ballblazer**, **Commando**, **Ace of Aces**,
  **Basketbrawl**, **Apple Snaffle** and the rest — used to run in silence. The chip is
  now emulated in software and every cartridge tested that actually drives one plays
  its music. No extra hardware: the 7800's cartridge audio line was already wired on
  this board and simply unused. The synthesis runs on the core that sits idle once a
  game starts, so the picture costs nothing for the sound._

  _The emulation has since been rebuilt around how the chip actually works, and that
  turned up two faults rather than rough edges: **everything had been playing an octave
  too low**, and one whole class of sound — the paired 16-bit channels, used by more
  than a quarter of the POKEY library — **made no sound at all**. Both are fixed, along with
  the filters, the noise generators and two-tone mode. The result is checked against
  MAME's own POKEY clock by clock, not by ear._

- **FM sound — the YM2151 is emulated**
  _The FM chip from Atari's XM expansion module is carried on the board by a few 7800
  titles — **1942**, **Wonder Boy**, **Pac-Man Collection 40th Anniversary**,
  **Block'Em Sock'Em** and some forty music demos — and all of them used to play in
  silence. It is now emulated in software, again with no extra hardware. There is no
  useful "roughly right" FM, so this is a full port of MAME's core rather than an
  approximation, checked against it sample for sample. The dozen largest images are
  still over the 144KB limit and stay out of reach._

- **Carts that declare an extra chip now get their real mapper**
  _A cartridge announcing a POKEY or a YM2151 in its `.a78` header was read as a plain
  flat ROM whatever its actual bankswitching, which is why **Wonder Boy** and both
  **Pac-Man Collection 40th Anniversary** dumps never started. Those two header bits
  say what else sits on the board, not how it banks, and are now decoded the way MAME
  does it._

- **Many fixes in cartridge types**
  _Every type's mapping was verified against MAME and the ProSystem emulator, and
  many games work now that would not start at all before — **Double Dragon**,
  **Rampage**, **Alien Brigade** and some **Ikari Warriors** dumps among them._

- **Pitfall II runs — the DPC chip is emulated**
  _**Pitfall II - Lost Caverns** carries a co-processor in the cartridge, not just a
  bankswitching scheme: counters that stream graphics data, and a three-voice music
  generator. The cart used to answer nothing at all for those images. It now plays,
  music included, and so do the other DPC titles — no extra hardware, and it works
  on the Pico 2 board too, because the DPC makes no sound of its own: it hands the
  console a volume level and the game does the rest._

- **Supercharger cassettes play**
  _The Starpath/Arcadia titles — **Communist Mutants from Space**, **Dragonstomper**,
  **Escape from the Mindmaster** and the rest of the tape library — now load and run
  on an Atari 2600, multi-load games included. They never worked on this cart before._

- **UA Ltd cartridges run — 16 more titles**
  _**Pleiades**, **Funky Fish**, **Time Pilot**, **Vanguard**, **Zaxxon**, **Galaxian**,
  **Fathom**, **Gingerbread Man**, **Hobo** and the Brazilian Digivision conversions were
  read as ordinary F8 carts: the menu listed them, they started, and the game did not
  work. They pick a bank through an address below the cartridge window, which nothing
  here had been watching for. **Mickey** wants its two banks the other way round — rename
  that file to `.UAS`._

- **CommaVid cartridges run — 17 more titles**
  _**MagiCard**, **Video Life** and the MagiCard sample programs answered nothing at all:
  the menu listed them, the cartridge started, and the screen stayed black. These boards
  carry a kilobyte of RAM alongside a 2KB ROM, read through one window and written
  through another, and none of that was being served. They run now._

  _A 4KB CommaVid file is not a doubled ROM — it is MagiCard's own save format, a
  kilobyte of stored RAM followed by the program, which is how the sample programs and
  the saved screens are distributed. Those files now show what was saved in them instead
  of a mirror of their own code. No other flashcart firmware this project draws on loads
  them; only Stella does._

- **Amiga's Power Play Arcade cartridges run — 7 more images**
  _The **Power Play Arcade Video Game Album** and the Amiga prototypes built on the same
  board — **3-D Havoc**, **Surf's Up**, **S.A.C. Alert** — were read as ordinary 4K, F8,
  F6 and F4 carts: the menu listed them, they started, and the bank switching did nothing
  at all. This board is the only one here that takes its bank number off the **data** bus
  instead of an address, and it commits the switch on the reset vector itself — which is
  exactly how a game lands on the entry point of the bank it is jumping into._

  _The complete 32KB dump of the album — all eight games — was not even on the list of
  things to fix: it had been sitting there as an F4 cart, and only re-measuring the whole
  library turned it up. One warning: the 16KB dump of the same album is **partial**. Its
  own menu points at banks that are not in the file, and no firmware can repair that._

- **Star Castle Arcade starts at all — 15 images, 30 files**
  _The Harmony cartridge these builds were written for can keep a high-score table in its
  own flash, and the game asks for that table **on the way in**, about two seconds after
  power-on. It asks by reading one address and waiting for the cartridge to say "not busy
  any more". Nothing here answered that address, so what went out was the plain ROM byte
  underneath it — and in every bank of every copy in the library that byte happens to have
  the "busy" bit set. The game sat on its waiting screen for good; the loop it waits in
  has no other way out written into it._

  _The cartridge now says "done, nothing transferred", which is what a real Harmony says
  once a transfer has finished, so the game goes through and plays. The score table starts
  empty and **is not saved yet** — writing to flash while the other core is driving the
  cartridge bus is a separate problem, and the same one the 7800 High Score Cart needs.
  Confirmed on a real console._

- **Six more bankswitching schemes answer the bus at all — 19 more files**
  _`DF`, `DFSC`, `EFSC`, `0840` and the Wickstead Design board behind **Pursuit of the
  Pink Panther** were *recognised* by the firmware and then not emulated. The cartridge
  did not misbehave — it stayed silent and the screen stayed black. They all run now._

  _`DF` had a second problem underneath the first: the firmware looked for the wrong
  four-character signature, so a `DF` cartridge had never been detected at all. Four
  files in the library were dead for that reason alone, and nothing reported it._

  _The Pink Panther prototype was the one with teeth. Its board delays every bank switch
  by four cycles, so the jump instruction that triggers the switch still finishes reading
  itself out of the **old** bank and only its destination comes from the new one. Get
  that wrong and the game does not degrade — it falls over within a few frames. The only
  dump in circulation is also a known bad one, with two of its eight 1KB segments
  swapped, which the firmware now puts back._

  _Still honestly broken: **BF** and **BFSC** cartridges are 256KB and the ROM buffer is
  144KB, so they stay marked in red and unusable. There are eight such files and every
  one of them is a bankswitching test program rather than a game._

- **Bankset cartridges run on the 7800 — 9 more files, StoneAge among them**
  _A Bankset cart holds two complete images in one file: one for the processor, one for
  the video chip, at the very same addresses, and the console's HALT line picks between
  them. The official Bankset test suite and **StoneAge (Final)**, a finished homebrew
  game, were read as ordinary flat carts, so the half meant for the video chip was being
  fed to the processor. All nine files that fit in memory run now, picture and sound._

  _Getting there needed one thing no source documents: the HALT line has to be trusted
  only after it has stayed asserted for several bus accesses in a row, because the
  processor keeps using the bus for a moment after the video chip has claimed it. The
  official RAM test demo then needed two further rounds before its music came back — it
  plays its song out of cartridge RAM, and the cart was losing every write the 6502 makes
  with an indexed address. That demo still shows a little flicker on its on-screen text._

  _The five 2x128K titles — **Bubble Bobble** among them — do not fit: 256KB of image
  cannot coexist with everything else in the RP2040's 256KB of RAM, so those load the
  processor's half only and run with the wrong graphics. Bankset needs the HALT line,
  which the Pico 2 board does not have._

- **Three more cartridge types supported**
  _**mRAM** and **VersaBoard** on the 7800, **4KSC** on the 2600 — boards the cart
  simply did not answer before, so they showed nothing at all. The **Rescue On
  Fractalus** prototype is the pick of the bunch; the rest are demos that finally
  scroll the way they were meant to. Each one follows MAME's own decoding, and the
  address mapping was checked against it exhaustively before anything was flashed._

- **Many flickering artifacts are fixed**
  _Reading the bus at the wrong point in the CPU's write cycle caused flickering
  artifacts during gameplay; with that fixed you should get a visibly cleaner picture._

  _Two more were tracked down on a real console afterwards. Cartridges with a POKEY
  showed brief flickering lines — **Bentley Bear's Crystal Quest**, **Donkey Kong
  PK-XM**, **Commando** — which are gone now that those carts run the faster clock the
  FM ones already used. And the cartridge's data lines had been driven at the chip's
  default strength since the project began, which turned out to be marginal for this
  bus: the title that streams data hardest, **LZSS Player**, would not start at all.
  Driving them harder fixes it. Both were established by shipping builds that changed
  one thing at a time and testing them on the console, not by reasoning about it._

- **ROMs of unusual sizes now run**
  _Carts other than 16/32/48KB — 8KB and 28KB among them — used to give a white
  screen. They are now mapped the way real hardware does it, whatever the size._

- **Large ROMs instead of an error**
  _Images over 144KB are no longer refused: they load truncated and are clearly
  colour-marked in the listing, so you can just try them. False warnings for `.a78`
  files that fit exactly are gone too._

- **Easier browsing**
  - _The listing is sorted: directories first, then alphabetically._
  - _Subdirectories no longer vanish in folders holding hundreds of files._
  - _The full file name scrolls under the cursor, despite the 12-character row._
  - _Long ROM names display and load correctly — now up to 127 characters._
  - _When the listing does not fit, the footer shows how many entries you can see._

  _That last limit had been 79, and in a full library about one file in eight is longer
  than that. Every one of them was listed as a broken row: it sorted to the top, would
  not scroll and would not start — while the game itself was perfectly fine. It took a
  console to find, because nothing in the firmware reports it. A name too long for the
  new limit is now drawn as `NAME TOO LNG` in the colour oversized ROMs get, so a name
  the cart cannot read says so instead of posing as a file you can pick._

  _And picking an entry the cart cannot open no longer hangs it. That used to need a
  power cycle, and you could trigger it just by changing files over USB while the menu
  was still on screen._

- **Stability**
  Several buffer overflows were fixed, including one where a large ROM corrupted the
  USB drive until the next power cycle. The emulation clock was lowered to 250MHz
  after measurements showed a higher one bought no timing margin at all, so the chip
  runs cooler and calmer — with one exception since added: cartridges carrying a POKEY
  or a YM2151 run at 300MHz, because on those the extra margin turned out to be worth
  something on screen.

_Supercharger titles run on an Atari 2600; on an Atari 7800 in its 2600 mode they
do not, and that one is not solved._

_A few music demos — **Bloodfighter**, **LZSS Player**, **RMT POKEY Player**, **White
Lamp** — show a narrow band of smearing down the left edge of the picture, about a
seventh of the screen wide. It is still there and it is not for want of looking: the
clock in both directions, the audio pin, the sound synthesis, the cartridge's POKEY
window, the bus timing and all four available drive strengths were each tested
separately on a real console, and none of them moves it. An emulator does not
reproduce it either. Whatever it is, it is not in the firmware, so it is out of reach
from this side._

_Not fixed, and out of the cartridge's reach: an NTSC ROM on a PAL console may run
too fast and show interference along the bottom of the screen — use a PAL version
where one exists. Some 2600 schemes stay out of range for a harder reason — **DPC+**
and **CDFJ** cartridges carry their own ARM program and expect a processor in the
cartridge to run it, which is what a Harmony has and this board does not._

_One POKEY is emulated, so the handful of demos wiring up two get roughly half their
parts. The audio line exists only on **PicoA10400** — the Pico 2 board uses the
standard Raspberry Pi Pico footprint, where the cartridge bus consumes every
available pin, so **Pico2A10400 stays silent** whether the cartridge asks for a POKEY
or a YM2151. DPC music is the exception and plays on both boards, for the reason
given above._

_The DPC music generator is driven from a free-running clock rather than from the
console's own cycles. Games where music is an accompaniment — Pitfall II among them —
do not care. The few DPC **music demos**, which carry no graphics at all and tie their
display to the music, can come up differently from one power-on to the next._

_Tip: the menu background tells you which build is flashed — **blue is the PAL build,
red is the NTSC one**. Handy, since a region mismatch is easy to create and looks like
a fault._

# PicoA10400

Flashcart for Atari 2600 and Atari 7800 based on Pico "Purple" clone, easy to build and cheap.
This is a "double-face" flashcart, it could be used for both Atari 2600 and Atari 7800, simply rotating it and inserting the cart in different Atari!!
It doesn't support all bank-switching schemas, but enough to enjoy your A2600 / A7800 with a single flash-carts!!

A special thanks to other opensource project for Atari multicarts from which i got a lot of info, ideas and also code:
https://github.com/robinhedwards/UnoCart-2600

https://github.com/karrika/Otaku-flash

![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_06.jpg)

**WARNING!** "purple" Pico has not the same pinout of original Raspberry "green" ones, you MUST use the clone or you may damage your hardware.
Also note that the battery used is a RECHARGEABLE LIR2032, if you want to use a NON reachargeable battery you must add a diode in circuit!!!

Tested only on PAL consoles so far, feel free to send comments and feedback on AtariAge thread:
https://forums.atariage.com/topic/374297-picoa10400-preview/

**NOTE** Please look at picture for soldering side of the components, or your shell won't close!! they are different from the pcb mask!!!

![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_01.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_02.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_03.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_04.jpg)

Also added a Raspberry Pico 2 version, relative files are named Pico2A10400. It works but consider its smaller flash size for roms (3mb):

![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/Pico2A10400.jpg)

Gerbers file are provided for the PCB, add you pico clone, and flash the firmware ".uf2" in the Pico by connecting it while pressing button on Pico and drop it in the opened windows on PC.
After flashed with firmware, and every time you have to change your ROMS repository, you can simply connect the Pico to PC and drag&drop "BIN" files into.

**NOTE 2** Due to different timing of PicoA10400 and the Atari consoles, that can't be resetted, the flashcart MUST BE POWERED ON (with POWER SWITCH ON CART) BEFORE POWERING THE CONSOLE!!! Also, some games and ALL A7800 GAMES NEEDS THAT THE CONSOLE IS POWERED OFF THEN POWERED ON TO START!!!!

Even if the diode should protect your console, **DO NOT CONNECT PICO WHILE INSERTED IN A POWERED ON CONSOLE!**

19th january 2025: added Pico 10400 Alternative Version by XAD, with improvements in pcb and shell: https://www.nightfallcrew.com/17/01/2025/picoa10400-flashcart-for-atari-2600-7800/
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_08.jpg)
