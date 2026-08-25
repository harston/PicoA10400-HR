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

- **Three more cartridge types supported**
  _**mRAM** and **VersaBoard** on the 7800, **4KSC** on the 2600 — boards the cart
  simply did not answer before, so they showed nothing at all. The **Rescue On
  Fractalus** prototype is the pick of the bunch; the rest are demos that finally
  scroll the way they were meant to. Each one follows MAME's own decoding, and the
  address mapping was checked against it exhaustively before anything was flashed._

- **Many flickering artifacts are fixed**
  _Reading the bus at the wrong point in the CPU's write cycle caused flickering
  artifacts during gameplay; with that fixed you should get a visibly cleaner picture._

- **ROMs of unusual sizes now run**
  _Carts other than 16/32/48KB — 8KB and 28KB among them — used to give a white
  screen. They are now mapped the way real hardware does it, whatever the size._

- **Large ROMs instead of an error**
  _Images over 144KB are no longer refused: they load truncated and are clearly
  colour-marked in the listing, so you can just try them. False warnings for `.a78`
  files that fit exactly are gone too._

- **Easier browsing**
  - _The listing is sorted: directories first, then alphabetically._
  - _Vanish in folders holding hundreds of files._
  - _The full file name scrolls under the cursor, despite the 12-character row._
  - _Long ROM names display and load correctly._
  - _When the listing does not fit, the footer shows how many entries you can see._

- **Stability**
  Several buffer overflows were fixed, including one where a large ROM corrupted the
  USB drive until the next power cycle. The emulation clock was lowered to 250MHz
  after measurements showed a higher one bought no timing margin at all, so the chip
  runs cooler and calmer.

_Supercharger titles run on an Atari 2600; on an Atari 7800 in its 2600 mode they
do not, and that one is not solved._

_Not fixed, and out of the cartridge's reach: an NTSC ROM on a PAL console may run
too fast and show interference along the bottom of the screen — use a PAL version
where one exists. Some 2600 schemes stay out of range for a harder reason — **DPC+**
and **CDFJ** cartridges carry their own ARM program and expect a processor in the
cartridge to run it, which is what a Harmony has and this board does not. The
**YM2151** FM chip used by some 7800 homebrew is not emulated either, so those
titles stay silent._

_One POKEY is emulated, so the handful of demos wiring up two get roughly half their
parts. The audio line exists only on **PicoA10400** — the Pico 2 board uses the
standard Raspberry Pi Pico footprint, where the cartridge bus consumes every
available pin, so **Pico2A10400 stays silent** no matter what the cartridge asks for.
DPC music is the exception and plays on both boards, for the reason given above._

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
