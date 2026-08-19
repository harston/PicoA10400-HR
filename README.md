> ### ⚠️ HEADS UP, TRAVELER!
>
> This is a **fork** of the original repo by Andrea Ottaviani: https://github.com/aotta/PicoA10400
>
> Changes here are highly experimental, occasionally hand-crafted by an AI with more confidence than reading comprehension, and come with an industry-standard **zero guarantee that anything works**. Flash at your own risk, and maybe keep a fire extinguisher nearby (for the vibes, not because Pico boards actually catch fire... probably).
>
> If you still decide to use it, do the right thing and drop a ⭐ on aotta's original repo. He did the actual hard work.

# PicoA10400-HR — what this build gives you

- **Many fixes in cartridge types**
  _Every type's mapping was verified against MAME and the ProSystem emulator, and
  many games work now that would not start at all before — **Double Dragon**,
  **Rampage**, **Alien Brigade** and some **Ikari Warriors** dumps among them._

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

_Not fixed, and out of the cartridge's reach: an NTSC ROM on a PAL console may run
too fast and show interference along the bottom of the screen — use a PAL version
where one exists. PicoA10400 has no POKEY chip, so titles needing one run without
their music._

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
