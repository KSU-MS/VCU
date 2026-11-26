# flasherx

`FlashTxx.c` and `FlashTxx.h` copied unmodified from
[joepasquariello/FlasherX](https://github.com/joepasquariello/FlasherX) at
commit `9d5067bf168cd8937201b1fc4b120836464a77fa` (2022-10-15). That code is
released into the public domain by its authors (Niels A. Moseley, Jon Zeeff,
Deb Hollenback, Paul Stoffregen, Frank Boesing, Joe Pasquariello).

Only used by `src/ota_teensy.cpp`, which needs `flash_move` (copies the staged
image over the running firmware from RAM, then reboots) and the Teensy core's
`eepromemu_flash_*` declarations. Everything else about the update protocol
lives in `lib/ota`.

Vendored rather than pulled in as a dependency because FlasherX is an Arduino
sketch, not a PlatformIO library.
