# PlatformIO extra script for the bootloader env: the platform's bare-metal
# ARM setup adds `-s` to ranlib's flags, which this toolchain's ranlib
# rejects (a warning per library; ar already writes the index). Run ranlib
# without its flags instead; this has to be a pre script, as libraries get
# their build environment before post scripts run.
Import("env")
env.Replace(RANLIBCOM="$RANLIB $TARGET")
