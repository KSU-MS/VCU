# PlatformIO extra script for the native env.
#
# ksu-fw-common bundles Arduino-only sources (logger, gps) next to the portable
# can_tools.cpp we actually want. PIO builds every source in a library, so drop
# the Arduino ones here instead of forking the library.
Import("env")

SKIP = ("logger.cpp", "gps.cpp")


def skip_arduino_sources(node):
    path = node.get_path()
    if "ksu-fw-common" in path and path.endswith(SKIP):
        return None
    return node


env.AddBuildMiddleware(skip_arduino_sources)
