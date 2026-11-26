#pragma once
// Include from exactly one .cpp per test suite. PlatformIO parses doctest's
// per-assert output, so report successes too. Unlike PIO's example we keep
// doctest's exit code, so a failing test can never look green in CI.
#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest.h>

int main(int argc, char **argv) {
  doctest::Context context;
  context.setOption("success", true);
  context.applyCommandLine(argc, argv);
  return context.run();
}
