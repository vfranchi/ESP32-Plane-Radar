#pragma once
// Minimal check macro: no framework, non-zero exit on failure.
#include <cstdio>

inline int& testFailures() {
  static int n = 0;
  return n;
}

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
      ++testFailures();                                                    \
    }                                                                      \
  } while (0)

inline int testSummary(const char* name) {
  if (testFailures() == 0) {
    std::printf("OK: %s\n", name);
    return 0;
  }
  std::printf("FAILED: %s (%d checks)\n", name, testFailures());
  return 1;
}
