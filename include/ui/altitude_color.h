#pragma once
/**
 * Aircraft colour by altitude, from the legend Vitor supplied (2026-10-19):
 * red-orange at the bottom, hue sweeping through amber/yellow/lime/green/cyan/blue
 * to violet at 40k ft.
 *
 * The frame sprite is RGB332 and the panel is BGR (kDisplayRgbOrder), so the
 * sprite stores 3 bits of BLUE / 3 of green / only 2 of red: the continuous HSL
 * ramp collapses to 26 distinct screen colours. kRamp holds the FIRST altitude at
 * which each of those pixels appears -- the maximum resolution the panel can show,
 * and it means no colour maths (and no float) at runtime.
 *
 * The table is GENERATED: edit the stops in scripts/gen_altitude_ramp.py, re-run
 * it and paste the block; `python3 scripts/gen_altitude_ramp.py --check` fails when
 * the two disagree. Never hand-edit numbers inside the generated block. The
 * generator clamps lightness to a monotone envelope so the ramp never steps
 * backwards in altitude (see the generator's docstring for why).
 *
 * Unknown altitude (-1: "ground", or the feed sent no alt_baro/alt_geom) is drawn
 * neutral grey.
 */
#include <cstdint>

namespace ui::altitude {

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct Step {
  int32_t min_ft;
  Rgb rgb;
};

/**
 * The feed reported no altitude number; drawn neutral. A sentinel instead of -1
 * because the type must also hold 60000 ft, which does not fit int16_t.
 */
constexpr uint16_t kUnknownAltFt = 0xFFFF;

constexpr Rgb kNeutral = {170, 180, 195};

constexpr int kRampCount = 26;

// <generated: ramp>
constexpr Step kRamp[kRampCount] = {
    {    0, {249,  55,   6}},  // sprite pixel B0 G1 R3
    {  130, {248,  64,   9}},  // sprite pixel B0 G2 R3
    {  610, {243,  96,  18}},  // sprite pixel B0 G3 R3
    { 1300, {241, 128,  28}},  // sprite pixel B0 G4 R3
    { 1730, {241, 139,  32}},  // sprite pixel B1 G4 R3
    { 2750, {241, 160,  34}},  // sprite pixel B1 G5 R3
    { 4490, {241, 192,  34}},  // sprite pixel B1 G6 R3
    { 6110, {241, 224,  34}},  // sprite pixel B1 G7 R3
    { 8120, {191, 241,  34}},  // sprite pixel B1 G7 R2
    { 9040, {127, 241,  34}},  // sprite pixel B1 G7 R1
    { 9950, { 63, 241,  34}},  // sprite pixel B1 G7 R0
    {12090, { 34, 241,  64}},  // sprite pixel B2 G7 R0
    {13290, { 34, 241,  96}},  // sprite pixel B3 G7 R0
    {14490, { 34, 241, 128}},  // sprite pixel B4 G7 R0
    {15700, { 34, 241, 160}},  // sprite pixel B5 G7 R0
    {16900, { 34, 241, 192}},  // sprite pixel B6 G7 R0
    {18100, { 34, 241, 224}},  // sprite pixel B7 G7 R0
    {19410, { 34, 223, 241}},  // sprite pixel B7 G6 R0
    {20850, { 34, 191, 241}},  // sprite pixel B7 G5 R0
    {22530, { 34, 159, 241}},  // sprite pixel B7 G4 R0
    {24210, { 34, 127, 241}},  // sprite pixel B7 G3 R0
    {25890, { 34,  95, 241}},  // sprite pixel B7 G2 R0
    {27570, { 34,  63, 241}},  // sprite pixel B7 G1 R0
    {29960, { 64,  49, 242}},  // sprite pixel B7 G1 R1
    {33560, {128,  49, 242}},  // sprite pixel B7 G1 R2
    {37170, {192,  49, 242}},  // sprite pixel B7 G1 R3
};
// </generated: ramp>

/**
 * Colour for an altitude in ft; neutral grey when unknown.
 *
 * Not constexpr: the framework appends -std=gnu++11 to the compile line (after
 * the project's gnu++17), so a looping constexpr function is rejected on the
 * device -- the host tests build with -std=c++17 and do not catch that.
 */
inline Rgb colorForAltFt(uint16_t alt_ft) {
  if (alt_ft == kUnknownAltFt) {
    return kNeutral;
  }
  Rgb out = kRamp[0].rgb;
  for (int i = 1; i < kRampCount; ++i) {
    if (alt_ft < kRamp[i].min_ft) {
      break;
    }
    out = kRamp[i].rgb;
  }
  return out;
}

}  // namespace ui::altitude
