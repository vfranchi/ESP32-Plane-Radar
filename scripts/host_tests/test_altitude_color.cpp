/**
 * Host test for the altitude -> colour ramp used by the radar icons.
 *
 * The ramp is already quantised to the pixels the panel actually shows, so the
 * properties worth checking are that every step is a DISTINCT pixel, that the
 * ramp never returns to a pixel it already used (a step that goes forward and
 * back is what the monotone-lightness envelope exists to prevent), and that it
 * is fine enough to read as a gradient.
 */
#include "test_util.h"
#include "ui/altitude_color.h"

namespace {

/**
 * The pixel the rgb332 frame sprite stores for this logical colour.
 *
 * panelColor565() swaps red and blue (the panel is BGR), so the sprite's 3-bit
 * field carries the logical BLUE channel and its 2-bit field the logical red.
 * Quantising as (r>>5, g>>5, b>>6) here would validate the ramp against pixels
 * the panel never shows -- that mistake is why the first draft of this plan
 * missed a backward step around 2k ft.
 */
struct Q332 {
  int b;
  int g;
  int r;
};

Q332 quantise(ui::altitude::Rgb c) { return Q332{c.b >> 5, c.g >> 5, c.r >> 6}; }

bool samePixel(Q332 a, Q332 b) {
  return a.r == b.r && a.g == b.g && a.b == b.b;
}

}  // namespace

int main() {
  using ui::altitude::colorForAltFt;
  using ui::altitude::kNeutral;
  using ui::altitude::kRamp;
  using ui::altitude::kRampCount;
  using ui::altitude::kUnknownAltFt;

  // A fine ramp, not a handful of bands.
  CHECK(kRampCount >= 25);
  CHECK(kRamp[0].min_ft == 0);

  // Thresholds strictly ascending, or the scan picks the wrong step.
  for (int i = 1; i < kRampCount; ++i) {
    CHECK(kRamp[i].min_ft > kRamp[i - 1].min_ft);
  }

  // Unknown altitude is neutral grey, not "zero feet orange".
  CHECK(colorForAltFt(kUnknownAltFt).r == kNeutral.r);
  CHECK(colorForAltFt(kUnknownAltFt).g == kNeutral.g);
  CHECK(colorForAltFt(kUnknownAltFt).b == kNeutral.b);

  // Each step starts at its own altitude and returns its own colour.
  for (int i = 0; i < kRampCount; ++i) {
    CHECK(colorForAltFt(kRamp[i].min_ft).r == kRamp[i].rgb.r);
    CHECK(colorForAltFt(kRamp[i].min_ft).g == kRamp[i].rgb.g);
    CHECK(colorForAltFt(kRamp[i].min_ft).b == kRamp[i].rgb.b);
  }
  // One foot under a threshold still belongs to the previous step.
  for (int i = 1; i < kRampCount; ++i) {
    const ui::altitude::Rgb below = colorForAltFt(kRamp[i].min_ft - 1);
    CHECK(below.r == kRamp[i - 1].rgb.r);
    CHECK(below.g == kRamp[i - 1].rgb.g);
    CHECK(below.b == kRamp[i - 1].rgb.b);
  }

  // Above the last step the top colour holds (airliners cruise at 41k).
  CHECK(colorForAltFt(60000).r == kRamp[kRampCount - 1].rgb.r);
  CHECK(colorForAltFt(60000).b == kRamp[kRampCount - 1].rgb.b);

  // A step that quantises onto its neighbour is dead weight: it means the table
  // was hand-edited or the generator lost a pixel.
  for (int i = 0; i < kRampCount; ++i) {
    CHECK(!samePixel(quantise(kRamp[i].rgb), quantise(kNeutral)));
    if (i > 0) {
      CHECK(!samePixel(quantise(kRamp[i].rgb), quantise(kRamp[i - 1].rgb)));
    }
  }

  // The ramp must never walk back to a pixel it already used. That is the whole
  // point of the monotone-lightness envelope: without it the ramp dips into a
  // cooler orange at ~1.8k ft and returns at ~2.4k, so the icon steps forward
  // and then back while the aircraft climbs.
  for (int i = 1; i < kRampCount; ++i) {
    for (int j = 0; j < i; ++j) {
      CHECK(!samePixel(quantise(kRamp[i].rgb), quantise(kRamp[j].rgb)));
    }
  }

  // Spot checks against the legend: bottom red-orange, mid green, top violet.
  CHECK(colorForAltFt(0).r > 200 && colorForAltFt(0).g < 100);
  CHECK(colorForAltFt(12000).g > 180 && colorForAltFt(12000).r < 80);
  CHECK(colorForAltFt(40000).b > 150 && colorForAltFt(40000).g < 60);

  return testSummary("test_altitude_color");
}
