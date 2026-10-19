#!/usr/bin/env python3
"""Generate the altitude -> colour ramp table in include/ui/altitude_color.h.

The frame sprite is RGB332 and the panel is BGR (kDisplayRgbOrder), so the sprite
stores 3 bits of BLUE / 3 of green / only 2 of red. A continuous HSL ramp
therefore collapses to a finite set of screen colours: sampling every 10 ft and
keeping the first altitude where the STORED pixel changes gives the maximum
resolution the panel can show, with no colour maths at runtime. Lightness is
clamped to a monotone envelope so the ramp never steps backwards.

usage:
  python3 scripts/gen_altitude_ramp.py          # print the generated block
  python3 scripts/gen_altitude_ramp.py --check  # exit 1 if the header is stale
"""
import colorsys
import pathlib
import sys

# Altitude legend stops: (altitude ft, hue deg, saturation %, lightness %).
# 0 ft is redder than the legend's hsl(23) on purpose: hsl(23) and hsl(26) are
# the same RGB332 pixel, and the bottom band must read red/orange.
STOPS = [
    (0, 12.0, 95.0, 50.0),
    (1000, 26.25, 88.0, 52.08),
    (2000, 32.5, 88.0, 53.875),
    (4000, 43.0, 88.0, 51.5),
    (6000, 54.0, 88.0, 44.8),
    (8000, 72.0, 88.0, 41.8),
    (10000, 112.5, 88.0, 41.0),
    (20000, 189.66, 88.0, 43.86),
    (30000, 244.83, 88.0, 57.03),
    (40000, 300.0, 88.0, 43.0),
]
SAMPLE_STEP_FT = 10
MAX_ALT_FT = 40000

HEADER = pathlib.Path(__file__).resolve().parent.parent / "include/ui/altitude_color.h"
BEGIN = "// <generated: ramp>"
END = "// </generated: ramp>"


def interpolate(alt_ft):
    """Hue/sat/light at this altitude, linear between the legend stops."""
    if alt_ft <= STOPS[0][0]:
        return STOPS[0][1:]
    if alt_ft >= STOPS[-1][0]:
        return STOPS[-1][1:]
    for (a0, h0, s0, l0), (a1, h1, s1, l1) in zip(STOPS, STOPS[1:]):
        if a0 <= alt_ft <= a1:
            t = (alt_ft - a0) / (a1 - a0)
            return (h0 + t * (h1 - h0), s0 + t * (s1 - s0), l0 + t * (l1 - l0))
    raise AssertionError("unreachable")


def rgb8(alt_ft):
    h, s, _l = interpolate(alt_ft)
    r, g, b = colorsys.hls_to_rgb(h / 360.0, lightness(alt_ft) / 100.0, s / 100.0)
    return (round(r * 255), round(g * 255), round(b * 255))


def lightness(alt_ft):
    """Monotone (non-decreasing) lightness: the running maximum of the stop curve.

    The legend dips 53.875 % (2k ft) -> 51.5 % (4k ft). That dip drags the blue
    channel across a 3-bit boundary twice, giving a step that goes forward and
    then back (~1.8k-2.4k ft). Clamping to the running maximum removes the
    reversal; the accepted cost is brighter mid/top bands than the legend.
    """
    floor = max(l for a, _h, _s, l in STOPS if a <= alt_ft)
    return max(floor, interpolate(alt_ft)[2])


def sprite_pixel(rgb):
    """Byte the rgb332 frame sprite stores for this logical colour.

    panelColor565() swaps red and blue (kDisplayRgbOrder), so the sprite's 3-bit
    field carries the logical BLUE channel and its 2-bit field the logical red --
    the opposite of the usual RGB332 layout. Quantising as
    (r>>5, g>>5, b>>6) would validate the ramp against pixels the panel never
    shows, and that is exactly how a backward step around 2k ft got missed.
    """
    r, g, b = rgb
    return (b >> 5, g >> 5, r >> 6)


def build_rows():
    rows = []
    previous = None
    for alt in range(0, MAX_ALT_FT + SAMPLE_STEP_FT, SAMPLE_STEP_FT):
        rgb = rgb8(alt)
        px = sprite_pixel(rgb)
        if px == previous:
            continue
        rows.append((alt, rgb, px))
        previous = px
    return rows


def render(rows):
    lines = [BEGIN, "constexpr Step kRamp[kRampCount] = {"]
    for alt, (r, g, b), px in rows:
        lines.append("    {%5d, {%3d, %3d, %3d}},  // sprite pixel B%d G%d R%d"
                     % (alt, r, g, b, px[0], px[1], px[2]))
    lines += ["};", END]
    return "\n".join(lines)


def main():
    block = render(build_rows())
    if "--check" not in sys.argv:
        print(block)
        return 0
    text = HEADER.read_text()
    start = text.find(BEGIN)
    end = text.find(END)
    if start < 0 or end < 0:
        print("header has no generated-block markers", file=sys.stderr)
        return 1
    if text[start:end + len(END)] == block:
        print("up to date")
        return 0
    print("stale: regenerate the block between the markers", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
