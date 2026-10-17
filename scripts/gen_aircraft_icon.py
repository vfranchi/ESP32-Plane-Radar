#!/usr/bin/env python3
"""Generate pre-rotated aircraft icon bitmaps for the radar UI.

Run: /opt/hermes/.venv/bin/python3 scripts/gen_aircraft_icon.py

Emits the fixed-wing silhouette twice at different sizes -- `aircraft_icon` for
airliners and `aircraft_icon_small` for private/light traffic, so a light airframe
is separable by size as well as by colour -- plus `helicopter_icon`, the rotorcraft
shape (emitter category A7), at the airliner size.

A shape is a list of (polygons, weight) layers. A weight below 1 rasterises the
layer onto a lower coverage nibble, which the firmware's pre-blended palette draws
as a faded component -- that is how the helicopter's rotor blades read as spinning
without any runtime alpha work.

Writes:
  include/ui/aircraft_icon_data.h            generated, committed
  src/ui/aircraft_icon_data.cpp              generated, committed
  .artifacts/<namespace>_preview.png         visual check only, gitignored
"""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw

ICON_PX = 19          # airliner icon: square bounding box, pixels
# Private/light icon. Must stay equal to radar::kPrivateAircraftIconSizePx; the
# firmware static_asserts the two against each other, so a mismatch fails the build.
ICON_PX_SMALL = 13
ROTATIONS = 72        # one every 5 degrees
SUPERSAMPLE = 8       # rasterise at ICON_PX * SUPERSAMPLE, then downscale

# The 19px set, kept as constants for the tests: 4bpp, two pixels per byte.
BYTES_PER_ROW = (ICON_PX + 1) // 2
BYTES_PER_ROT = BYTES_PER_ROW * ICON_PX

ROOT = Path(__file__).resolve().parents[1]
OUT_H = ROOT / "include" / "ui" / "aircraft_icon_data.h"
OUT_CPP = ROOT / "src" / "ui" / "aircraft_icon_data.cpp"
ARTIFACTS = ROOT / ".artifacts"

# --- rotorcraft (emitter category A7) -------------------------------------------
# Top-down helicopter, nose at +Y, same unit box as the plane. The rotor is drawn
# as its own layer at a lower weight: at 19 px a full-opacity four-blade cross
# collapses into an asterisk and stops reading as an aircraft, while faded blades
# keep the disc legible and still say "rotor" instead of "swept wing".


def blade(angle_deg: float, root: float, tip: float,
          w_root: float, w_tip: float) -> list[tuple[float, float]]:
    """One rotor blade along `angle_deg` (degrees, 0 = nose/up) as a quadrilateral."""
    a = math.radians(angle_deg)
    ax, ay = math.sin(a), math.cos(a)      # along the blade
    px, py = math.cos(a), -math.sin(a)     # across it

    def pt(along: float, off: float) -> tuple[float, float]:
        return (ax * along + px * off, ay * along + py * off)

    return [pt(root, -w_root / 2), pt(tip, -w_tip / 2),
            pt(tip, w_tip / 2), pt(root, w_root / 2)]


def rotor(angles, root: float, tip: float, w_root: float, w_tip: float,
          dy: float = 0.0) -> list[list[tuple[float, float]]]:
    """A rotor: one blade per angle, shifted by `dy` along the fuselage axis."""
    return [[(x, y + dy) for x, y in blade(a, root, tip, w_root, w_tip)]
            for a in angles]


# Geometry is tuned for the SMALL icon size (the rotorcraft shares the private
# aircraft's box): at 13 px the 19 px outline broke into a spray of single pixels,
# the tail boom vanished and the blades merged with the body. Chunky features and
# a wider boom are what survive the downscale, same trade the plane icon makes.
HELI_FUSELAGE = [(0.00, 0.85), (0.16, 0.55), (0.26, 0.10), (0.18, -0.30),
                 (0.11, -0.60), (0.10, -0.80), (0.00, -0.88), (-0.10, -0.80),
                 (-0.11, -0.60), (-0.18, -0.30), (-0.26, 0.10), (-0.16, 0.55)]
# Tail rotor: a short bar across the end of the boom (angle 0 points at the nose,
# so the across-the-body pair is 90/270).
HELI_TAIL_ROTOR = rotor([90, 270], 0.0, 0.22, 0.16, 0.10, dy=-0.78)
# Main rotor: four fat blades, none of them lying along the fuselage axis, so the
# blades never merge with the body or read as a spike off the nose.
HELI_MAIN_ROTOR = rotor([42, 132, 222, 312], 0.28, 0.98, 0.40, 0.22, dy=0.12)

# Top-down silhouette, nose at +Y. Unit box: 1.0 = half the icon edge.
# Geometry approved via the preview renders (2026-09-28). No FIN polygon:
# a top-down fin is seen edge-on and disappears inside the fuselage.
FUSELAGE = [(0.00, 1.00), (0.12, 0.15), (0.10, -0.60),
            (0.00, -1.00), (-0.10, -0.60), (-0.12, 0.15)]
# Wing, 5 points: long root chord at the body, tapering to a thin tip.
# root LE -> leading-edge kink -> tip LE -> tip TE -> root TE
WING_R = [(0.10, 0.30), (0.58, -0.02), (1.00, -0.42), (0.98, -0.54), (0.10, -0.08)]
WING_L = [(-x, y) for x, y in WING_R]
TAIL_R = [(0.08, -0.58), (0.50, -0.86), (0.47, -0.96), (0.08, -0.80)]
TAIL_L = [(-x, y) for x, y in TAIL_R]

POLYGONS = [FUSELAGE, WING_R, WING_L, TAIL_R, TAIL_L]

# TUNING THE WING: only WING_R's 2nd and 3rd numbers change the root thickness.
#   leve  (light)   [(0.10, 0.24), (0.60,  0.02), (1.00, -0.42), (0.98, -0.54), (0.10, -0.04)]
#   media (current) [(0.10, 0.30), (0.58, -0.02), (1.00, -0.42), (0.98, -0.54), (0.10, -0.08)]
#   forte (strong)  [(0.10, 0.36), (0.55, -0.04), (1.00, -0.42), (0.98, -0.54), (0.10, -0.12)]
# Changing the wing changes NO constant -- no ICON_PX, no byte counts, nothing in
# the firmware. It only changes generated data. Rerun Tasks 2-4 after editing.

PLANE_SHAPE = [(POLYGONS, 1.0)]
# Full opacity: the owner wants the rotor as solid as the body, and the 13 px
# geometry (fat blades, chunkier fuselage) is what keeps it from reading as an
# asterisk. Lower this to fade the rotor without touching any other constant.
HELICOPTER_BLADE_WEIGHT = 1.0
HELICOPTER_SHAPE = [([HELI_FUSELAGE] + HELI_TAIL_ROTOR, 1.0),
                    (HELI_MAIN_ROTOR, HELICOPTER_BLADE_WEIGHT)]

# (namespace, size, preview colour, shape) -- the colours mirror the firmware theme.
# The helicopter shares the airliner colour on purpose: it is told apart by its
# rotor, and one palette serves both sets (see radar_display.cpp).
SETS = [("aircraft_icon", ICON_PX, (250, 204, 60), PLANE_SHAPE),
        ("aircraft_icon_small", ICON_PX_SMALL, (0, 255, 80), PLANE_SHAPE),
        ("helicopter_icon", ICON_PX_SMALL, (250, 204, 60), HELICOPTER_SHAPE)]
BG_RGB = (4, 10, 28)

NAMESPACE_TMPL = """\
namespace data::{ns} {{

/** Square icon bounding box, pixels. */
constexpr int kSize = {size};
/** Pre-rendered rotations; index i is i * (360 / kRotations) degrees. */
constexpr int kRotations = {rotations};
/** Bytes per rotation: 4bpp, {bytes_per_row} bytes per {size}px row, high nibble first. */
constexpr int kBytesPerRotation = {bytes_per_rot};
/** Palette index treated as transparent by pushImage. */
constexpr int kTransparentIndex = 0;

/** kRotations * kBytesPerRotation bytes, rotation-major. */
extern const uint8_t kIcon[];

}}  // namespace data::{ns}
"""

HEADER_TMPL = """\
// Generated by scripts/gen_aircraft_icon.py -- do not edit.
#pragma once

#include <cstdint>

{sections}"""


def bytes_per_row(size_px: int = ICON_PX) -> int:
    return (size_px + 1) // 2


def bytes_per_rot(size_px: int = ICON_PX) -> int:
    return bytes_per_row(size_px) * size_px


def render_base(size_px: int = ICON_PX, shape=None) -> Image.Image:
    """Rasterise one icon shape once, nose up, at SUPERSAMPLE resolution.

    `shape` is a list of (polygons, weight) layers; the default is the fixed-wing
    silhouette. A weight below 1 scales that layer's coverage, which survives the
    downscale as a lower nibble and renders as a faded part of the icon.
    """
    shape = PLANE_SHAPE if shape is None else shape
    size = size_px * SUPERSAMPLE
    img = Image.new("L", (size, size), 0)
    for polys, weight in shape:
        layer = Image.new("L", (size, size), 0)
        draw = ImageDraw.Draw(layer)
        for poly in polys:
            pts = [((x + 1.0) * 0.5 * (size - 1),
                    (1.0 - y) * 0.5 * (size - 1)) for x, y in poly]
            draw.polygon(pts, fill=255)
        if weight < 1.0:
            layer = layer.point(lambda v: int(v * weight))
        img = Image.composite(layer, img, layer)
    return img


def rotation_alpha(base: Image.Image, heading_deg: float,
                   size_px: int = ICON_PX) -> list[int]:
    """Coverage (0..15) per pixel for one heading; heading grows clockwise, 0 = north.

    `base` must come from render_base(size_px): the rotation and the downscale both
    key off size_px, since the supersampled image is scaled down to that box.
    """
    # PIL rotates counter-clockwise for positive angles, hence the negation.
    rot = base.rotate(-heading_deg, resample=Image.BICUBIC, fillcolor=0)
    small = rot.resize((size_px, size_px), Image.LANCZOS)
    return [min(15, (v * 15 + 127) // 255) for v in small.tobytes()]


def pack4bpp(alpha: list[int], size_px: int = ICON_PX) -> bytes:
    out = bytearray()
    for row in range(size_px):
        start = row * size_px
        for col in range(0, size_px, 2):
            hi = alpha[start + col]
            lo = alpha[start + col + 1] if col + 1 < size_px else 0
            out.append((hi << 4) | lo)
    return bytes(out)


def build(size_px: int = ICON_PX, shape=None) -> bytes:
    base = render_base(size_px, shape)
    blob = bytearray()
    for i in range(ROTATIONS):
        blob += pack4bpp(
            rotation_alpha(base, i * 360.0 / ROTATIONS, size_px), size_px)
    return bytes(blob)


def render_header(sets) -> str:
    sections = "\n".join(
        NAMESPACE_TMPL.format(ns=ns, size=size, rotations=ROTATIONS,
                              bytes_per_row=bytes_per_row(size),
                              bytes_per_rot=bytes_per_rot(size))
        for ns, size, _, _ in sets)
    return HEADER_TMPL.format(sections=sections)


def render_cpp(sets) -> str:
    lines = [
        "// Generated by scripts/gen_aircraft_icon.py -- do not edit.",
        '#include "ui/aircraft_icon_data.h"',
        "",
    ]
    for ns, size, _, shape in sets:
        blob = build(size, shape)
        lines += [
            f"namespace data::{ns} {{",
            "",
            "const uint8_t kIcon[] = {",
        ]
        for i in range(0, len(blob), 16):
            chunk = blob[i:i + 16]
            lines.append("    " + " ".join(f"0x{b:02X}," for b in chunk))
        lines += [
            "};",
            "",
            "static_assert(sizeof(kIcon) == kRotations * kBytesPerRotation,",
            f'              "aircraft icon data size mismatch ({ns})");',
            "",
            f"}}  // namespace data::{ns}",
            "",
        ]
    return "\n".join(lines)


def write_preview(base: Image.Image, out_path: Path, size_px: int,
                  colour: tuple[int, int, int]) -> None:
    """Tile every rotation into one PNG so the shape can be eyeballed before flashing."""
    cell = size_px + 2
    cols = 12
    rows = (ROTATIONS + cols - 1) // cols
    canvas = Image.new("RGB", (cols * cell, rows * cell), BG_RGB)
    for i in range(ROTATIONS):
        small = base.rotate(-i * 360.0 / ROTATIONS,
                            resample=Image.BICUBIC, fillcolor=0)
        small = small.resize((size_px, size_px), Image.LANCZOS)
        px = small.load()
        tile = Image.new("RGB", (size_px, size_px), BG_RGB)
        tpx = tile.load()
        for y in range(size_px):
            for x in range(size_px):
                a = px[x, y] / 255.0
                tpx[x, y] = tuple(int(colour[c] * a + BG_RGB[c] * (1 - a))
                                  for c in range(3))
        canvas.paste(tile, ((i % cols) * cell + 1, (i // cols) * cell + 1))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)


def main() -> int:
    OUT_H.parent.mkdir(parents=True, exist_ok=True)
    OUT_CPP.parent.mkdir(parents=True, exist_ok=True)
    OUT_H.write_text(render_header(SETS), encoding="utf-8")
    OUT_CPP.write_text(render_cpp(SETS), encoding="utf-8")

    total = 0
    for ns, size, colour, shape in SETS:
        blob_len = ROTATIONS * bytes_per_rot(size)
        total += blob_len
        preview = ARTIFACTS / f"{ns}_preview.png"
        write_preview(render_base(size, shape), preview, size, colour)
        print(f"  data::{ns}: {size}x{size} px, {blob_len} bytes "
              f"({ROTATIONS} rotations)")
    print(f"wrote {OUT_H.relative_to(ROOT)} + {OUT_CPP.relative_to(ROOT)} "
          f"({total} bytes total)")
    print(f"previews: {ARTIFACTS.relative_to(ROOT)}/<namespace>_preview.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
