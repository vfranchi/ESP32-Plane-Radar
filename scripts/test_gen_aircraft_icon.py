"""Tests for gen_aircraft_icon.

Run: /opt/hermes/.venv/bin/python3 -m pytest scripts/test_gen_aircraft_icon.py -q
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gen_aircraft_icon as gen


def test_blob_length_matches_metadata():
    blob = gen.build()
    assert gen.BYTES_PER_ROT == 190  # 19 px, 4bpp, 2 px/byte, 10 bytes/row
    assert len(blob) == gen.ROTATIONS * gen.BYTES_PER_ROT


def test_rotation_zero_nose_points_up():
    alpha = gen.rotation_alpha(gen.render_base(), 0.0)
    centre = gen.ICON_PX // 2
    top_row = [c for c in range(gen.ICON_PX) if alpha[c] > 0]
    assert top_row, "top row is blank - nose is not pointing up"
    assert min(abs(c - centre) for c in top_row) <= 2


def test_quarter_turn_nose_points_right():
    alpha = gen.rotation_alpha(gen.render_base(), 90.0)
    centre = gen.ICON_PX // 2
    row = [c for c in range(gen.ICON_PX)
           if alpha[centre * gen.ICON_PX + c] > 0]
    assert row and max(row) >= gen.ICON_PX - 3


def test_every_rotation_has_ink():
    base = gen.render_base()
    for i in range(gen.ROTATIONS):
        alpha = gen.rotation_alpha(base, i * 360.0 / gen.ROTATIONS)
        assert max(alpha) > 0, f"rotation {i} is blank"
        assert len(alpha) == gen.ICON_PX * gen.ICON_PX


def test_alpha_is_clamped_to_four_bits():
    base = gen.render_base()
    for i in range(gen.ROTATIONS):
        alpha = gen.rotation_alpha(base, i * 360.0 / gen.ROTATIONS)
        assert all(0 <= v <= 15 for v in alpha), f"rotation {i} out of range"


def test_packing_is_high_nibble_first():
    alpha = [0] * (gen.ICON_PX * gen.ICON_PX)
    alpha[0] = 1
    alpha[1] = 2
    packed = gen.pack4bpp(alpha)
    assert packed[0] == 0x12


def test_output_is_deterministic():
    assert gen.build() == gen.build()


def test_small_set_is_sized_and_packed():
    size = gen.ICON_PX_SMALL
    assert gen.bytes_per_row(size) == (size + 1) // 2   # 4bpp, 2 px/byte
    assert gen.bytes_per_rot(size) == gen.bytes_per_row(size) * size
    blob = gen.build(size)
    assert len(blob) == gen.ROTATIONS * gen.bytes_per_rot(size)
    # The firmware asserts this against radar::kPrivateAircraftIconSizePx at build
    # time, so the size itself is left free to move here.


def test_small_set_has_ink_in_every_rotation():
    base = gen.render_base(gen.ICON_PX_SMALL)
    for i in range(gen.ROTATIONS):
        alpha = gen.rotation_alpha(base, i * 360.0 / gen.ROTATIONS, gen.ICON_PX_SMALL)
        assert max(alpha) > 0, f"small rotation {i} is blank"
        assert len(alpha) == gen.ICON_PX_SMALL * gen.ICON_PX_SMALL


def test_both_sets_reach_the_header():
    header = gen.render_header(gen.SETS)
    assert "namespace data::aircraft_icon {" in header
    assert "namespace data::aircraft_icon_small {" in header
    assert f"constexpr int kSize = {gen.ICON_PX_SMALL};" in header
