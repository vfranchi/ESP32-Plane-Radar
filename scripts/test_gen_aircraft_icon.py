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


def test_all_sets_reach_the_header():
    header = gen.render_header(gen.SETS)
    assert "namespace data::aircraft_icon {" in header
    assert "namespace data::aircraft_icon_small {" in header
    assert "namespace data::helicopter_icon {" in header
    assert f"constexpr int kSize = {gen.ICON_PX_SMALL};" in header


def test_helicopter_set_is_sized_like_the_small_aircraft():
    """Rotorcraft ride at the private/light box size, not the airliner one."""
    size = gen.ICON_PX_SMALL
    blob = gen.build(size, gen.HELICOPTER_SHAPE)
    assert len(blob) == gen.ROTATIONS * gen.bytes_per_rot(size)
    # The firmware static_asserts this against radar::kHelicopterIconSizePx.
    assert gen.ICON_PX_SMALL == 13


def test_helicopter_has_ink_in_every_rotation():
    size = gen.ICON_PX_SMALL
    base = gen.render_base(size, gen.HELICOPTER_SHAPE)
    for i in range(gen.ROTATIONS):
        alpha = gen.rotation_alpha(base, i * 360.0 / gen.ROTATIONS, size)
        assert max(alpha) > 0, f"helicopter rotation {i} is blank"
        assert all(0 <= v <= 15 for v in alpha)
        assert len(alpha) == size * size


def test_helicopter_nose_points_up_at_rotation_zero():
    """Checked on the centre column, not the top row: the rotor blades reach
    higher than the nose, so the topmost ink of the icon is not the nose."""
    size = gen.ICON_PX_SMALL
    alpha = gen.rotation_alpha(gen.render_base(size, gen.HELICOPTER_SHAPE), 0.0, size)
    centre = size // 2
    col = [r for r in range(size) if alpha[r * size + centre] > 0]
    assert col, "centre column is blank - nose is not pointing up"
    assert min(col) <= 3, f"nose is not near the top (first ink at row {min(col)})"


def test_helicopter_blades_are_faded_but_present():
    """Full-opacity blades make the icon an asterisk, invisible ones a bare fuselage.

    The rotor must therefore land strictly between the two: some full-coverage
    pixels (the body and the tail rotor) and a solid patch of faded ones.
    """
    size = gen.ICON_PX_SMALL
    alpha = gen.rotation_alpha(gen.render_base(size, gen.HELICOPTER_SHAPE), 0.0, size)
    assert max(alpha) == 15, "no full-coverage pixels - body missing"
    faded = [v for v in alpha if 0 < v < 15]
    # 75 of the 169 pixels are blades at weight 0.60; the assert is a floor, not the
    # measurement, so a re-tune of the blade geometry does not have to match it.
    assert len(faded) >= 30, f"rotor blades lost in the downscale ({len(faded)} px)"
    assert 8 <= max(faded) <= 14, f"blade coverage off the fade range: {max(faded)}"


def test_helicopter_differs_from_the_plane_blob():
    size = gen.ICON_PX_SMALL
    plane_blob = gen.build(size, gen.PLANE_SHAPE)
    heli_blob = gen.build(size, gen.HELICOPTER_SHAPE)
    assert plane_blob != heli_blob
    assert len(plane_blob) == len(heli_blob)
