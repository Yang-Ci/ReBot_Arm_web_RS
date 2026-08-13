"""通用工具函数测试。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rebot_b601_rs_sim.utils import limit_array, parse_gravity_scale


def test_limit_array_scalar() -> None:
    arr = limit_array(np.array(5.0), 3.0)
    assert arr.shape == (1,)
    assert arr[0] == pytest.approx(3.0)


def test_limit_array_1d() -> None:
    arr = limit_array(np.array([1.0, -8.0, 4.0]), 5.0)
    np.testing.assert_allclose(arr, [1.0, -5.0, 4.0])


def test_limit_array_per_dim() -> None:
    arr = limit_array(np.array([1.0, -8.0, 4.0]), np.array([0.5, 10.0, 3.0]))
    np.testing.assert_allclose(arr, [0.5, -8.0, 3.0])


def test_parse_gravity_scale_single() -> None:
    scale = parse_gravity_scale("1.2")
    np.testing.assert_allclose(scale, [1.2, 1.2, 1.2, 1.2, 1.2, 1.2])


def test_parse_gravity_scale_six() -> None:
    scale = parse_gravity_scale("1.0,1.2,1.0,1.0,1.0,1.0")
    np.testing.assert_allclose(scale, [1.0, 1.2, 1.0, 1.0, 1.0, 1.0])


def test_parse_gravity_scale_invalid() -> None:
    with pytest.raises(Exception):
        parse_gravity_scale("1.0,2.0")
