"""Tests for keyboard teleoperation speed scaling."""

import pytest

from rosbot.TeleopSpeedRamp import key_repeat_timed_out, speed_scale


def test_terminal_repeat_timeout_expires_after_configured_interval():
    """Detect when repeated terminal key events have stopped."""
    assert not key_repeat_timed_out(10.0, 10.29, 0.3)
    assert key_repeat_timed_out(10.0, 10.3, 0.3)
    assert not key_repeat_timed_out(None, 10.3, 0.3)


def test_speed_scale_starts_at_configured_value():
    """Begin a movement at the configured start scale."""
    assert speed_scale(0.2, 1.0, 2.0, 0.0) == 0.2


def test_speed_scale_interpolates_linearly():
    """Interpolate between start and end scales during acceleration."""
    assert speed_scale(0.2, 1.0, 2.0, 1.0) == pytest.approx(0.6)


def test_speed_scale_reaches_end_value():
    """Reach and hold the configured end scale after the ramp."""
    assert speed_scale(0.2, 1.0, 2.0, 2.0) == pytest.approx(1.0)
    assert speed_scale(0.2, 1.0, 2.0, 5.0) == pytest.approx(1.0)


def test_non_positive_duration_uses_end_value():
    """Avoid division by zero for an invalid duration."""
    assert speed_scale(0.2, 1.0, 0.0, 0.0) == 1.0
