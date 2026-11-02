"""Tests for joystick message mapping."""

from rosbot.TeleopJoystickNode import TeleopJoystickNode


def test_axis_value_supports_legacy_controller_layouts():
    """Read a legacy forward axis when the preferred index is absent."""
    assert TeleopJoystickNode._axis_value([0.0, 0.0, 1.0], (5, 2)) == 1.0


def test_axis_value_ignores_missing_and_deadzone_values():
    """Treat missing axes and values within the deadzone as neutral."""
    assert TeleopJoystickNode._axis_value([0.4], (0, 3)) == 0.0


def test_button_pressed_supports_alternate_stop_button():
    """Recognize either configured stop button."""
    assert TeleopJoystickNode._button_pressed([0, 1], (10, 1))


def test_button_pressed_handles_short_messages():
    """Treat missing buttons as unpressed."""
    assert not TeleopJoystickNode._button_pressed([0], (1, 10))
