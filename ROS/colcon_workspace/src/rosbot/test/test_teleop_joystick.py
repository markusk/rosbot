"""Tests for joystick message mapping."""

from rosbot.TeleopArbitratorNode import TeleopArbitratorNode
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


def test_arbitrator_ignores_neutral_joystick_when_keyboard_is_active():
    """Keep keyboard control when an inactive joystick reports neutral."""
    source, should_stop = TeleopArbitratorNode._resolve_source(
        'keyboard', 'joystick', False)
    assert source == 'keyboard'
    assert not should_stop


def test_arbitrator_switches_to_active_joystick_without_combining():
    """Give control to an active joystick without combining commands."""
    source, should_stop = TeleopArbitratorNode._resolve_source(
        'keyboard', 'joystick', True)
    assert source == 'joystick'
    assert not should_stop


def test_arbitrator_keyboard_stop_stops_all_control():
    """Clear the active source when keyboard sends a stop command."""
    source, should_stop = TeleopArbitratorNode._resolve_source(
        'joystick', 'keyboard', False)
    assert source is None
    assert should_stop


def test_arbitrator_joystick_stop_stops_all_control():
    """Clear the active source when joystick sends an explicit stop."""
    source, should_stop = TeleopArbitratorNode._resolve_source(
        'keyboard', 'joystick', False, explicit_stop=True)
    assert source is None
    assert should_stop
