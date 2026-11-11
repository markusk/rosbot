"""Tests for cmd_vel watchdog timeout behavior."""

from rosbot.CmdVelWatchdog import command_timed_out


def test_no_command_is_not_timed_out_before_first_message():
    """Keep startup idle distinct from an expired command."""
    assert not command_timed_out(None, now=10.0, timeout=0.5)


def test_command_is_not_timed_out_before_deadline():
    """Allow commands until the configured timeout elapses."""
    assert not command_timed_out(10.0, now=10.49, timeout=0.5)


def test_command_times_out_at_deadline():
    """Expire a command once its age reaches the configured timeout."""
    assert command_timed_out(10.0, now=10.5, timeout=0.5)


def test_recent_command_refreshes_timeout():
    """Use the latest command timestamp for timeout calculations."""
    assert not command_timed_out(10.2, now=10.6, timeout=0.5)
