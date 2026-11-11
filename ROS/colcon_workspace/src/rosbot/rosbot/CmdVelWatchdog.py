"""Time-based helpers for the motor command watchdog."""


def command_timed_out(last_command_time, now, timeout):
    """Return whether a command timestamp exceeded its timeout."""
    return (
        last_command_time is not None
        and now - last_command_time >= timeout
    )
