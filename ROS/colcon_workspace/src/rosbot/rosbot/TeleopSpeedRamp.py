"""Helpers for scaling keyboard teleoperation commands over time."""


def key_repeat_timed_out(last_input_time, now, timeout):
    """Return whether terminal key repeats have stopped arriving."""
    return (
        last_input_time is not None
        and now - last_input_time >= timeout
    )


def speed_scale(start_scale, end_scale, duration, elapsed):
    """Interpolate from the start scale to the end scale over a duration."""
    if duration <= 0.0:
        return end_scale

    progress = max(0.0, min(1.0, elapsed / duration))
    return start_scale + (end_scale - start_scale) * progress
