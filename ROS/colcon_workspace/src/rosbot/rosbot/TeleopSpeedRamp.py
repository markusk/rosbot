"""Helpers for scaling keyboard teleoperation commands over time."""


def speed_scale(start_scale, end_scale, duration, elapsed):
    """Interpolate from the start scale to the end scale over a duration."""
    if duration <= 0.0:
        return end_scale

    progress = max(0.0, min(1.0, elapsed / duration))
    return start_scale + (end_scale - start_scale) * progress
