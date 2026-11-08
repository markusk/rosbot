"""Tests for pigpio-backed motor channel behavior."""

from rosbot.MotorControllerNode import MotorChannel


class FakePigpio:
    OUTPUT = 1

    def __init__(self):
        self.calls = []

    def set_mode(self, pin, mode):
        self.calls.append(('set_mode', pin, mode))

    def write(self, pin, value):
        self.calls.append(('write', pin, value))

    def set_PWM_range(self, pin, value):
        self.calls.append(('set_PWM_range', pin, value))

    def set_PWM_frequency(self, pin, value):
        self.calls.append(('set_PWM_frequency', pin, value))

    def set_PWM_dutycycle(self, pin, value):
        self.calls.append(('set_PWM_dutycycle', pin, value))


def test_motor_channel_applies_reverse_direction_and_pwm():
    """Stop PWM before setting reverse direction and applying duty cycle."""
    pi = FakePigpio()
    motor = MotorChannel(pi, in_a=5, in_b=6, enable=12, output_mode=pi.OUTPUT)
    pi.calls.clear()

    motor.set_speed(-0.5)

    assert pi.calls == [
        ('set_PWM_dutycycle', 12, 0),
        ('write', 5, 0),
        ('write', 6, 1),
        ('set_PWM_dutycycle', 12, 128),
    ]


def test_motor_channel_stops_inside_deadband():
    """Disable PWM and both direction inputs for near-zero speed."""
    pi = FakePigpio()
    motor = MotorChannel(pi, in_a=5, in_b=6, enable=12, output_mode=pi.OUTPUT)
    pi.calls.clear()

    motor.set_speed(0.01)

    assert pi.calls == [
        ('set_PWM_dutycycle', 12, 0),
        ('write', 5, 0),
        ('write', 6, 0),
    ]
