"""Tests for gpiozero-backed motor channel behavior."""

from rosbot.MotorControllerNode import MotorChannel


class FakeOutputDevice:
    def __init__(self, pin, initial_value):
        self.pin = pin
        self.value = initial_value
        self.calls = []

    def on(self):
        self.value = True
        self.calls.append(('on',))

    def off(self):
        self.value = False
        self.calls.append(('off',))

    def close(self):
        self.calls.append(('close',))


class FakePWMOutputDevice(FakeOutputDevice):
    def __init__(self, pin, frequency, initial_value):
        super().__init__(pin, initial_value)
        self.frequency = frequency


class FakeDeviceFactory:
    def __init__(self, device_type):
        self.device_type = device_type
        self.devices = []

    def __call__(self, *args, **kwargs):
        device = self.device_type(*args, **kwargs)
        self.devices.append(device)
        return device


def create_motor():
    output_factory = FakeDeviceFactory(FakeOutputDevice)
    pwm_factory = FakeDeviceFactory(FakePWMOutputDevice)
    motor = MotorChannel(
        in_a=5,
        in_b=6,
        enable=12,
        output_device_factory=output_factory,
        pwm_output_device_factory=pwm_factory,
    )
    return motor, output_factory, pwm_factory


def test_motor_channel_applies_reverse_direction_and_pwm():
    """Stop PWM before setting reverse direction and applying duty cycle."""
    motor, output_factory, pwm_factory = create_motor()

    motor.set_speed(-0.5)

    assert output_factory.devices[0].value is False
    assert output_factory.devices[1].value is True
    assert pwm_factory.devices[0].value == 0.5


def test_motor_channel_stops_inside_deadband():
    """Disable PWM and both direction inputs for near-zero speed."""
    motor, output_factory, pwm_factory = create_motor()

    motor.set_speed(0.01)

    assert pwm_factory.devices[0].value == 0
    assert output_factory.devices[0].value is False
    assert output_factory.devices[1].value is False
