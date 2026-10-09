"""Deterministic checks for the split IMU and barometer sensor models."""

from __future__ import annotations

from math import isclose
from random import Random

from bridge.sensors import BarometerConfig, BarometerSensor, ImuConfig, ImuSensor


def main() -> None:
    """Verify zero-noise, bias, and seeded Gaussian sensor behavior."""
    imu = ImuSensor()
    rates, force = imu.read((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0 / 240.0)
    assert rates == (0.0, 0.0, 0.0)
    assert force == (0.0, 0.0, 9.81)

    biased_imu = ImuSensor(
        config=ImuConfig(
            gyro_bias_rad_s=(0.1, 0.0, 0.0),
            accelerometer_bias_mps2=(0.0, -0.2, 0.0),
        )
    )
    rates, force = biased_imu.read((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0 / 240.0)
    assert rates == (0.1, 0.0, 0.0)
    assert force == (0.0, -0.2, 9.81)

    config = ImuConfig(gyro_noise_sigma_rad_s=(0.01, 0.01, 0.01), accelerometer_noise_sigma_mps2=(0.1, 0.1, 0.1))
    first = ImuSensor(config=config, rng=Random(7)).read((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0 / 240.0)
    second = ImuSensor(config=config, rng=Random(7)).read((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0 / 240.0)
    assert first == second

    barometer = BarometerSensor()
    assert isclose(barometer.read(0.0), 101_325.0)
    biased_barometer = BarometerSensor(BarometerConfig(pressure_bias_pa=12.0))
    assert isclose(biased_barometer.read(0.0), 101_337.0)
    first_pressure = BarometerSensor(BarometerConfig(pressure_noise_sigma_pa=2.0), Random(7)).read(3.0)
    second_pressure = BarometerSensor(BarometerConfig(pressure_noise_sigma_pa=2.0), Random(7)).read(3.0)
    assert first_pressure == second_pressure
    print("Sensor self-check passed")


if __name__ == "__main__":
    main()
