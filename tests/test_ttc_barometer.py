"""Small regression checks for the TTC BMP388-inspired barometer."""

from dataclasses import replace
from pathlib import Path
import sys
import tempfile
import unittest


EXAMPLES_ROOT = Path(__file__).resolve().parents[1] / "examples"
sys.path[:0] = [str(EXAMPLES_ROOT), str(EXAMPLES_ROOT / "07-optical-navigation")]

from ttc_strike.config import RuntimeConfig, StrikeConfig
from ttc_strike.config_loader import load_yaml_config
from ttc_strike.sensing import Barometer, BarometerReading, VerticalEstimator, VerticalImuReading


class BarometerTest(unittest.TestCase):
    """Verify the independent cadence and deterministic noise model."""

    def test_cadence_bias_and_seeded_drift(self) -> None:
        """Sample at 40 Hz, preserve bias, and repeat one seeded noisy run."""
        runtime = replace(
            RuntimeConfig(),
            barometer_sample_hz=40.0,
            barometer_noise_sigma_m=0.0,
            barometer_bias_m=0.4,
            barometer_drift_sigma_m_per_sqrt_s=0.03,
        )
        config = StrikeConfig(runtime=runtime)
        first, second = Barometer(config), Barometer(config)

        first_reading = first.sample(5.0, 0.0)
        self.assertIsNotNone(first_reading)
        self.assertAlmostEqual(first_reading.altitude_m, 5.4)
        self.assertIsNone(first.sample(5.0, 0.024))

        first_next = first.sample(5.0, 0.025)
        second.sample(5.0, 0.0)
        second_next = second.sample(5.0, 0.025)
        self.assertIsNotNone(first_next)
        self.assertIsNotNone(second_next)
        self.assertEqual(first_next, second_next)

    def test_altitude_ema_precedes_velocity_inference(self) -> None:
        """Filter altitude before deriving vertical speed from a noisy step."""
        config = StrikeConfig(runtime=replace(RuntimeConfig(), barometer_noise_sigma_m=0.0, barometer_altitude_old_weight=0.8))
        barometer = Barometer(config)
        first = barometer.sample(0.0, 0.0)
        second = barometer.sample(10.0, 0.025)
        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertEqual(second.raw_altitude_m, 10.0)
        self.assertAlmostEqual(second.altitude_m, 2.0)

    def test_vertical_estimator_barometer_correction(self) -> None:
        """Correct predicted IMU drift toward the barometer altitude."""
        config = StrikeConfig(runtime=replace(RuntimeConfig(), vertical_estimator_alpha=0.1, vertical_estimator_beta=0.0))
        estimator = VerticalEstimator(config, 0.0)
        estimate = estimator.update(VerticalImuReading(1.0), 1.0, BarometerReading(0.0, 0.0))
        self.assertAlmostEqual(estimate.altitude_m, 0.9)

    def test_yaml_sensor_validation(self) -> None:
        """Load valid runtime sensor settings and reject an invalid rate."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scenario.yaml"
            path.write_text("runtime:\n  sensors:\n    barometer:\n      sample_hz: 40\n      altitude_noise_sigma_m: 0.1\n")
            config = load_yaml_config(path)
            self.assertEqual(config.barometer_sample_hz, 40.0)
            self.assertEqual(config.barometer_noise_sigma_m, 0.1)

            path.write_text("runtime:\n  sensors:\n    barometer:\n      sample_hz: 0\n")
            with self.assertRaisesRegex(ValueError, "sample_hz"):
                load_yaml_config(path)


if __name__ == "__main__":
    unittest.main()
