"""Train and load a small NumPy MLP that predicts vertical acceleration."""

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np


INPUT_SCALE = 3.0
OUTPUT_SCALE = 4.0
HIDDEN_NEURONS = 8
LEARNING_RATE = 0.08
EPOCHS = 4_000
SEED = 19


def teacher_acceleration(error_m: np.ndarray, vertical_velocity_mps: np.ndarray) -> np.ndarray:
    """Return the bounded PD acceleration labels used for the first learned policy."""
    return np.clip(1.2 * error_m - 1.2 * vertical_velocity_mps, -OUTPUT_SCALE, OUTPUT_SCALE)


def make_training_data() -> tuple[np.ndarray, np.ndarray]:
    """Create a deterministic grid of normalized state inputs and teacher outputs."""
    values = np.linspace(-INPUT_SCALE, INPUT_SCALE, 81)
    error_grid, velocity_grid = np.meshgrid(values, values)
    features = np.column_stack((error_grid.ravel() / INPUT_SCALE, velocity_grid.ravel() / INPUT_SCALE))
    targets = teacher_acceleration(error_grid.ravel(), velocity_grid.ravel()).reshape(-1, 1) / OUTPUT_SCALE
    return features, targets


def make_validation_data(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Create separately seeded random states that never take part in training updates."""
    states = rng.uniform(-INPUT_SCALE, INPUT_SCALE, size=(2_000, 2))
    features = states / INPUT_SCALE
    targets = teacher_acceleration(states[:, 0], states[:, 1]).reshape(-1, 1) / OUTPUT_SCALE
    return features, targets


@dataclass
class VerticalAccelerationMlp:
    """Store a 2-to-8-to-1 tanh regression network and predict acceleration."""

    w1: np.ndarray
    b1: np.ndarray
    w2: np.ndarray
    b2: np.ndarray

    @classmethod
    def random(cls, rng: np.random.Generator) -> "VerticalAccelerationMlp":
        """Create a network with small random weights and zero biases."""
        return cls(
            rng.normal(0.0, 0.5, size=(2, HIDDEN_NEURONS)),
            np.zeros((1, HIDDEN_NEURONS)),
            rng.normal(0.0, 0.5, size=(HIDDEN_NEURONS, 1)),
            np.zeros((1, 1)),
        )

    @classmethod
    def load(cls, path: Path) -> "VerticalAccelerationMlp":
        """Load trained NumPy weights from an NPZ file."""
        with np.load(path) as data:
            return cls(data["w1"], data["b1"], data["w2"], data["b2"])

    def save(self, path: Path) -> None:
        """Save trained weights so a simulator can run inference without retraining."""
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, w1=self.w1, b1=self.b1, w2=self.w2, b2=self.b2)

    def forward(self, features: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return normalized acceleration predictions and hidden-layer values."""
        hidden = np.tanh(features @ self.w1 + self.b1)
        return hidden @ self.w2 + self.b2, hidden

    def predict_acceleration(self, error_m: float, vertical_velocity_mps: float) -> float:
        """Convert physical state inputs into a bounded physical acceleration prediction."""
        features = np.array([[error_m / INPUT_SCALE, vertical_velocity_mps / INPUT_SCALE]])
        normalized_prediction, _ = self.forward(features)
        return float(np.clip(normalized_prediction[0, 0] * OUTPUT_SCALE, -OUTPUT_SCALE, OUTPUT_SCALE))


def train(model: VerticalAccelerationMlp, features: np.ndarray, targets: np.ndarray) -> list[float]:
    """Train the MLP with full-batch MSE gradient descent and return loss history."""
    losses: list[float] = []
    sample_count = features.shape[0]
    for epoch in range(EPOCHS):
        predictions, hidden = model.forward(features)
        error = predictions - targets
        losses.append(float(np.mean(error**2)))
        output_gradient = 2.0 * error / sample_count
        w2_gradient = hidden.T @ output_gradient
        b2_gradient = np.sum(output_gradient, axis=0, keepdims=True)
        hidden_gradient = (output_gradient @ model.w2.T) * (1.0 - hidden**2)
        w1_gradient = features.T @ hidden_gradient
        b1_gradient = np.sum(hidden_gradient, axis=0, keepdims=True)
        model.w1 -= LEARNING_RATE * w1_gradient
        model.b1 -= LEARNING_RATE * b1_gradient
        model.w2 -= LEARNING_RATE * w2_gradient
        model.b2 -= LEARNING_RATE * b2_gradient
        if epoch % 500 == 0 or epoch == EPOCHS - 1:
            print(f"epoch {epoch:4d} | MSE {losses[-1]:.6f}")
    return losses


def validation_mae(model: VerticalAccelerationMlp, features: np.ndarray, targets: np.ndarray) -> float:
    """Return mean absolute error in physical acceleration units on unseen states."""
    predictions, _ = model.forward(features)
    return float(np.mean(np.abs(predictions - targets)) * OUTPUT_SCALE)


def main() -> None:
    """Train a deterministic acceleration MLP, validate it, and save its weights."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/vertical_acceleration_mlp.npz"))
    args = parser.parse_args()
    rng = np.random.default_rng(SEED)
    features, targets = make_training_data()
    validation_features, validation_targets = make_validation_data(rng)
    model = VerticalAccelerationMlp.random(rng)
    train(model, features, targets)
    mae_mps2 = validation_mae(model, validation_features, validation_targets)
    assert mae_mps2 <= 0.15, f"Validation MAE {mae_mps2:.3f} m/s² is above the 0.15 m/s² gate"
    model.save(args.output)
    print(f"VALIDATION MAE: {mae_mps2:.3f} m/s²")
    print(f"MODEL SAVED: {args.output}")


if __name__ == "__main__":
    main()
