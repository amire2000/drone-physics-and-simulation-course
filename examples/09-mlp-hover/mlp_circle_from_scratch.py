"""Train a small NumPy MLP to classify points inside a circle."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


RADIUS = 0.55
HIDDEN_NEURONS = 8
LEARNING_RATE = 0.15
EPOCHS = 2_500
SEED = 7


def make_dataset(rng: np.random.Generator, count: int) -> tuple[np.ndarray, np.ndarray]:
    """Create labelled points: one inside the circle, zero outside it."""
    points = rng.uniform(-1.0, 1.0, size=(count, 2))
    labels = (np.sum(points**2, axis=1, keepdims=True) <= RADIUS**2).astype(float)
    return points, labels


def initialize_parameters(rng: np.random.Generator) -> dict[str, np.ndarray]:
    """Create small random weights and zero biases for a 2-to-8-to-1 MLP."""
    return {
        "w1": rng.normal(0.0, 0.7, size=(2, HIDDEN_NEURONS)),
        "b1": np.zeros((1, HIDDEN_NEURONS)),
        "w2": rng.normal(0.0, 0.7, size=(HIDDEN_NEURONS, 1)),
        "b2": np.zeros((1, 1)),
    }


def sigmoid(values: np.ndarray) -> np.ndarray:
    """Convert any real number into a probability between zero and one."""
    return 1.0 / (1.0 + np.exp(-np.clip(values, -50.0, 50.0)))


def forward(points: np.ndarray, parameters: dict[str, np.ndarray]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Run points through the hidden layer and return inside-circle probabilities."""
    hidden_input = points @ parameters["w1"] + parameters["b1"]
    hidden_output = np.tanh(hidden_input)
    output_input = hidden_output @ parameters["w2"] + parameters["b2"]
    probabilities = sigmoid(output_input)
    return probabilities, {"points": points, "hidden": hidden_output, "probabilities": probabilities}


def binary_cross_entropy(labels: np.ndarray, probabilities: np.ndarray) -> float:
    """Measure how far predicted probabilities are from zero-or-one labels."""
    safe_probabilities = np.clip(probabilities, 1e-7, 1.0 - 1e-7)
    return float(-np.mean(labels * np.log(safe_probabilities) + (1.0 - labels) * np.log(1.0 - safe_probabilities)))


def backward(
    labels: np.ndarray,
    parameters: dict[str, np.ndarray],
    cache: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    """Use the prediction error to calculate one gradient for every parameter."""
    sample_count = labels.shape[0]
    output_error = (cache["probabilities"] - labels) / sample_count
    gradients_w2 = cache["hidden"].T @ output_error
    gradients_b2 = np.sum(output_error, axis=0, keepdims=True)

    hidden_error = (output_error @ parameters["w2"].T) * (1.0 - cache["hidden"] ** 2)
    gradients_w1 = cache["points"].T @ hidden_error
    gradients_b1 = np.sum(hidden_error, axis=0, keepdims=True)
    return {"w1": gradients_w1, "b1": gradients_b1, "w2": gradients_w2, "b2": gradients_b2}


def update_parameters(parameters: dict[str, np.ndarray], gradients: dict[str, np.ndarray]) -> None:
    """Move each weight and bias a small step toward lower loss."""
    for name, gradient in gradients.items():
        parameters[name] -= LEARNING_RATE * gradient


def accuracy(labels: np.ndarray, probabilities: np.ndarray) -> float:
    """Return the percentage of inside-or-outside decisions that are correct."""
    predictions = probabilities >= 0.5
    return float(np.mean(predictions == labels))


def train(
    train_points: np.ndarray,
    train_labels: np.ndarray,
    validation_points: np.ndarray,
    validation_labels: np.ndarray,
    parameters: dict[str, np.ndarray],
) -> tuple[list[float], list[float]]:
    """Train on one dataset while recording loss on unseen validation points."""
    train_losses: list[float] = []
    validation_losses: list[float] = []
    for epoch in range(EPOCHS):
        train_probabilities, cache = forward(train_points, parameters)
        gradients = backward(train_labels, parameters, cache)
        update_parameters(parameters, gradients)

        train_probabilities, _ = forward(train_points, parameters)
        validation_probabilities, _ = forward(validation_points, parameters)
        train_losses.append(binary_cross_entropy(train_labels, train_probabilities))
        validation_losses.append(binary_cross_entropy(validation_labels, validation_probabilities))

        if epoch % 250 == 0 or epoch == EPOCHS - 1:
            print(
                f"epoch {epoch:4d} | train loss {train_losses[-1]:.4f} | "
                f"validation loss {validation_losses[-1]:.4f} | "
                f"validation accuracy {accuracy(validation_labels, validation_probabilities):.1%}"
            )
    return train_losses, validation_losses


def plot_results(
    validation_points: np.ndarray,
    validation_labels: np.ndarray,
    parameters: dict[str, np.ndarray],
    train_losses: list[float],
    validation_losses: list[float],
    output_path: Path,
) -> None:
    """Save the learned decision region and loss curves to one teaching plot."""
    axis = np.linspace(-1.0, 1.0, 220)
    grid_x, grid_y = np.meshgrid(axis, axis)
    grid_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    grid_probabilities, _ = forward(grid_points, parameters)
    probability_map = grid_probabilities.reshape(grid_x.shape)

    figure, (decision_axis, loss_axis) = plt.subplots(1, 2, figsize=(13, 5))
    decision_axis.contourf(grid_x, grid_y, probability_map, levels=np.linspace(0.0, 1.0, 12), cmap="RdYlBu", alpha=0.75)
    decision_axis.contour(grid_x, grid_y, probability_map, levels=[0.5], colors="black", linewidths=2)
    outside = validation_labels[:, 0] == 0.0
    decision_axis.scatter(validation_points[outside, 0], validation_points[outside, 1], c="#2563eb", s=18, label="outside: 0")
    decision_axis.scatter(validation_points[~outside, 0], validation_points[~outside, 1], c="#ea580c", s=18, label="inside: 1")
    decision_axis.set(title="MLP decision boundary on unseen validation points", xlabel="x", ylabel="y", xlim=(-1, 1), ylim=(-1, 1), aspect="equal")
    decision_axis.legend(loc="upper right")

    loss_axis.plot(train_losses, label="training loss", color="#2563eb")
    loss_axis.plot(validation_losses, label="validation loss", color="#ea580c")
    loss_axis.set(title="Loss becomes smaller during training", xlabel="epoch", ylabel="binary cross-entropy")
    loss_axis.legend()
    loss_axis.grid(alpha=0.3)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> None:
    """Create data, train the MLP, validate it, and save a learning plot."""
    rng = np.random.default_rng(SEED)
    train_points, train_labels = make_dataset(rng, count=800)
    validation_points, validation_labels = make_dataset(rng, count=400)
    parameters = initialize_parameters(rng)
    train_losses, validation_losses = train(
        train_points, train_labels, validation_points, validation_labels, parameters
    )
    validation_probabilities, _ = forward(validation_points, parameters)
    validation_accuracy = accuracy(validation_labels, validation_probabilities)
    output_path = Path("outputs/mlp_circle_classifier.png")
    plot_results(
        validation_points, validation_labels, parameters, train_losses, validation_losses, output_path
    )
    print(f"\nFINAL VALIDATION ACCURACY: {validation_accuracy:.1%}")
    print(f"PLOT SAVED: {output_path}")
    assert validation_accuracy >= 0.95, "The default MLP should learn the circle reliably."


if __name__ == "__main__":
    main()
