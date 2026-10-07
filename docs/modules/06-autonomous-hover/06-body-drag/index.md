# Topic 6: Body drag

## By the end, you will be able to

- calculate quadratic drag from air-relative velocity;
- explain why drag opposes motion.

```mermaid
flowchart LR
    velocity[Air-relative velocity] --> drag[Quadratic body drag]
    drag --> decel[Reduced acceleration]
```

Run `uv run python examples/06-autonomous-hover/reduced_order_simulation.py --scenario drag`.

---

## Exercise and review

Double the drag scale and compare velocity decay. Why does drag grow faster
than velocity? Which body axis has the largest projected area?

Previous: [Topic 5](../05-reaction-yaw-torque/index.md). Next: [Topic 7](../07-rotor-drag/index.md).
