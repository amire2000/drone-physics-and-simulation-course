# Topic 10: Battery limits and motor dynamics

## By the end, you will be able to

- connect current draw to voltage sag;
- observe delayed RPM and reduced thrust.

```mermaid
flowchart LR
    command[Motor command] --> current[Battery current]
    current --> voltage[Voltage sag]
    voltage --> rpm[Available RPM]
    rpm --> thrust[Available thrust]
```

Run `uv run python examples/06-autonomous-hover/reduced_order_simulation.py --scenario battery`.

---

## Exercise and review

Compare voltage and RPM at low and high throttle. Why does the motor not jump
to its target RPM? What happens to yaw authority as voltage falls?

Previous: [Topic 9](../09-angular-damping/index.md). Next: [Topic 11](../11-control-and-mixing/index.md).
