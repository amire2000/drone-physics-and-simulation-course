# Topic 12: Optional advanced forces

## By the end, you will be able to

- identify the limits of the simple force model;
- enable advanced effects one at a time;
- compare each effect with a baseline run.

```mermaid
flowchart LR
    baseline[Validated baseline] --> toggle[One optional effect]
    toggle --> compare[Before/after graph]
```

The optional settings cover rotor inflow, blade flapping, ground effect, and
gyroscopic torque. Run focused scenarios through the shared PyBullet engine;
keep them disabled for the first autonomous flight.

---

## Exercise and review

Enable one effect and record the graph change. Why should effects be enabled
one at a time? Which effect is most sensitive to low altitude?

Previous: [Topic 11](../11-control-and-mixing/index.md). Next: [Topic 13](../13-physics-engine-validation/index.md).
