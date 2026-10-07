# Module 6 example instructions

The repository rules in the root `AGENTS.md` apply here. These rules add the
Module 6 simulation architecture.

## Topic-owned simulation loops

- Every topic example must run independently.
- Each topic owns its `run_experiment()` PyBullet loop, reduced-order loop,
  new force or behavior method, cumulative force sequence, and validation.
- The loop must visibly call the methods copied from previous topics before it
  calls the new method introduced by the current topic.
- Do not move force application into a generic runner or force registry.
- PyBullet and reduced-order loops must use the same equations. Backend-specific
  adapters are allowed only where the backend supplies different infrastructure,
  such as native contact resolution.

## Code regions

Use editor-foldable Python regions to make the cumulative learning sequence
visible in each topic example:

```python
# region Previous topic forces
...
# endregion

# region Current topic force
...
# endregion

# region Motor state and telemetry
...
# endregion

# region Topic simulation loops
...
# endregion
```

Keep previous force methods copied and readable inside the `Previous topic
forces` region. Put the new lesson method in `Current topic force` and add a
prominent banner comment at both the method definition and its loop call site.
The banner must name the topic and the behavior being introduced, explain its
position in the cumulative force order, and make the new method easy to find
when reading the loop. Use the same region names consistently across Topics
0–14. With the Better Comments extension, prefix new-topic banners with
`# !` so they use the configured highlight color; use `# *` for optional
previous-topic reminders.

## Shared infrastructure boundary

Use `examples/common/` for reusable vehicle data, telemetry types, common CLI
parsing, sensing, integration, recording, and output helpers.

The common topic runner may own backend selection, PyBullet connection cleanup,
summary/output handling, and GUI exit handling. It must receive topic-owned
loop and validation callables; it must never apply a force or hide the topic's
cumulative force order.

Interactive PyBullet runs use the shared `examples/common/tk_controls.py`
external Tk window for Start, Pause, Restart, and Quit. The runner owns the window
lifecycle; each topic loop polls it and owns the reset of its physics state.
Do not put Tk calls in force methods or duplicate control windows in topics.

Interactive topics may use `examples/common/safety.py` for conservative
auto-pause limits. Safety checks may pause and explain a run, but they must not
apply forces or become a hidden controller.

The shared `Sample` dataset is passive telemetry only:

- topic loops create and populate samples;
- `Sample` must not apply forces, read PyBullet, or advance physics;
- common `save_results()` may serialize CSV data and create selected graphs;
- common `parse_args()` owns shared CLI options; topic-specific options are
  declared as module-level argument constants;
- common output code must not calculate topic physics or hide the force sequence.

## Force-method requirements

Every new force or behavior method must include:

- a concise Python docstring;
- inline comments for its equation;
- frame and sign-convention comments;
- a comment explaining where it is called in the loop;
- a matching Markdown explanation in the topic lesson.

Each topic must leave one runnable self-check or deterministic command that
proves its behavior works. After documentation changes, run the relevant topic
self-checks and:

```bash
uv run mkdocs build --strict
```
