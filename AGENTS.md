# Learning Repository

This repository is a personal technical learning site.

## Goal

Teach drone simulation incrementally from rigid-body fundamentals to an
autonomous precision-hover capstone.

The learning track is:

PyBullet setup → forces → URDF and inertia → propeller forces → motor mixing
and PID → battery limits → autonomous hover.

## Code design rules

Apply SOLID principles pragmatically: give modules and classes clear
responsibilities, separate calculation logic from simulation and UI, and keep
interfaces small. Prefer composition and introduce abstractions only when they
support an actual extension or testing need.

### SOLID guide for PyBullet examples

- **Single Responsibility:** keep `main()` as a thin composition root. It may
  parse arguments, construct components, run the selected scenario, and close
  resources; it must not contain the full PyBullet simulation implementation.
- Keep PyBullet world creation, GUI/input handling, state reading, force
  application, controller calculations, and reporting in separate functions or
  focused classes.
- **Open/Closed:** add a new force or behavior by adding one topic-owned method
  and extending that topic's cumulative loop. Keep reusable setup, sensing,
  integration, and output utilities in `examples/common/`.
- **Liskov Substitution:** do not introduce inheritance unless the replacement
  can genuinely be used wherever the base type is expected. Prefer small
  composable callables or data objects for one-off examples.
- **Interface Segregation:** pass only the state, model, settings, or engine
  capability a function needs. Avoid handing every helper the entire PyBullet
  client and application state.
- **Dependency Inversion:** keep pure physics calculations independent of
  PyBullet where practical. Inject clocks, output paths, or simulation seams
  only when that makes the example testable; do not create abstractions for one
  implementation.
- A topic example must remain runnable on its own. Topic loops intentionally
  show the cumulative force sequence; shared vehicle data and backend utilities
  remain the source of truth for model values and infrastructure.
- Every new force or behavior method must have a concise Python docstring,
  inline comments for its equation, frame, sign convention, and call site, and
  a Markdown explanation with the equation, symbol table, SI units, numerical
  example, and implementation link.
- Each refactor must leave one small runnable self-check or deterministic
  command that proves the behavior still works.

### Better Comments convention

The workspace uses the Better Comments VS Code extension. Use color tags for
important implementation comments:

- `# !` for the current lesson's new force or behavior;
- `# *` for optional previous-topic reminders;
- `# ?` for an open design or learning question.

For cumulative simulation examples, place a prominent `# !` banner at the new
method definition and at its call site in the simulation loop. The banner must
name the topic, identify the behavior being introduced, and explain its place
in the cumulative sequence.

## Git rules

When the user asks to commit, show the proposed commit message and wait for
explicit approval before creating the commit.

## Documentation rules

When adding a lesson:

1. Explain the intuition first.
2. Add only the mathematics needed for the lesson.
3. Add a diagram when it clarifies forces or coordinate frames.
4. Add a small runnable example.
5. Add a hands-on exercise and 3–5 option-based checkpoint quizzes. Open-ended
   review questions may supplement the quiz, but must not be the only knowledge
   check.
6. Link prerequisites and the next topic.
7. Start with a short “By the end, you will be able to” list.
8. Separate major lesson sections with a Markdown horizontal rule (`---`).
9. Keep MkDocs Material's code-copy control enabled for every code block.
10. Add a concise docstring to every Python method or function introduced in a lesson example.
11. Add a Mermaid flow or event diagram near the module header for important modules; use it to show the main execution or event sequence before detailed sections.
12. End each lesson with a detailed “Topic summary” that reinforces the main
    physical insight, interprets the simulation evidence, identifies the
    important baseline or limitation, and explains how the result supports the
    next topic.

Use MkDocs Material. Keep each lesson in `docs/modules/<module>/`, its images
in `docs/modules/<module>/images/`, and its runnable code in the matching
`examples/<module>/` folder. After documentation changes, run
`uv run mkdocs build --strict`.

Use the existing `.quiz` pattern from `docs/javascripts/quiz.js` for checkpoint
quizzes. Each quiz must include plausible radio options, one correct answer, a
unique radio `name`, a visible `Check answer` button, and a brief explanation
through `data-explanation`.

## Design-document rules

Before implementing a non-trivial physics, control, architecture, integration,
or research decision, save its plan under `design/` in the matching group:
`physics/`, `navigation/`, `integration/`, `learning/`, or `research/`.
Update [`design/README.md`](design/README.md) whenever a design note is added,
moved, or materially revised.

Each design note must include a clear title, creation or revision date, short
description, status, key decisions, links to related lessons/examples, and
Mermaid diagrams when a flow or boundary would be clearer than prose. Use the
latest Git commit date for existing notes; use the current date for new or
revised notes.
