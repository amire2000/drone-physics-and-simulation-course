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

## Documentation rules

When adding a lesson:

1. Explain the intuition first.
2. Add only the mathematics needed for the lesson.
3. Add a diagram when it clarifies forces or coordinate frames.
4. Add a small runnable example.
5. Add a hands-on exercise and 3–5 review questions.
6. Link prerequisites and the next topic.
7. Start with a short “By the end, you will be able to” list.
8. Separate major lesson sections with a Markdown horizontal rule (`---`).
9. Keep MkDocs Material's code-copy control enabled for every code block.
10. Add a concise docstring to every Python method or function introduced in a lesson example.
11. Add a Mermaid flow or event diagram near the module header for important modules; use it to show the main execution or event sequence before detailed sections.

Use MkDocs Material. Keep each lesson in `docs/modules/<module>/`, its images
in `docs/modules/<module>/images/`, and its runnable code in the matching
`examples/<module>/` folder. After documentation changes, run
`uv run mkdocs build --strict`.
