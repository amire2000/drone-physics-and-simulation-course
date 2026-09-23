# Learning Repository

This repository is a personal technical learning site.

## Goal

Teach drone simulation incrementally from rigid-body fundamentals to an
autonomous precision-hover capstone.

The learning track is:

PyBullet setup → forces → URDF and inertia → propeller forces → motor mixing
and PID → battery limits → autonomous hover.

## Documentation rules

When adding a lesson:

1. Explain the intuition first.
2. Add only the mathematics needed for the lesson.
3. Add a diagram when it clarifies forces or coordinate frames.
4. Add a small runnable example.
5. Add a hands-on exercise and 3–5 review questions.
6. Link prerequisites and the next topic.
7. Start with a short “By the end, you will be able to” list.

Use MkDocs Material. Keep each lesson in `docs/modules/<module>/` and its
runnable code in the matching `examples/<module>/` folder. After documentation
changes, run `uv run mkdocs build --strict`.
