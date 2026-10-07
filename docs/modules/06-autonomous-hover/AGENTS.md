# Module 6 documentation instructions

The repository rules in the root `AGENTS.md` apply here. These rules add the
Module 6 learning-path requirements.

## Lesson structure

- Keep Topics 0–14 in numbered folders under this module.
- Every topic must include intuition first, only the mathematics needed for the
  lesson, a runnable command, a hands-on exercise, 3–5 option-based quizzes,
  prerequisite and next-topic links, and a detailed `Topic summary`.
- Add a Mermaid diagram when it clarifies a force, coordinate frame, or event
  sequence.
- Add a graph, PNG, or animated GIF that shows the topic behavior over time or
  as a physical input changes. Use clear SI units, labels, legends, and axes.
- Embed generated visual assets from the topic's `images/` directory.

## Force and behavior explanations

For every new force or behavior method, document:

- the governing equation and symbol table;
- SI units and one numerical example;
- coordinate frame and sign convention;
- where the method is called in the topic simulation loop;
- a direct link to the implementation.

The module `index.md` is the learning-path overview. It should connect Topics
0–14 and explain the capstone, but detailed equations and experiments belong in
the topic lessons.
