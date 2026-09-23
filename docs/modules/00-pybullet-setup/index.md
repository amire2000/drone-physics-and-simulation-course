# Module 0: Install PyBullet and run the GUI

## By the end, you will be able to

- Install this course's Python environment with `uv`.
- Open and inspect a small PyBullet scene.
- Run the same scene without a GUI when a display is unavailable.

## Install

This course uses [uv](https://docs.astral.sh/uv/) to keep every lesson in one reproducible Python environment. From the repository root, run:

```bash
uv sync
```

On Ubuntu, the GUI needs a running desktop session and graphics driver. If you are connected through a terminal without a display, use the headless command below instead.

## Run the first simulation

```bash
uv run python examples/00-pybullet-setup/sandbox.py
```

The script opens a window, creates a ground plane and a cube, then steps the world at 240 Hz. The cube falls because the world has gravity.

For a terminal-only run:

```bash
uv run python examples/00-pybullet-setup/sandbox.py --headless
```

Headless mode uses PyBullet's `DIRECT` connection. It does not open a window, but it reports the cube's final height and checks that it fell.

## The GUI

The large center view is the simulated world. Drag to orbit the camera, use the mouse wheel to zoom, and use the left sidebar to reset the simulation or toggle debug views. The simulator advances only when the script calls `stepSimulation()`: this is the loop that later applies forces and reads the drone state.

The plane and cube are deliberately simple. Before modelling a drone, we need to trust the world, gravity, time step, and visual inspection tools.

## Hands-on: change the experiment

1. Run the GUI script and watch the cube reach the plane.
2. In `examples/00-pybullet-setup/sandbox.py`, change `CUBE_START_HEIGHT`.
3. Run it again and compare how long the cube takes to reach the plane.
4. Change `GRAVITY_Z` to `-3.0`; predict the result before rerunning it.

The next module explains why those changes affect motion: [Module 1: Mass and external forces](../01-mass-and-forces/index.md).

## Review quiz

<form class="quiz" data-answer="a" data-explanation="It installs the dependencies declared for this project.">
  <fieldset><legend>1. What does <code>uv sync</code> do for this course?</legend>
    <label><input type="radio" name="q1" value="a"> Installs this project's dependencies</label><br>
    <label><input type="radio" name="q1" value="b"> Opens the PyBullet GUI</label><br>
    <label><input type="radio" name="q1" value="c"> Sets gravity to Earth gravity</label>
  </fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="b" data-explanation="GUI connects PyBullet to its interactive visualizer.">
  <fieldset><legend>2. Which connection opens the interactive PyBullet window?</legend>
    <label><input type="radio" name="q2" value="a"> DIRECT</label><br>
    <label><input type="radio" name="q2" value="b"> GUI</label><br>
    <label><input type="radio" name="q2" value="c"> UDP</label>
  </fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="a" data-explanation="The script sets downward gravity and advances the simulation.">
  <fieldset><legend>3. Why does the cube move in the starter scene?</legend>
    <label><input type="radio" name="q3" value="a"> Gravity acts while the simulation steps</label><br>
    <label><input type="radio" name="q3" value="b"> The plane pulls it down</label><br>
    <label><input type="radio" name="q3" value="c"> A motor pushes it down</label>
  </fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="b" data-explanation="Each call advances the physics world by one configured time step.">
  <fieldset><legend>4. What does <code>stepSimulation()</code> do?</legend>
    <label><input type="radio" name="q4" value="a"> Loads a drone model</label><br>
    <label><input type="radio" name="q4" value="b"> Advances the physics world one time step</label><br>
    <label><input type="radio" name="q4" value="c"> Moves the camera</label>
  </fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="b" data-explanation="Headless mode is for a terminal-only environment without a usable display.">
  <fieldset><legend>5. When should you use <code>--headless</code>?</legend>
    <label><input type="radio" name="q5" value="a"> Whenever you want to inspect the GUI</label><br>
    <label><input type="radio" name="q5" value="b"> When no usable display is available</label><br>
    <label><input type="radio" name="q5" value="c"> Only after Module 6</label>
  </fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>
