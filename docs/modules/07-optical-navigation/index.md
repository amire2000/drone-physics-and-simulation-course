# Module 7: Forward camera and monocular optical navigation

## By the end, you will be able to

- Attach a forward-facing RGB camera to the simulated drone body.
- Estimate real optical flow between consecutive camera frames.
- Explain why monocular optical flow measures relative image motion, not metric
  distance by itself.

---

## Module achievement

This planned module extends the simulator with a body-mounted RGB camera. The
drone will produce consecutive images, and an optical-flow algorithm will turn
their changing pixels into a motion estimate. Learners will compare this image
motion with known simulated movement and identify where scale is lost.

The result is a visual-navigation signal that can later support navigation
without directly reading perfect world position from PyBullet.

---

Prerequisite: [Module 6: Autonomous hover](../06-autonomous-hover/index.md).
Next: [Module 8: Betaflight SITL bridge](../08-betaflight-sitl/index.md).
