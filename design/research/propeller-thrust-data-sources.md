# Propeller thrust-data sources for Module 3

**Last updated:** 2026-09-29
**Status:** Adopted
**Description:** Trusted bench-data sources and the selected Module 3 teaching dataset.

## Recommendation

Use a **small, typed Markdown table** from EMAX's official ECO II 3210
bench data, with a link to the original manufacturer table. It is the best
first tutorial example because the page keeps complete test rows for several
props and explicitly identifies the motor KV and propeller combination.

Do not embed a screenshot of a manufacturer table unless the manufacturer
gives permission. The pages below do not state a reusable-data or image
licence. A short, attributed table of facts plus a direct source link is the
safest teaching format; learners can open the complete original data.

---

## Source 1 — recommended first example

**Manufacturer:** EMAX  
**Product:** [ECO II 3210 motor family](https://shop.emaxmodel.com/collections/all/products/emax-ecoii-3210-800kv-900kv-1050kv-1200kv-brushless-drone-motor-for-8-10inch-fpv-rc-drone)  
**Matched setup:** 900 KV motor with a `GF9045` tri-blade propeller.  
**Published fields:** motor KV, recommended cells/propeller range, propeller
name and blade count, voltage, current, RPM, thrust, power, efficiency, and
throttle.

Suggested lesson extract, transcribed from EMAX's official table:

| Throttle | Voltage | Current | RPM | Thrust |
| ---: | ---: | ---: | ---: | ---: |
| 30% | 25.0 V | 2.68 A | 5,856 RPM | 397 g |
| 50% | 24.9 V | 9.77 A | 9,312 RPM | 1,138 g |
| 70% | 24.8 V | 22.64 A | 12,290 RPM | 1,992 g |
| 100% | 24.5 V | 54.90 A | 15,925 RPM | 3,408 g |

Why it works for the tutorial:

- Each row supplies the complete calibration facts already required by the
  propeller lesson.
- The measured voltage drops as current rises, making battery sag visible.
- It supports a simple exercise: convert 1,992 g to newtons, then calculate
  a single-point candidate \(k_T\). Explain that one point is a teaching
  approximation, not a complete propeller model.

---

## Source 2 — best comparison database

**Manufacturer:** T-Motor  
**Product:** [Velox V2208 V2, 1750 KV](https://www.t-hobby.com/products/fpv-brushless-motor-v2208-v2-for-freestyle-drones)  
**Matched propeller:** `T5143S` (5.1-inch, 4.3-inch pitch, tri-blade by the
manufacturer's naming)  
**Published fields:** throttle, voltage, current, RPM, thrust, power,
efficiency, and ambient temperature.

Useful 6S 1750 KV rows from the manufacturer table:

| Throttle | Voltage | Current | RPM | Thrust |
| ---: | ---: | ---: | ---: | ---: |
| 50% | 23.58 V | 6.51 A | 18,900 RPM | 536.79 g |
| 70% | 23.41 V | 14.43 A | 24,410 RPM | 928.24 g |
| 100% | 23.04 V | 30.76 A | 30,807 RPM | 1,460.13 g |

Why keep it as the second exercise:

- It is closer to the course's 5-inch/6S/1750 KV reference.
- The former manufacturer-store URL redirects, so retain it as a linked
  alternative rather than the course's embedded source.

---

## Additional official sources

| Source | Useful data | Best use |
| --- | --- | --- |
| [T-Motor P2306 V3 1750 KV](https://www.t-hobby.com/products/tmotor-p2306-v3) | 5-inch propeller tables with throttle, V, A, RPM, thrust, power, and efficiency | Alternative 5-inch 6S/1750 KV example; compare a `P49436-3` propeller with the course's 5×4.3×3 reference. |
| [EMAX ECO II 4315](https://emaxmodel.com/motor-detail-dark.html?motor=eco-ii-4315) | Four KV options, 13–15-inch propellers, plus voltage, current, thrust, power, efficiency, RPM, and throttle test data | Later large-drone lesson; not the first beginner example. |
| [APC performance-data archive](https://www.apcprop.com/technical-information/performance-data/) | Geometry and analytical propeller performance across current production props | Propeller-only geometry/RPM/thrust comparison; it does not provide motor KV, battery voltage, or current. |

---

## Safe way to embed the chosen data

Use this course-owned Markdown table and cite the EMAX page immediately
below it:

```markdown
| Throttle | Voltage | Current | RPM | Thrust |
| ---: | ---: | ---: | ---: | ---: |
| 30% | 25.0 V | 2.68 A | 5,856 RPM | 397 g |
| 50% | 24.9 V | 9.77 A | 9,312 RPM | 1,138 g |
| 70% | 24.8 V | 22.64 A | 12,290 RPM | 1,992 g |
| 100% | 24.5 V | 54.90 A | 15,925 RPM | 3,408 g |

Source: [EMAX ECO II 3210 official test data](https://shop.emaxmodel.com/collections/all/products/emax-ecoii-3210-800kv-900kv-1050kv-1200kv-brushless-drone-motor-for-8-10inch-fpv-rc-drone).
```

Label it **"Selected rows, transcribed from the manufacturer's bench-test
table"**. Do not call it independently verified, and do not use it to claim
that another motor, battery, or propeller will deliver the same result.

EMAX's [terms of service](https://shop.emaxmodel.com/pages/terms-of-service)
state that its website content is copyrighted. The table above is a short
factual extract with attribution; do not copy its product photography,
graphics, or complete data tables into the course without written permission.

## Source check date

Sources checked 2026-09-29. Manufacturer pages can change or be replaced;
keep the direct link next to every embedded extract.
