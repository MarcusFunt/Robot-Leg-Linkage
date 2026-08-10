# PR A — Motion profiles and static support analysis

This implementation intentionally focuses on a wheeled-biped linkage rather than a walking/contact-phase model.

## Included

- Jerk-limited symmetric S-curve reciprocation with configured maximum velocity, acceleration, and jerk.
- Analytic sinusoidal reciprocation with configured cycle time.
- One trajectory state (`theta`, `omega`, `alpha`) shared by animation and inverse dynamics.
- Static vertical support analysis at every crank angle, separate from the dynamic inertia case.
- Effective vertical moment arm, vertical support force per input torque, and normalized vertical mechanical advantage.
- Static holding torque including configured gravity.
- Time-domain RMS torque, peak torque, speed, power, and joint-reaction screening.
- Continuous requested-window reachability gating so impossible motion windows do not report zero/OK sizing results.
- Richer CSV export with trajectory and static-support columns.

## Explicitly out of scope

- Walking stance/swing load phases.
- Friction-cone or foot-ground slip analysis.
- Jump/impact models.
- Material databases or CAD mass-property import.
- Structural link stress/fatigue analysis.
