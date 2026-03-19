# OTF (On-The-Fly) Shoot-on-the-Move Code Review Findings

This document summarizes bugs and improvements found during a review of the
shoot-on-the-move / OTF system, vision pipeline, and turret control code.

---

## Critical

### 1. Integer Division in `TIME_OF_FLIGHT_MAP` (Cannon.java)

**Lines:** `Cannon.java:54-55`

`35/30` and `24/30` are **integer division** in Java. They evaluate to `1` and
`0` respectively, instead of `1.167` and `0.8`. The 64-inch entry has a
time-of-flight of **zero seconds**, which causes `getFuturePoseFromTime(0)` to
return the current pose. The OTF iterative loop converges immediately because
the predicted pose never moves — the robot shoots with no lead at all.

**Impact:** OTF is fundamentally broken at close-to-mid range. The entire
motion-compensation algorithm is a no-op for the majority of shooting
distances.

**Fix:** Use `35d/30d` and `24d/30d` to force floating-point division.

---

## High

### 2. No Turret Angular Velocity Feedforward (Turret.java / Cannon.java)

The turret PID uses `kP=150, kD=12, kS=0.33` but **kV=0** (no velocity
feedforward). When the chassis rotates at angular velocity `omega`, the turret
must counter-rotate at `-omega` to keep its field-relative aim. Without
feedforward, the PID is purely reactive — it waits for position error to
accumulate before responding.

**Impact:** While driving and shooting, the turret constantly lags behind the
chassis rotation. The kD term helps but cannot fully compensate because it
responds to error rate rather than proactively canceling the known disturbance.

**Fix:** Compute chassis angular velocity and pass it as a feedforward voltage
to `PositionVoltage`. This lets the turret proactively counter-rotate against
the chassis without waiting for error to build up.

### 3. Vision Trust Floor of Zero (PhotonVision.java)

`trust = ambiguity * 1.2` can evaluate to ~0 when ambiguity is low (e.g.
multi-tag solve). A standard deviation of 0 tells the Kalman filter to place
**infinite** confidence in the vision measurement, completely overriding
odometry. Even tiny vision noise causes the pose estimate to snap/jump, which
propagates directly to turret aim via `drivetrain.getPose()`.

**Impact:** During active driving (especially while collecting), low-ambiguity
vision updates can jerk the pose estimate, causing turret oscillation and
missed shots.

**Fix:** Clamp the trust to a minimum floor (e.g. 0.3 meters standard
deviation) so vision is always blended with odometry, never overrides it.

---

## Medium

### 4. `turretAim(Target)` Ignores Its Argument (Cannon.java)

`Cannon.turretAim(Target target)` calls `turret.turretAimCommand(this)` which
reads `cannon.getTarget()` — the `target` parameter is silently ignored.

**Fix:** Pass the target through.

### 5. `indexWhenOnTarget` Never Re-checks Aim (Cannon.java)

Once `isOnTarget()` becomes true and the indexer starts, it feeds balls
continuously even if the system goes off-target. If the OTF calculation
oscillates (e.g. from vision pose jumps), balls fire while aimed incorrectly.

**Fix:** Gate the indexer power on a continuous on-target check rather than
a one-shot wait.

### 6. Duplicate `storedTarget` Assignment (Cannon.java)

The constructor assigns `storedTarget = FieldConstants.GOAL_POSITION` twice
(lines 97 and 99). Harmless but indicates a copy-paste error.

**Fix:** Remove the duplicate line.

---

## Notes

- The copilot B button (`smartShoot`) runs the **non-OTF** path while
  collecting. It does not use motion compensation — only the driver LB
  (`shootOTF`) does. This is by design but is worth considering when
  diagnosing collect-and-shoot failures.
- The 0.25-second `SPINDEXER_DELAY` in `autoIndex` means the robot moves
  for 250ms between the "on target" check and the ball actually reaching the
  flywheel. This is a physics limitation but should be accounted for in
  tuning.
- Current draw from the collector (80A stator), shooter (160A stator), and
  swerve drivetrain can cause battery voltage sag under heavy simultaneous
  load, momentarily disturbing all PID controllers.
