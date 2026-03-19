# OTF / Vision / Turret System Review — Findings Round 2

This document captures issues found during a deep review of the shoot-on-the-move,
vision localization, and turret control code. Each issue includes an explanation of
**why** it matters and what the fix looks like. The corresponding commits reference
this document.

---

## 1. CRITICAL — `smartShoot()` one-shot indexer gate

**Files:** `Cannon.java`

**The problem:**
`smartShoot()` uses a `WaitUntilCommand` as a one-shot gate: once the turret, hood,
and shooter are all on-target, it starts `autoIndex()` which feeds balls **forever**
without re-checking aim. If the turret or hood drifts off-target after the gate
opens (due to vision noise, collecting vibration, or robot motion), balls still fire
inaccurately.

Compare with `shootOTF()` which correctly uses `indexWhenOnTarget()` — a continuous
check that pauses the indexer whenever aim is lost.

**Why this is the likely root cause of the collect-and-shoot failure:**
When the copilot presses B, `smartShoot()` runs alongside `collectCommand()`. The
collector's 80A motor causes voltage sag and mechanical vibration. If this pushes the
turret or hood off-target after the one-shot gate already opened, the indexer
keeps feeding — launching balls while not aimed correctly.

**The fix:**
Replace the `WaitUntilCommand` + `autoIndex` pattern with a two-stage approach:
1. Wait for ALL systems (including shooter spin-up) to be ready.
2. Then continuously gate the indexer on turret + hood aim only.

We don't gate on the shooter continuously because each ball momentarily slows the
flywheel below tolerance, which would stutter the firing rate.

---

## 2. CRITICAL — `indexWhenOnTarget()` missing subsystem requirement and cleanup

**Files:** `Cannon.java`

**The problem:**
The `RunCommand` inside `indexWhenOnTarget()` doesn't declare the `Indexer` as a
required subsystem. This means:
- Other commands (like copilot manual index) can simultaneously control the indexer
  motors, causing conflicting outputs.
- When the parent command is interrupted (e.g., driver releases LB during OTF), the
  indexer has no `finallyDo` cleanup — it stays at whatever power it was last set to.
  Unlike turret/hood/shooter which have default commands, the indexer has **none**.

**The fix:**
Add the `indexer` subsystem as a requirement and add `finallyDo` cleanup.

---

## 3. HIGH — Vision timestamp calibration bakes in UDP jitter

**Files:** `PhotonVision.java`

**The problem:**
`macTimeOffset` is computed exactly once, on the very first valid UDP packet:
```java
if (macTimeOffset == 0) {
    macTimeOffset = Utils.getCurrentTimeSeconds() - updatedPose.timestamp;
}
```
Whatever network latency that first packet experienced is permanently baked into
**all future** vision timestamps. If the first packet is 50ms late, every subsequent
measurement is timestamped ~50ms too recent, and the Kalman filter under-corrects
for robot motion between capture and fusion.

**The fix:**
Use an exponential moving average (EMA) to continuously refine the offset. This
averages out UDP jitter while tracking any clock drift between the Mac and RIO.

---

## 4. HIGH — Mac-side ambiguity check uses unfiltered result

**Files:** `MacMini.java`

**The problem:**
Line 202 checks pose ambiguity on `latestResult` (before tag filtering) instead of
`useableResult` (after filtering). If the highest-confidence target is in the ignore
list, the remaining targets may have acceptable ambiguity — but the check still
rejects the entire result based on the removed tag.

**The fix:**
Use `useableResult.getBestTarget()` for the ambiguity check.

---

## 5. HIGH — 3-argument `addVisionMeasurement` override commented out with no explanation

**Files:** `Swerve.java`

**The problem:**
The 3-argument `addVisionMeasurement` override is commented out. This is actually
**correct** for the current usage — `PhotonVision` passes timestamps already
converted to CTRE's time domain via `macTimeOffset`, so applying `fpgaToCurrentTime()`
would double-convert. However, without explanation, someone may uncomment it thinking
it was accidentally disabled, breaking all vision timestamps.

Meanwhile, the 2-argument override **does** apply `fpgaToCurrentTime()`, creating an
inconsistent API: the two overloads expect different time domains.

**The fix:**
Add clear documentation explaining the time domain contract.

---

## 6. MODERATE — Default turret command has no angular velocity feedforward

**Files:** `Turret.java`, `RobotContainer.java`

**The problem:**
The default turret command (`cannon.turretAim()` → `turretAimCommand(Cannon)`) always
passes zero for chassis angular velocity feedforward. This means during `smartShoot()`
(which uses the default turret command, not OTF), any chassis rotation must be tracked
purely by PID — which lags behind.

If the driver turns while the copilot is shooting via `smartShoot()`, the turret has
no feedforward help, increasing the chance of firing while off-target.

**The fix:**
Pass the chassis omega to the default turret aim command too.

---

## 7. MODERATE — `shootOTF()` missing indexer cleanup on interrupt

**Files:** `Cannon.java`

**The problem:**
When the driver releases LB, `shootOTF()` is interrupted. The turret/hood/shooter
resume their default commands, but the indexer (controlled by `indexWhenOnTarget()`)
has no cleanup and no default command. The indexer stays at its last power setting.

**The fix:**
Addressed by fix #2 (adding `finallyDo` to `indexWhenOnTarget()`).

---

## 8. MODERATE — Vision trust uses same std dev for x, y, and rotation

**Files:** `PhotonVision.java`

**The problem:**
```java
VecBuilder.fill(trust, trust, trust)  // [x, y, theta]
```
Vision rotation estimates are typically more reliable than translation (tag geometry
constrains rotation well). Using the same trust value means either under-trusting
rotation or over-trusting translation. During shoot-on-the-move, over-trusting
translational position can cause the pose estimate to jump, making the turret chase
phantom movements.

**The fix:**
Use a tighter (lower) std dev for the rotation component.

---

## 9. MINOR — Sparse interpolation maps

**Files:** `Cannon.java`, `Hood.java`, `Shooter.java`

The `TIME_OF_FLIGHT_MAP`, `VELOCITY_MAP`, and `HOOD_MAP` all have only 3 data
points. Linear interpolation between these points may poorly approximate the actual
ballistic curves, especially at mid-range distances. Adding 2–3 more calibration
points would improve accuracy. This is a tuning task, not a code fix.
