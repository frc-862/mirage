# Network Findings: Mac Mini ↔ RoboRIO UDP Communication

This document describes reliability and correctness issues found in the UDP
networking layer between the Mac Mini vision processor (`frc.robot.mac.MacMini`)
and the RoboRIO receiver (`frc.robot.subsystems.PhotonVision`).

Each issue has been fixed in its own commit so you can review them one at a time.

---

## 1. Null pointer crash in Mac main loop (bug)

**File:** `MacMini.java` — `run()` method

`getEstimatedPose()` can return `null` (lines 116 and 128), but the main loop
only checks `info.pose != null` without first checking whether `info` itself is
null. This causes a `NullPointerException` that crashes the vision processor.

---

## 2. Socket creation failure silently ignored (bug)

**File:** `MacMini.java` — constructor

If `new DatagramSocket()` throws a `SocketException`, the error is logged but
`socket` remains `null`. The `run()` method then calls `socket.send()`, which
throws an NPE on every iteration — spinning forever and flooding the log with
"Failed to send packet" messages.

---

## 3. No packet magic number or validation (reliability)

**Files:** `MacMini.java`, `PhotonVision.java`

Any stray UDP traffic arriving on port 12345 is blindly parsed as a valid pose
and fed into the Kalman filter. There is no header, version byte, or integrity
check. On a busy FRC field network this could corrupt the pose estimator.

**Fix:** Added a 4-byte magic number (`0x00000862` — team 862), a 1-byte
protocol version, and a 4-byte sequence counter. The receiver rejects packets
that don't start with the correct magic/version bytes.

---

## 4. Time offset computed too late (accuracy)

**File:** `PhotonVision.java` — `periodic()` method

The "instant offset" between Mac time and RIO time was calculated inside
`periodic()`, which runs at 50 Hz. Up to 20 ms of scheduling delay between
packet arrival and `periodic()` execution gets baked into the offset, making
all vision timestamps systematically late.

**Fix:** Capture `Utils.getCurrentTimeSeconds()` in the receive thread at the
moment the packet arrives, and store it alongside the parsed data. The EMA
calculation in `periodic()` now uses that receive-time instead of process-time.

---

## 5. `InetAddress.getByName()` resolved every packet (performance)

**File:** `MacMini.java` — `getBinaryPacket()`

The destination address `"10.8.62.2"` was resolved from a string on every
packet send (~1000 times/sec). While Java may cache the result, this is
unnecessary overhead.

**Fix:** Resolve the address once in the constructor and reuse it.

---

## 6. No receive socket timeout (robustness)

**File:** `PhotonVision.java` — receive thread

`socket.receive()` blocks indefinitely. If the Mac Mini crashes or the network
cable is unplugged, the receive thread hangs forever with no exception, no log
message, and no way to detect the failure.

**Fix:** Set a 500 ms socket timeout. On timeout, the thread logs a warning and
loops back to receive again. This also enables staleness detection (see #7).

---

## 7. No staleness detection (reliability)

**File:** `PhotonVision.java` — `periodic()` method

When `pose.getAndSet(null)` returns null, there is no way to distinguish "no
tags visible this cycle" from "the Mac Mini has been dead for 10 seconds." The
ICMP ping thread only confirms the OS is up, not that the vision application is
running.

**Fix:** Track the timestamp of the last successfully received packet. In
`periodic()`, if the gap exceeds a threshold, publish a "vision stale" warning
to Shuffleboard.

---

## 8. ByteBuffer byte order not explicit (fragility)

**Files:** `MacMini.java`, `PhotonVision.java`

Both sides rely on Java's default `BIG_ENDIAN` byte order. This works today
because both are Java, but the contract is implicit and would silently break if
either side were ported to a language with a different default.

**Fix:** Explicitly set `ByteOrder.BIG_ENDIAN` on both sides.

---

## 9. Mac sends 20× faster than the RIO consumes (efficiency)

**File:** `MacMini.java` — `run()` loop

The Mac loops with a 1 ms sleep (~1000 Hz), but the RIO's `periodic()` runs at
50 Hz. The `AtomicReference.set()` overwrites ~19 out of every 20 packets
before they are read. All that pose-estimation work is wasted, and it burns CPU
that the PhotonVision pipeline could use.

**Fix:** Increased the sleep to 15 ms (~66 Hz), which still provides multiple
fresh readings per RIO cycle while dramatically reducing wasted work.

---

## 10. Ambiguity threshold off-by-one (minor)

**Files:** `MacMini.java`, `PhotonVision.java`

The Mac filters with `ambiguity > 1.0` (rejects values above 1.0, keeps 1.0)
but the RIO filters with `ambiguity < 1` (rejects 1.0). A pose with exactly
1.0 ambiguity passes the Mac but is rejected by the RIO.

**Fix:** Aligned both sides to use `<=` / `>=` consistently so the boundary
value is handled the same way.
