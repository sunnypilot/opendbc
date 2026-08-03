# CCNC non-HDA2 steering-fault investigation

Status: active research; Panda-timed lane-gated transformer passed initial road validation, force-ownership validation pending
Vehicle: 2024 Hyundai Sonata Hybrid without HDA II (`HYUNDAI_SONATA_HEV_2024`)
Branch: `ccnc-port-lite`
Last updated: 2026-08-03

## Executive summary

The steering fault is not explained by missing DBC bits, an incorrect LFA counter, an incorrect LFA checksum, excessive steering angle, driver torque, or openpilot torque content alone.

The strongest evidence points to the way the non-HDA2 camera's 100 Hz `LFA` stream reaches the MDPS/EPS. On this CCNC car the front camera is the original LFA producer and the MDPS appears to require that physical stream to remain continuous and tightly timed. Relaying the camera message through `card`/USB, even byte-for-byte, creates bursts, gaps, or dropped counters. Exact stock content relayed through the application layer eventually makes the MDPS enter its fail state. The MDPS failure occurs before the camera drops its request and before CCNC reports the visible `0x162` faults.

The next design therefore does not generate or replay `LFA` in openpilot. Panda forwards each real camera frame at hardware timing and optionally changes only:

- `StrTqReqVal` (bits 41..51)
- `ActToiSta` (bits 52..53)
- the 16-bit Hyundai checksum (bits 0..15)

The physical camera counter, every other defined signal, every currently unknown bit, and the physical arrival cadence remain camera-owned. A small safety-checked command from openpilot is consumed inside Panda and never transmitted on CAN. Invalid, stale, disengaged, or unsafe commands revert immediately to unmodified stock pass-through. Nothing suppresses or edits the CCNC fault message `0x162`.

The lane-gated implementation subsequently passed an initial moving road validation: a continuous 90-second window contained 8,988 source frames and 8,988 physical outputs, no counter discontinuities, 9,002 healthy MDPS samples, and 1,797 clear CCNC fault samples. A separate corrected 20-second byte-pairing capture included two request handoffs and showed 218 exact frames, 1,777 modified frames, and zero changes outside torque, request, and checksum. This exceeds the prior 7-second, 26-second, and 30-plus-second false-clean windows.

The Panda protocol already supports a second, disabled-by-default force-ownership mode for steering without lane lines. That remains the next experiment. The global Panda blocked-transmission counter increased by two during the handoff capture even though physical forwarding and every vehicle fault channel remained clean; this boundary behavior must be attributed before the implementation is considered production-ready.

## Goal and non-goals

The goal is to make openpilot steer this direct-camera, non-HDA2 CCNC architecture without hiding faults and without disabling the camera or MDPS diagnostics that expose the problem.

The investigation explicitly does **not**:

- block or rewrite `CCNC_0x162` fault fields;
- mask `FAULT_LSS`, `FAULT_LFA`, or `FAULT_DAS`;
- block the MDPS status message `0xEA` from the camera;
- claim success based only on standstill testing or roads where the camera never requests torque;
- treat a clean short window as proof of a fix;
- attempt to fix upstream openpilot loop lag as part of this work.

The existing `ccnc-port` branch can steer because it blocks fault propagation and sends replacement steering messages. That is useful operationally, but it cannot reveal which requirement the stock system is enforcing. This branch keeps the diagnostic path intact.

## Vehicle and network architecture

### Non-HDA2 CCNC vehicle under test

The vehicle has no separate HDA2 ADAS driving ECU. The front camera produces the 16-byte `LFA` message at approximately 100 Hz and sends it directly toward MDPS/EPS.

With the Hyundai A harness used here:

- Panda bus 2 is the camera side.
- Panda bus 0 is E-CAN / the car and MDPS side.
- Camera `LFA` is address `0x12A` (decimal 298), length 16.
- MDPS status is address `0xEA` (decimal 234), length 24.
- The CCNC fault/status display message is `0x162` (decimal 354), length 32.

### Why HDA2 behavior is not a valid control for this topology

HDA2 cars have a separate ADAS unit and a different harness/message path. The camera's LKA command is consumed or forwarded by that ADAS unit, and openpilot can disable or replace the relevant ADAS output. A direct camera-to-MDPS non-HDA2 car does not have that timing boundary. Success on HDA2 therefore does not prove that a host-generated `LFA` stream is acceptable to this MDPS.

### The three forwarding architectures tested

```text
Stock / raw Panda forwarding

  camera bus 2 -- physical 0x12A --> Panda -- immediate forward --> bus 0 MDPS

Application replay (faults)

  camera bus 2 --> Panda --> USB --> card/CarState --> CarController
                                                |
  bus 0 MDPS <-- Panda <-- USB <-- rebuilt/replayed 0x12A

Panda-timed transform (next road test)

  openpilot -- safety-checked desired torque command --> Panda internal state
                                                           |
  camera bus 2 -- physical 0x12A --> Panda edits in place --+--> bus 0 MDPS
```

The last architecture retains the timing property of the first while allowing the torque ownership needed by openpilot.

## Reproducible symptom

The visible symptom is a `Take Control Immediately` alert on the comma four plus dashboard faults when LFA or LKA attempts real steering while the car is moving.

Observed trigger matrix:

| Condition | Result |
|---|---|
| Parked / standstill | No representative steering fault; MDPS is not exercising the same moving-state path |
| LFA enabled on a road with no usable lane lines | Engage/disengage can remain clean because the camera normally keeps `ActToiSta=0` and requests no steering |
| LFA enabled with usable lane lines | Fault occurs when an active steering request is sent |
| Stock LKA lane-departure intervention | Fault can also occur when the camera requests steering for a lane departure |
| Engage or disengage while moving after active steering | Faults can be reported during the handoff |
| Openpilot torque substituted into relayed LFA | Faults, sometimes after an initially clean interval |
| Exact stock camera bytes replayed through the host | Also faults after a variable interval |

The no-lane-line result is not evidence that the message relay is correct. It is a negative control showing that the failure requires the active MDPS torque-request path.

## Relevant CAN signals

### Camera `LFA` (`0x12A`, 16 bytes)

The DBC now covers all 128 bits, including previously unnamed fields. The most important fields for this investigation are:

| Signal | Bits | Meaning in this investigation |
|---|---:|---|
| `CHECKSUM` | 0..15 | Hyundai CAN-FD CRC16, little-endian in bytes 0 and 1 |
| `COUNTER` | 16..23 | Physical camera rolling counter, 0..255 |
| `StrTqReqVal` | 41..51 | Signed requested steering torque, raw offset -1024 |
| `ActToiSta` | 52..53 | Torque-overlay request/state; observed active value is 1 |
| `ToiFltSta` | 54..55 | Camera-side torque-overlay fault state |
| `LKA_SysWrn` | 60..63 | LKA warning state |
| `Damping_Gain` | 104..111 | Damping configuration retained from the source frame |

The fully mapped payload also contains `LKA_OptUsmSta`, `LKA_RcgSta`, lane warning/sound fields, `LKA_SysIndReq`, the LFA button, BCA/FCA/ELK states, pedestrian information, and `NEW_SIGNAL_1` through `NEW_SIGNAL_8`. The new transformer preserves all of those fields bit-for-bit.

The source definition is in `opendbc/dbc/generator/hyundai/hyundai_canfd.dbc` under `BO_ 298 LFA`.

### MDPS status (`0xEA`)

The decisive MDPS fields are:

| Signal | Bits | Observed use |
|---|---:|---|
| `MDPS_LkaToiActvSta` | 48..49 | MDPS acknowledgement/active state |
| `MDPS_LkaToiUnblSta` | 50..51 | MDPS unable state |
| `MDPS_LkaToiFltSta` | 52..53 | MDPS detects a torque-overlay fault |
| `MDPS_LkaFailSta` | 54..55 | MDPS is in the LKA torque-overlay fail state |
| `MDPS_OutTqVal` | 64..75 | MDPS output torque |
| `MDPS_StrTqSnsrVal` | 80..92 | Driver/column torque sensor |
| `MDPS_PaStrAnglVal` | 96..111 | Steering angle used during event correlation |

The DBC comments describe `MDPS_LkaToiFltSta` as covering CAN failures, internal MDPS failures, and LKA-controller logic failures. That is consistent with a stream-integrity/timing rejection even when the torque value itself is modest.

### CCNC fault message (`0x162`)

The visible faults correlated in this work are:

| Signal | Bits | Display meaning for value 1 |
|---|---:|---|
| `FAULT_LSS` | 219..221 | Check lane safety system |
| `FAULT_LFA` | 234..236 | Check lane following assist system |
| `FAULT_DAS` | 246..248 | Check driver assistance system |

These signals are observed only. They are not blocked or changed by the proposed solution.

## Checksum result

For the 16-byte Hyundai CAN-FD `LFA` payload, the validated checksum procedure is:

1. Start CRC at zero.
2. Process payload bytes 2 through 15 using polynomial `0x1021`.
3. Process address low byte, then address high byte.
4. XOR the result with `0x041D` for a 16-byte message.
5. Store the result little-endian in bytes 0 and 1.

The captured sample below validates with this algorithm:

```text
3d b7 8a 00 00 01 08 f0 00 0a 00 00 00 64 00 00
```

The checksum implementation is shared in `hyundai_common_canfd_compute_checksum`. Checksum correctness was necessary for exact reconstruction, but correct checksums did not prevent the application-relay fault.

## Experiment log

### 1. Baseline trigger characterization

**Question:** Does simply pressing the LFA button cause the fault, or does real steering matter?

**Method:** Engage/disengage LFA at standstill, on roads without usable lane lines, and on roads with lane lines. Also observe stock LKA lane-departure behavior.

**Result:** Button use without an active request can be clean. The fault follows actual steering requests while moving, including stock LKA intervention.

**Conclusion:** The trigger is in the active torque-overlay path, not the button itself or only openpilot's engagement state.

### 2. Reconstruct `LFA` with the previously known DBC signals

**Question:** Is the generated message rejected because openpilot does not copy enough stock fields?

**Method:** Copy the decoded camera `LFA` into `CarState`, overwrite only the steering fields, repack, and send it to E-CAN.

**Result:** Steering still faulted.

**Conclusion:** Either unrepresented bits, counter/checksum behavior, ownership, or timing still differed.

### 3. Add every previously empty bit to the DBC

**Question:** Were unmapped bits being zeroed by the packer and invalidating the message?

**Method:** Add `NEW_SIGNAL_1` through `NEW_SIGNAL_8` and all other empty regions so the entire 16-byte payload could be decoded and reconstructed.

**Result:** With steering inactive, the repacked message was proven byte-for-byte equal to the camera source. Active steering still faulted.

**Conclusion:** Missing DBC coverage was a real byte-preservation concern, but it was not the root cause of the steering fault.

### 4. Seed the generated counter from the camera counter plus one

**Question:** Does MDPS require openpilot to continue the camera counter sequence rather than start its own?

**Method:** Initialize the host-generated `COUNTER` from the latest camera counter and advance it.

**Result:** Fault persisted.

**Conclusion:** A correctly seeded independent host counter is insufficient.

### 5. Follow every camera counter using `vl_all`

**Question:** Are duplicate or skipped reconstructed counters the cause?

**Method:** Copy all camera messages received during each parser update and emit reconstructed messages with each observed source counter.

**Result:** This removed the obvious duplicate-counter behavior in the host output, but the car still faulted.

**Conclusion:** Logical counter order in the batch is not enough; physical delivery timing remains different.

### 6. Raw Panda forwarding

**Question:** Can an untouched physical camera stream pass through the harness cleanly?

**Method:** Allow Panda's normal hardware forwarding path to pass camera `0x12A` from bus 2 to bus 0 without application replay.

**Result:** Clean no-fault windows were observed. Some early windows did not contain meaningful lane torque, so they were not treated as conclusive on their own.

**Conclusion:** Raw forwarding is the correct control path and must be preserved in the final design.

### 7. Initial application replay and Panda rejection

**Question:** Is application-level exact replay truly reaching the vehicle?

**Method:** Replay the camera bytes from openpilot and inspect Panda TX acceptance.

**Result:** Nonzero stock torque could be rejected by standard Panda torque safety while openpilot controls were off. A host-side claim of exact replay was therefore not proof that the same bytes reached MDPS.

**Conclusion:** Any replay experiment needs explicit provenance-aware safety accounting or a different architecture.

### 8. One-use stock-message provenance cache

**Question:** Can Panda accept exact stock replay without weakening checks for modified commands?

**Method:** Cache each physical camera `LFA` in Panda safety. Permit one identical host replay, consume the cached entry, synchronize torque-safety state from stock, and apply normal steering limits to any modified frame. Reject stale/duplicate copies.

**Result:**

- 495 of 495 comparable exact stock replays were accepted.
- Panda reported zero rejections in the clean comparison window.
- `0x162` remained clear in that window.
- A modified openpilot takeover produced 368 consecutive accepted torque frames and approximately 4.2 seconds of clean steering.
- Longer clean windows, including more than 30 seconds and clean handoffs, were also seen.
- Faults eventually returned with zero Panda TX rejections.

**Conclusion:** Panda rejection explained earlier misleading results but was not the final root cause. The variable delay to failure made short tests especially misleading.

### 9. Establish event causality with MDPS and CCNC timelines

**Question:** Which ECU reports failure first?

**Method:** Correlate camera `0x12A`, host-returned `0x12A`, MDPS `0xEA`, CCNC `0x162`, openpilot lateral-active state, speed, angle, driver torque, and output torque on a common monotonic timeline.

**Representative event:**

| Time from capture start | Event |
|---:|---|
| 20.898-20.899 s | MDPS sets `MDPS_LkaToiFltSta=1` and `MDPS_LkaFailSta=1` |
| about 21.008 s | Camera drops its active `ActToiSta` request |
| about 21.024 s | Exact inactive camera content reaches the EPS-side output |
| about 21.055 s | CCNC `FAULT_LSS` / `FAULT_LFA` / `FAULT_DAS` become visible |

**Result:** MDPS reports failure roughly 100 ms before the camera withdraws steering and before CCNC displays the faults.

**Conclusion:** `0x162` is a downstream report, not the origin of the failure. Hiding it cannot fix the MDPS rejection.

### 10. Test angle, driver torque, and requested torque magnitude

**Question:** Is this the known high-angle steering fault or driver override?

**Method:** Inspect the physical state at multiple failures.

**Results:**

- One exact-stock-stream failure occurred after about 7 seconds at approximately -3.7 degrees steering angle.
- Driver torque was about 44 raw units.
- MDPS output torque was about -1.7 Nm.
- Other failures occurred with modest requested torque and near-center steering.

**Conclusion:** These are not explained by the usual sustained high-angle fault prevention, high torque, or strong driver opposition.

### 11. Block MDPS `0xEA` toward the camera

**Question:** Is the camera causing the later CCNC faults after seeing MDPS status?

**Method:** Temporarily prevent the MDPS status message from reaching the camera.

**Result:** The camera immediately produced LSS/LFA failures consistent with a missing-status timeout.

**Conclusion:** `0xEA` is a required handshake/status input. Blocking it is not a viable solution and was reverted.

### 12. Replay exact stock content for an extended period

**Question:** Is modified openpilot torque itself what MDPS rejects?

**Method:** Keep openpilot lateral control inactive and relay exact byte-for-byte stock camera `LFA` through the application path while the stock camera is actively steering.

**Results:**

- One run failed after about 7 seconds of exact active stock content.
- Another exact-stock run lasted about 26 seconds before failing.
- MDPS failed before the later camera and CCNC reactions.

**Conclusion:** Openpilot torque content is not required to trigger the failure. Application-level transport of exact stock content is enough.

This is the experiment that most strongly moved the investigation from a payload hypothesis to a physical stream/timing hypothesis.

### 13. Measure application-relay cadence

**Question:** Does the host reproduce a physical 100 Hz stream even when every source counter is retained?

**Method:** Measure source camera receipt and host-returned TX timing, counters, batch boundaries, and source-to-host age.

**Representative 6.975-second window:**

- 694 host TX frames
- zero logical counter discontinuities
- 44 back-to-back TX pairs
- 86 TX gaps greater than 15 ms
- maximum host TX gap of 29.2 ms
- camera receipt was also delivered to the host in batches: 41 back-to-back pairs, 88 gaps, maximum 29.8 ms

`card` placed every accumulated `CS.lfa_msgs` entry into one `sendcan` list. This preserved logical order but delivered multiple frames together after a gap.

**Conclusion:** USB/parser/application batching changes the physical inter-frame cadence even when the counter sequence is perfect.

### 14. Emit only one latest camera frame per `card` iteration

**Question:** Would application pacing remove bursts and satisfy MDPS?

**Method:** Stop sending the full batch and send only the newest source message once per `card` loop.

**Results:**

- Multi-frame application bursts were removed.
- Only about 460-482 output frames were emitted per 500 camera frames, approximately 92-96 Hz.
- Roughly 4-8% of source counters were dropped.
- MDPS faults persisted.

Representative post-change events:

- MDPS failure at 17.146 s, followed by `0x162` at 17.279 s.
- Exact-stock lateral-inactive window became active at 20.976 s, MDPS failed at 21.345 s, and visible faults followed at about 21.446 s.
- A later openpilot interval began at 22.549 s and failed at 30.030 s.

**Conclusion:** The application loop cannot simultaneously avoid bursts and retain every 100 Hz camera frame. This experiment is not the fix and should not be retained as the final architecture.

### 15. Process and lag audit

**Question:** Are repeated openpilot restarts leaking driving processes and invalidating timing results?

**Method:** Inspect process trees/groups, CPU/memory/swap, settle time after restart, and route-level `carState.cumLagMs`.

**Process result:**

- Exactly one current manager stack was running: one parent and one manager process group.
- No old `controlsd`, `card`, `modeld`, or other driving stacks were found.
- Long-running `manage_athenad`/`athenad` and `manage_sunnylinkd`/`sunnylinkd` groups were intentional boot services, not leaked openpilot instances.
- The initial high load was dominated by normal restart work such as updater/model/UI/Panda initialization plus active log analysis.
- After settling, load was approximately 1.04 / 3.40 / 6.54, instantaneous CPU idle was 81-88%, about 379 MB was immediately free with roughly 1.7 GB cache, and no swap was in use.

**Loop timing result:** The loop does accumulate upstream scheduling phase drift relative to an ideal 100 Hz schedule. That issue is being treated only as a validity constraint here, not as something to fix in this branch.

Route `000000e8--195fddc347` after restart:

| Segment | Samples | Median `cumLagMs` | p95 | Max |
|---:|---:|---:|---:|---:|
| 0 | 5,480 | -3.555 | -1.508 | 4.690 |
| 1 | - | 0.303 | 4.301 | 8.788 |
| 2 | - | 7.164 | 8.771 | 14.872 |
| 3 | - | 7.913 | 9.131 | 13.590 |
| 4 | - | 8.238 | 9.424 | 16.931 |

Longer route `000000e7--8d34ee15ab`:

| Segment | Median `cumLagMs` | Max `cumLagMs` |
|---:|---:|---:|
| 0 | 3.97 | 16.51 |
| 1 | 12.26 | 19.44 |
| 2 | 13.31 | 19.53 |
| 3 | 14.59 | 21.22 |
| 4 | 27.27 | 46.40 |
| 5 | 46.67 | 56.75 |
| 6 | 55.96 | 70.51 |
| 7 | 68.24 | 81.28 |
| 8 | 80.87 | 89.28 |
| 9 | 85.88 | 95.10 |
| 10 | 95.75 | 104.76 |
| 11 | 108.41 | 115.07 |
| 12 | 111.67 | 119.74 |
| 13 | 114.00 | 120.07 |
| 14 | 114.93 | 131.01 |
| 15 | 129.39 | 143.67 |
| 16 | 138.84 | 149.70 |
| 17 | 148.25 | 156.12 |
| 18 | 153.01 | 160.31 |
| 19 | 155.43 | 161.99 |

Every listed segment had zero CAN-error delta.

`cumLagMs` is accumulated phase error, not proof that every outgoing frame was 155 ms old. Direct source-to-host output age remained roughly 2-20 ms because `card` drained current packets. The important implication is that a slightly-below-100-Hz host loop cannot preserve a strict, lossless 100 Hz physical stream indefinitely. The Panda-timed design removes that dependency from the steering stream without changing the upstream scheduler.

## Evidence hierarchy and present conclusion

### Direct observations

- The fault requires the active moving steering path; no-lane inactive requests can remain clean.
- All 128 LFA bits can be reconstructed exactly.
- A correct camera counter and checksum do not prevent eventual faults through the host path.
- Exact stock content, with no openpilot torque substitution, can fault after variable delays.
- MDPS `0xEA` fault/fail states rise before the camera withdraws steering and before CCNC `0x162` fault fields rise.
- Faults occurred at small steering angle, modest driver torque, and modest MDPS output torque.
- Application replay produces bursts/gaps; single-frame application pacing drops counters.
- No duplicate openpilot driving process stack was present during the audited runs.

### Strong inference

This MDPS enforces a continuity, freshness, timing, or source-stream rule that is violated when the camera's 100 Hz LFA traffic is terminated and recreated through the host application path. The exact internal timer or acceptance rule is not yet known, so this should not be stated as a proven specific timeout value.

### Responsibility assessment

This is best described as an openpilot integration assumption exposed by a newer Hyundai topology, not simply a software bug on one side:

- Hyundai is allowed to implement a stricter MDPS validity monitor on a safety-critical direct camera-to-MDPS stream.
- Existing openpilot LFA steering assumes that generating a valid-looking 100 Hz steering message from the host is sufficient.
- That assumption works on other architectures, including HDA2, but does not preserve the source timing semantics on this CCNC/non-HDA2 path.

The root-cause fix belongs in the integration: retain the OEM producer and physical cadence, and make only a safety-bounded in-flight substitution.

## Panda-timed transformer design

### Internal command

Openpilot sends an 8-byte standard-ID command at `0x7FF` on logical bus 0. It is an internal Panda control message, not a vehicle message.

| Byte(s) | Meaning |
|---:|---|
| 0..1 | Signed little-endian requested torque |
| 2 | Mode |
| 3 | Magic value `0xA5` |
| 4..7 | Must be zero |

Modes:

| Mode | Behavior |
|---:|---|
| 0 | Stock pass-through |
| 1 | Substitute torque only while the camera already has `ActToiSta=1`; request remains active |
| 2 | Same lane-gated substitution, but set request inactive for the permitted fault-avoidance cut |
| 3 | Force steering ownership even when the camera is inactive; request active; road use not yet enabled |
| 4 | Force-ownership request cut; road use not yet enabled |

Panda safety consumes a valid command internally. It does not enter a CAN TX queue, does not increment the blocked counter, and can never collide with a real `0x7FF` on the vehicle bus. An invalid command is rejected normally and immediately clears the cached transform mode.

### Safety checks retained

The command passes the existing Hyundai CAN-FD steering limits:

- maximum torque: 270
- maximum rate up: 2 per accepted command
- maximum rate down: 3 per accepted command
- maximum real-time delta: 112
- driver torque allowance: 250
- driver torque multiplier: 2
- minimum valid request frames before a cut: 89
- maximum consecutive request-cut frames: 2
- minimum request-cut interval: 810 ms

The transformer also requires controls to still be allowed at the instant the physical camera frame is forwarded. A zero-torque request command can pass the generic safety hook while controls are off, matching existing torque-safety behavior, but the forwarding modifier will not apply it.

### Forwarding behavior

For each physical bus-2 `0x12A` frame:

1. Verify address, bus, length, and source Hyundai checksum.
2. Check that the last internal command is newer than 50 ms.
3. Check that lateral controls are still allowed.
4. In lane-gated modes, require source `ActToiSta=1`.
5. Replace only torque and request fields.
6. Recompute the in-payload Hyundai CRC16.
7. Recompute Panda's wrapper checksum.
8. Forward immediately to bus 0.

If any condition fails, the unmodified source frame is forwarded and the command cache is cleared. The torque-safety baseline is synchronized to the stock torque that actually reached MDPS, so the next openpilot command must ramp from the real current value.

The reverse bus-0-to-bus-2 copy of `0x12A` is blocked. The physical camera bus-2-to-bus-0 copy is allowed. Host transmission of a physical `0x12A` is removed from this safety configuration, eliminating duplicates by construction.

### Why holding a command across physical frames is safe

The host may update slightly below 100 Hz. Panda can therefore apply the same already-checked torque to two adjacent physical camera frames. That cannot increase the torque rate: repeating a value is a zero delta. New torque values are applied only after the existing safety hook accepts them. If host updates stop, the 50 ms freshness limit returns the system to stock pass-through.

## Current automated validation

Completed locally before road deployment:

| Validation | Result |
|---|---|
| Focused camera-sync safety class | 70 passed, 4 skipped, 71 subtests passed |
| Full Hyundai CAN-FD safety file | 1,900 passed, 447 skipped, 1,868 subtests passed |
| Hyundai platform tests | 14 passed, 2 skipped, 393 subtests passed |
| Panda internal-command queue tests | 2 passed |
| Panda H7 firmware, bootstub, body, jungle, and host library build | Passed with `-Werror` |
| MISRA cppcheck safety pass | Passed, 268 of 386 available checkers active |
| Ruff on touched Python files | Passed |

The focused tests cover:

- byte-for-byte raw pass-through, including all `NEW_SIGNAL_*` fields;
- counter preservation;
- mutation of only torque/request/checksum;
- recomputed CRC equality with a freshly packed expected frame;
- lane-gated refusal to force an inactive camera request;
- clearing a cached command after any stock fallback;
- force-mode behavior in the safety harness;
- stale, disengaged, malformed, and rate-unsafe fallback;
- forward direction and reverse blocking;
- consumption of a valid internal command without CAN transmission;
- rejection of an invalid internal command without physical transmission.

### Parked device deployment on 2026-08-03

The matching opendbc and Panda sources were copied to the comma four and verified against the local files with checksum-based dry-run comparison before restart. The device-side SCons build completed successfully.

Pandad observed the prior firmware signature `0a63394a181a4ece`, expected the newly built signature `308aec12706c7e95`, flashed Panda, reset it, and the C++ pandad process reconnected successfully. After restart:

- one manager stack was running;
- Panda reported type `cuatro`, no firmware faults, no RX/TX buffer overflows, no blocked safety transmissions, and no invalid safety RX messages;
- source files still matched the local files after the restart;
- ignition line and ignition CAN were both false, so Panda correctly remained in `noOutput` with safety parameter 0.

The vehicle was powered down for this parked deployment. Consequently, active safety parameter 1034, live 100 Hz LFA forwarding, and source/destination byte comparison remain Stage 0 checks for the next ignition cycle. The deploy proves build/flash/reconnect health only; it is not a steering result.

The newly started updater process reported that the existing overlay updater already owned its lock and exited. It was not a second manager, `card`, `controlsd`, or other driving stack and does not change the steering conclusion.

### Stage 0 and Stage 1 live validation on 2026-08-03

The vehicle was powered and driven after the parked deployment. Live state confirmed:

- Panda type `cuatro` running `hyundaiCanfd` safety with parameter 1034;
- ignition line active;
- one manager and driving-process stack;
- no Panda firmware fault, RX buffer overflow, TX buffer overflow, or invalid safety RX state;
- valid vehicle CAN and no temporary or permanent steering fault in `carState`;
- no attempt was made to change or diagnose upstream loop lag beyond checking that duplicate driving stacks were absent.

Three moving captures were taken without blocking or rewriting `0xEA` or `0x162`:

| Capture | Physical LFA result | Steering substitution | Fault result |
|---|---|---|---|
| Initial 15 seconds | 1,498 camera inputs and 1,498 bus-0 outputs; zero missing/extra frames and zero counter discontinuities | 829 byte-identical frames and 669 modified frames; every difference confined to torque, request, and checksum | All 300 sampled `0x162` source messages had `FAULT_LSS=0`, `FAULT_LFA=0`, and `FAULT_DAS=0`; `carState` remained fault-free |
| Continuous 90 seconds | 8,988 inputs and 8,988 outputs; zero source or output counter discontinuities | Source and output request state matched for 6,069 active and 2,919 inactive frames | All 9,002 MDPS samples had fault/fail `(0, 0)`; all 1,797 CCNC samples had fault tuple `(0, 0, 0)` |
| Corrected 20-second handoff capture | 1,995 inputs paired with all 1,995 outputs at zero sequence offset; zero counter discontinuities | Two request-state transitions; 218 exact frames and 1,777 modified frames; zero unexpected changed bits | All 2,001 MDPS samples had fault/fail `(0, 0)`; all 398 CCNC samples had fault tuple `(0, 0, 0)` |

The first streaming implementation of the 90-second diagnostic paired a returned output only when its source copy had already appeared in host event order. Panda can report the returned output to the host before the source copy in the same cycle, which left 256 frames offset by one counter wrap and produced false `unexpected_diff` counts in that diagnostic. This was an observer-order bug, not a bus error. The follow-up capture stored both complete sequences, aligned them by counter occurrence, paired every frame at offset zero, and found zero unexpected differences.

No physical `0x7FF` internal command appeared during the initial capture. The physical source and output torque ranges differed while substitution was active, proving this was not merely stock forwarding, while request states and all non-steering payload bits remained camera-owned.

The Panda-wide `safetyTxBlocked` counter was 3 before the requested handoff exercise and 5 afterward. Because the counter is global and the live capture did not record the rejected address, the two increments cannot yet be assigned conclusively to the virtual steering command. Their timing is consistent with a safety-baseline boundary at the two handoffs, but another host command remains possible. In either case, the rejection path preserved the physical source frame, did not create a counter hole, and did not trigger MDPS or CCNC faults. The next instrumentation should identify the rejected address and reason rather than infer it from the aggregate counter.

Initial Stage 1 result: the Panda interrupt-time transformer resolves the fault condition that consistently affected application-layer replay. It has now remained clean beyond every previously observed failure interval and across two request transitions. This is strong single-vehicle evidence for the physical-cadence root cause, not yet a production-readiness claim. Longer-duration repetition, explicit stock LKA lane-departure testing, attribution of the two blocked transmissions, and force-ownership testing remain open.

The proven checkpoint was committed as:

- opendbc `296c67ab` (`hyundai: sync CCNC LFA steering in Panda`);
- Panda `9af46280` (`can: support safety-modified forwarded frames`);
- Sunnypilot parent `aa4663ba33` (`hyundai: checkpoint CCNC LFA camera sync`).

## Road-validation plan

### Stage 0: parked deployment and fail-safe checks

1. Warn the driver before restarting openpilot or flashing Panda.
2. Deploy both opendbc safety changes and the matching Panda driver changes together.
3. Confirm platform fingerprint `HYUNDAI_SONATA_HEV_2024` and safety parameter 1034.
4. Confirm Panda firmware signature matches the freshly built source.
5. Confirm only one manager/driving stack is running. This is a validity check, not a lag-fix effort.
6. While disengaged, compare bus-2 camera `0x12A` and bus-0 returned/forwarded `0x12A`:
   - identical payload;
   - identical counter;
   - one output per source frame;
   - no host-generated physical `0x12A`;
   - zero Panda safety rejections from valid internal mode-0 commands.
7. Stop or restart `card` once and confirm camera LFA immediately remains or returns to raw pass-through.

### Stage 1: moving, lane-gated openpilot substitution

Status: initial validation passed on 2026-08-03; longer repetition remains required. Force modes were disabled for these captures.

1. Begin on a straight, low-complexity road with stable lane lines and room for a safe manual takeover.
2. Record camera `0x12A`, forwarded/returned bus-0 `0x12A`, MDPS `0xEA`, CCNC `0x162`, `carControl`, `carState`, Panda health, speed, angle, and driver torque.
3. Allow stock LFA to become active first and verify MDPS acknowledgement.
4. Engage openpilot for at least 60 seconds, not only the previously misleading 4-30-second clean window.
5. Disengage, wait, and re-engage several times while moving.
6. Exercise stock LKA lane-departure behavior separately.
7. Include a longer continuous interval to exceed the 7-second and 26-second exact-replay failure examples.

Acceptance criteria:

- exactly one bus-0 `0x12A` for every physical camera counter;
- no duplicate or missing physical counters;
- no application-shaped 20-30 ms holes followed by bursts in the EPS-side stream;
- only checksum, torque bits, and request bits differ during substitution;
- all other 99 bits match the source frame;
- zero `MDPS_LkaToiFltSta` and `MDPS_LkaFailSta` transitions;
- no `FAULT_LSS`, `FAULT_LFA`, or `FAULT_DAS` on `0x162`;
- no valid internal command appears on a physical CAN TX queue;
- no Panda safety rejection during normal bounded torque updates;
- stale/disengaged behavior visibly returns to exact source bytes within at most 50 ms.

Failure protocol:

1. Do not suppress the warning.
2. Disengage and retain the complete pre-fault and post-fault timeline.
3. Identify the first nonzero state among MDPS fault/fail, camera request drop, and CCNC faults.
4. Compare the last 100 physical source and destination frames for counter, payload-mask, checksum, and inter-arrival time.
5. Record the age and mode of the last accepted internal command.
6. Revert to mode 0/raw pass-through if any transform invariant fails.

### Stage 2: steering without lane lines

Steering without lane lines is technically possible from openpilot's planning/control perspective. The remaining vehicle question is whether this MDPS accepts `ActToiSta=1` when the camera itself is not asserting ownership, and whether other LFA state bits form an additional ownership handshake.

Only after Stage 1 is clean:

1. Enable force mode in the Python sender while leaving all `0x162` diagnostics intact.
2. Begin with zero requested torque and confirm whether MDPS acknowledges the forced request.
3. Ramp by the normal 2-count safety limit to a very small torque.
4. Observe `MDPS_LkaToiActvSta`, `MDPS_LkaToiUnblSta`, `MDPS_LkaToiFltSta`, and `MDPS_LkaFailSta` before increasing duration or torque.
5. Test force-mode request cuts after the 89-frame/810-ms safety prerequisites.
6. If MDPS rejects force ownership while lane-gated substitution is clean, compare every source state field around stock activation. The missing requirement would then be an ownership/state handshake, not the original stream-cadence problem.

The code supports modes 3 and 4 now, but `CarController` deliberately sends only mode 0, 1, or 2 in the first deployment.

## Open questions

1. What exact MDPS monitor triggers the failure: maximum inter-frame gap, source-clock jitter, duplicate arrival, missing counter deadline, bus-level transmitter identity assumptions, or a combination?
2. Does the Panda-timed transform remain clean across many minutes and repeated moving handoffs?
3. Does CCNC/non-HDA2 force ownership require changing any field besides `StrTqReqVal` and `ActToiSta` when the camera has no lane solution?
4. Does MDPS expect a particular relationship between camera state and its own `MDPS_LkaToiActvSta` transition?
5. Are other recent CCNC non-HDA2 Hyundai/Kia/Genesis platforms subject to the same strict stream behavior?
6. Can a Panda-side diagnostic counter be added later without changing the safety boundary, so raw, transformed, stale-fallback, and invalid-source frames are observable directly?

## How others can contribute

Useful independent evidence should include all of the following, not only a dashboard video:

- exact vehicle/platform and whether it has HDA2;
- harness topology and Panda bus mapping;
- raw camera and EPS-side `0x12A` with timestamps and returned/rejected flags;
- `0xEA` MDPS status and `0x162` CCNC faults;
- whether source `ActToiSta` was active;
- requested torque, steering angle, speed, driver torque, and MDPS output torque;
- source/destination counter continuity and inter-arrival distributions;
- whether the test was raw forwarding, exact host replay, host-generated steering, or Panda-timed transform;
- duration of the clean interval before declaring success.

Short clean windows should be labeled as such. A result should not be called fixed until it exceeds the prior 7-second, 26-second, and 30-plus-second misleading windows and survives repeated handoffs.

## Current implementation files

opendbc:

- `opendbc/car/hyundai/carcontroller.py`
- `opendbc/car/hyundai/hyundaicanfd.py`
- `opendbc/car/hyundai/interface.py`
- `opendbc/car/hyundai/values.py`
- `opendbc/safety/declarations.h`
- `opendbc/safety/safety.h`
- `opendbc/safety/modes/hyundai_canfd.h`
- `opendbc/safety/modes/hyundai_common.h`
- `opendbc/safety/tests/test_hyundai_canfd.py`
- `opendbc/safety/tests/libsafety/libsafety_py.py`
- `opendbc/safety/tests/misra/main.c`
- `opendbc/car/hyundai/tests/test_hyundai.py`

Panda:

- `board/drivers/can_common.h`
- `board/drivers/fdcan.h`
- `tests/libpanda/libpanda_py.py`
- `tests/libpanda/test_internal_can_command.py`

## Bottom line

The evidence no longer supports treating this as a mysterious checksum or undefined-bit problem. The MDPS rejects even exact stock content when openpilot recreates the stream through the application path. The most defensible next experiment is to preserve the camera's original physical 100 Hz frames and timing inside Panda, substitute only safety-checked steering bits, and leave every fault channel visible. If that remains clean, it solves the underlying integration problem rather than hiding its consequences.
