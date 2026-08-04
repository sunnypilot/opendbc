# CCNC non-HDA2 steering-fault investigation

Status: active research; Panda-timed transformer, targeted camera-side MDPS acknowledgement handling, and dynamic pass-through handoff passed sustained no-line steering and 20 natural lane-state transitions with every observed MDPS and `0x162` fault field clear
Vehicle: 2024 Hyundai Sonata Hybrid without HDA II (`HYUNDAI_SONATA_HEV_2024`)
Branch: `ccnc-port-lite`
Last updated: 2026-08-04

## Executive summary

The steering fault is not explained by missing DBC bits, an incorrect LFA counter, an incorrect LFA checksum, excessive steering angle, driver torque, or openpilot torque content alone.

The strongest evidence points to the way the non-HDA2 camera's 100 Hz `LFA` stream reaches the MDPS/EPS. On this CCNC car the front camera is the original LFA producer and the MDPS appears to require that physical stream to remain continuous and tightly timed. Relaying the camera message through `card`/USB, even byte-for-byte, creates bursts, gaps, or dropped counters. Exact stock content relayed through the application layer eventually makes the MDPS enter its fail state. The MDPS failure occurs before the camera drops its request and before CCNC reports the visible `0x162` faults.

The next design therefore does not generate or replay `LFA` in openpilot. Panda forwards each real camera frame at hardware timing and optionally changes only:

- `StrTqReqVal` (bits 41..51)
- `ActToiSta` (bits 52..53)
- the 16-bit Hyundai checksum (bits 0..15)

The physical camera counter, every other defined signal, every currently unknown bit, and the physical arrival cadence remain camera-owned. A small safety-checked command from openpilot is consumed inside Panda and never transmitted on CAN. Invalid, stale, disengaged, or unsafe commands revert immediately to unmodified stock pass-through. Nothing suppresses or edits the CCNC fault message `0x162`.

The lane-gated implementation subsequently passed an initial moving road validation: a continuous 90-second window contained 8,988 source frames and 8,988 physical outputs, no counter discontinuities, 9,002 healthy MDPS samples, and 1,797 clear CCNC fault samples. A separate corrected 20-second byte-pairing capture included two request handoffs and showed 218 exact frames, 1,777 modified frames, and zero changes outside torque, request, and checksum. This exceeds the prior 7-second, 26-second, and 30-plus-second false-clean windows.

The disabled-by-default force-ownership mode was subsequently tested. It proved that the EPS can steer without lane lines: MDPS accepted the forced request and never set its unable, fault, or fail fields, even with requested torque spanning -270 through 270. CCNC nevertheless raised LSS/DAS and sometimes LFA faults immediately whenever the camera itself was lane-inactive. Recognition-only and recognition-plus-cached-damping variants failed the same way. These results separate two problems:

- the original application-relay failure was an MDPS timing/continuity failure, solved by Panda-timed transformation;
- forced no-line ownership is a CCNC cross-message consistency failure, not an EPS torque limitation.

Later isolation disproved lane geometry as the rejecting condition. Fabricated `0x161`, `0x1E0`, and `0x1B5` state did not prevent the fault. Preserving all of `0xEA` except the two-bit camera-facing `MDPS_LkaToiActvSta` acknowledgement did: the real MDPS continued to report honestly on the car side, while the camera no longer saw an acknowledgement that contradicted its own inactive/no-line request. Dynamic ownership forwards the authentic acknowledgement whenever the physical camera is active and hides only that active bit while openpilot owns an otherwise inactive camera frame. A 163.808-second moving validation crossed 20 physical camera state changes, reached openpilot torque +/-270, and kept every sampled MDPS and `0x162` fault field clear.

The global Panda blocked-transmission counter still showed isolated increments at four active-to-inactive handoffs, but never a rejection cascade, missing physical frame, MDPS fault, or `0x162` fault. Exact rejected-address attribution remains instrumentation work rather than evidence of a vehicle fault.

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

Panda-timed transform (validated for lane-gated steering)

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

#### Complete observed `LFA` field inventory

The added definitions cover 128 of 128 payload bits. A decode-and-repack sweep over 8,434 synchronized stock frames reproduced all 8,434 payloads exactly; a separate live sample also reproduced all 16 bytes exactly. This validates bit position, width, endianness, scaling, and preservation for the states exercised. It does not prove that every provisional signal name is semantically correct.

The following inventory combines the 50,851-frame low/local-road capture with the earlier higher-speed reference. `Boundary` means the value appeared briefly while recognition and torque-overlay states changed at slightly different instants.

| Signal | Bits / endian | Observed stock value(s) | Current interpretation |
|---|---|---|---|
| `CHECKSUM` | 0:16 LE | dynamic 16-bit value | Validated Hyundai CAN-FD CRC |
| `COUNTER` | 16:8 LE | every value 0..255 | Camera-owned 100 Hz rolling counter |
| `LKA_OptUsmSta` | 24:3 LE | 0 | Constant in tested configuration |
| `LKA_RcgSta` | 27:3 LE | 0 or 3 | Lane recognition; normally 3 during active steering, with boundary overlap |
| `LKA_LHLnWrnSta` | 30:2 LE | 0 | No left lane warning in captures |
| `LKA_RHLnWrnSta` | 32:2 LE | 0 | No right lane warning in captures |
| `LKA_HndsoffSnd` | 34:2 LE | 0 | No hands-off sound in captures |
| `LKA_StrSnd` | 36:2 LE | 0 | No steering sound in captures |
| `LKA_SysIndReq` | 38:3 LE | 0 | Constant in captures |
| `StrTqReqVal` | 41:11 LE, offset -1024 | 0 inactive; -232..209 active | Camera torque request |
| `ActToiSta` | 52:2 LE | 0 inactive, 1 active | EPS torque-overlay request |
| `ToiFltSta` | 54:2 LE | 0 | Camera-side overlay fault remained clear |
| `LFA_BUTTON` | 56:1 BE | 0 | Not the physical steering-wheel LFA button on this car |
| `BCA_Rear_WrnSta` | 57:3 LE | 0 | Constant in captures |
| `LKA_SysWrn` | 60:4 LE | 0 | LKA warning remained clear |
| `FCA_LO_WrnSta` | 64:1 LE | 0 | Constant in captures |
| `NEW_SIGNAL_1` | 67:3 BE | 0 | Structurally validated; semantics unknown |
| `FCA_LS_WrnSta` | 68:1 LE | 0 | Constant in captures |
| `NEW_SIGNAL_2` | 71:3 BE | 0 | Structurally validated; semantics unknown |
| `LKA_OnOffEquip2Sta` | 72:2 LE | 1 | Equipment/configuration state |
| `NEW_SIGNAL_3` | 79:6 BE | 2 | Structurally validated; semantics unknown |
| `LKA_UsmMod` | 80:2 LE | 0 | Constant in captures |
| `Info_PedtrnDst` | 84:3 BE | 0 | Constant in captures |
| `ELK_SysFlrSta` | 85:5 LE | 0 | ELK fault remained clear |
| `ELK_SymbDisp` | 90:3 LE | 0 | Constant in captures |
| `FCA_ESA_WrnSta` | 93:1 LE | 0 | Constant in captures |
| `NEW_SIGNAL_4` | 95:2 BE | 0 | Structurally validated; semantics unknown |
| `NEW_SIGNAL_6` | 100:5 BE | 0 | Structurally validated; semantics unknown |
| `FCA_ESA_CtrlSta` | 101:1 LE | 0 | Constant in captures |
| `NEW_SIGNAL_5` | 103:2 BE | 0 | Structurally validated; semantics unknown |
| `Damping_Gain` | 104:8 LE | 100 inactive; 10..82 active | Predominantly a speed-indexed EPS damping parameter |
| `NEW_SIGNAL_7` | 119:8 BE | 0 | Structurally validated; semantics unknown |
| `NEW_SIGNAL_8` | 127:8 BE | 0 | Structurally validated; semantics unknown |

The physical LFA button was independently found on `CRUISE_BUTTONS` (`0x1CF`) as `LDA_BTN` at bit 23. Its edge emits `ButtonType.lkas` and toggles sunnypilot's lateral-only `latActive` state. This explains why the main UI/control state can report `enabled=false` while the vehicle and `carControl.latActive` still permit lateral control; it is a MADS state distinction, not an LFA DBC mismatch.

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

### Complete camera-segment inventory

A 448-second raw capture found exactly 11 recurring addresses on Panda bus 2. The capture is exhaustive for that interval, not a claim that no optional message can ever appear.

| Address | Rate | Current DBC name | Frames | Defined-bit result |
|---:|---:|---|---:|---|
| `0x11A` | 100 Hz | `FR_CMR_01_10ms` | 44,868 | 96/128 bits defined; live undefined bits 72 and 74 were always set (`byte 9 = 0x05`) |
| `0x12A` | 100 Hz | `LFA` | 44,868 | 128/128 bits defined; no payload bit outside the DBC |
| `0x160` | 50 Hz | `ADRV_0x160` | 22,434 | 58/128 bits defined; live undefined bits 99 and 116 were set |
| `0x1A0` | 50 Hz | `SCC_CONTROL` | 22,434 | 247/256 bits defined; all undefined bits were zero |
| `0x161` | 20 Hz | `CCNC_0x161` | 8,974 | 256/256 bits defined; exact decode/repack sample |
| `0x200` | 20 Hz | `ADRV_0x200` | 8,974 | 40/64 bits defined; live undefined bits 41, 43, and 44 formed the constant missing value in byte 5 |
| `0x1B5` | 20 Hz | `FR_CMR_03_50ms` | 8,974 | 178/256 bits defined; all undefined bits were zero; exact decode/repack sample |
| `0x162` | 20 Hz | `CCNC_0x162` | 8,974 | 256/256 bits defined; exact decode/repack sample |
| `0x1E0` | 20 Hz | `LFAHDA_CLUSTER` | 8,973 | 34/128 bits defined; all undefined bits were zero; exact decode/repack sample |
| `0x1FA` | 10 Hz | `FR_CMR_02_100ms` | 4,487 | 146/256 bits defined; live undefined bits 171 and 173 were set (`byte 21 = 0x28`) |
| `0x38C` | 5 Hz | not defined | 2,244 | 32-byte frame: CRC in bytes 0..1, gap-free byte-2 counter, constant bytes 3..4=`f3 0f`, remaining bytes zero |

The DBC sender labels are not proof of physical origin on this topology; this table describes what arrived from the isolated bus-2 side. In particular, `0x161` and `0x162` are named `CCNC_*` in the DBC but are present on the camera-side segment.

Natural `ActToiSta` activation had near-deterministic companions:

| Message | Inactive state | Natural active state | Evidence |
|---|---|---|---|
| `0x12A` | `ActToiSta=0`, torque 0, damping 100 | `ActToiSta=1`, nonzero-capable torque, active damping | Direct steering command |
| `0x161` | `CENTERLINE=0`, `LANELINE_LEFT=0`, `LANELINE_RIGHT=0`, `LFA_ICON=1` | `CENTERLINE=1`, both lane lines=2, `LFA_ICON=2` | 2,807 active vs 6,167 inactive aligned samples; 97-100% separation outside transition boundaries |
| `0x1B5` | lane-quality/geometry fields invalid or degraded | both lane qualities normally 3 with live geometry | Camera lane model; transitions can lead/lag torque-overlay state |
| `0x1E0` | `LFA_ICON=1` | `LFA_ICON=2` | 2,808 active vs 6,165 inactive aligned samples; approximately 98-100% separation |

`0x162` showed no steering-ownership field in the clean data. Its small speed-limit correlations followed the road segment, while all LSS/LFA/DAS fault fields stayed zero. `0x11A`, `0x160`, `0x1A0`, `0x1FA`, `0x200`, and `0x38C` did not expose a comparably deterministic steering-activation bit after accounting for route/time confounding.

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

### 16. Validate Panda-timed lane-gated openpilot torque

**Question:** Does substituting openpilot torque inside each physical camera frame avoid the original active-steering fault without suppressing diagnostics?

**Method:** Deploy the interrupt-time transformer, leave force modes disabled, and compare every camera input to the physical bus-0 output while monitoring MDPS `0xEA` and CCNC `0x162`.

**Results:**

- 15-second capture: 1,498 inputs / 1,498 outputs, 829 exact and 669 modified, no missing or extra frames, and no changes outside checksum/torque/request.
- 90-second capture: 8,988 / 8,988, zero counter gaps, 6,069 active and 2,919 inactive requests, 9,002 healthy MDPS samples, and 1,797 clear CCNC samples.
- corrected 20-second handoff capture: 1,995 / 1,995 at zero sequence offset, 218 exact and 1,777 modified, two request transitions, zero unexpected bit changes, 2,001 healthy MDPS samples, and 398 clear CCNC samples.

**Conclusion:** The Panda-timed transformer fixes the original MDPS stream-integrity failure for lane-gated steering on this vehicle. It does so without changing `0x162` or hiding any MDPS status.

### 17. Force `0x12A` steering ownership without camera lanes

**Question:** Can the EPS steer when the camera has no lane solution if Panda forces only the `0x12A` request and openpilot torque?

**Method:** Enable the disabled force mode. Preserve the physical source frame/counter/cadence, replace torque and `ActToiSta`, recompute checksum, and keep every fault channel visible.

**75-second result:**

- 7,479 inputs and 7,479 outputs with zero counter gaps and no unexpected changes outside the permitted `0x12A` fields.
- 4,823 frames were forced active while the source camera request was inactive.
- Output torque covered -270 through 270.
- MDPS reported active on 7,203 samples and never set unable, fault, or fail.
- CCNC fault tuples `(FAULT_LSS, FAULT_LFA, FAULT_DAS)` were `(0,0,0)` for 534 samples, `(1,1,0)` for 248, `(1,0,0)` for 250, and `(1,0,1)` for 464.
- The first CCNC fault appeared about 25 ms after forced ownership began. Faults cleared when the camera naturally became active and returned when it became inactive.

**Conclusion:** The EPS is physically capable of steering without lane lines and accepted the request/torque. CCNC rejected the semantic ownership state. This is distinct from the earlier MDPS timing failure.

### 18. Force natural recognition state with zero requested torque

**Question:** Is `LKA_RcgSta=3` the missing CCNC ownership prerequisite?

**Method:** During camera-inactive intervals, force `ActToiSta=1`, `LKA_RcgSta=3`, and zero target torque while preserving every other source field.

**Result:** CCNC faulted at the first transition sample with LSS/DAS `(1,0,1)`. MDPS remained healthy.

**Conclusion:** Recognition state alone is insufficient, and torque magnitude is not required to trigger the CCNC rejection.

### 19. Force recognition plus the last natural active damping

**Question:** Does CCNC require the active damping state as well as recognition?

**Method:** Cache the last damping value observed while the camera was naturally active. During a later inactive interval, force request active, recognition 3, zero target torque, and the cached active damping.

**Result:** The camera's inactive damping was 100; the cached active value at the tested transition was 64. CCNC still faulted immediately with `(1,0,1)`, while MDPS remained healthy. In the 70-second monitor, all 5,656 active MDPS samples were fault-free, while CCNC produced 803 clear, 301 `(1,0,1)`, and 292 `(1,0,0)` samples.

**Conclusion:** `0x12A` recognition and damping exhaust the natural `0x12A` fields that distinguish ordinary active from inactive steering in the observed route, but they do not satisfy CCNC. The missing state is outside `0x12A` or depends on a relationship to MDPS acknowledgement.

### 20. Validate every defined `LFA` bit and study natural transitions

**Question:** Are the newly defined empty bits correct, and does any other `0x12A` signal participate in activation?

**Method:** Decode and repack a 90-second synchronized stock reference, enumerate all signal values, and compare natural active/inactive transitions.

**Results:**

- 8,983 source and 8,983 output frames; zero forced or unexpected changes.
- 8,434 fully decoded source frames repacked byte-for-byte with `roundtrip_bad=0`.
- In a separate 4,299-frame transition set, only `LKA_RcgSta` and `Damping_Gain` changed with activation beyond checksum/counter/torque/`ActToiSta`.
- `LKA_RcgSta` was normally 0 inactive and 3 active, but the larger local-road capture proved boundary overlap: recognition can drop to 0 for roughly 0.1-1.5 seconds while `ActToiSta` remains 1, and one transition briefly showed recognition 3 before activation.
- Every `NEW_SIGNAL_*`, warning, FCA, ELK, pedestrian, and sound field stayed at the values shown in the complete inventory above.

**Conclusion:** The 128-bit mapping is structurally correct for all observed values. `LKA_RcgSta` is a recognition indicator, not a strict torque-ownership prerequisite, and the provisional semantic names for constant unknown fields remain unproven.

### 21. Determine what controls `Damping_Gain`

**Question:** Is damping set only by speed, or also by requested torque, wheel position, steering rate, driver/EPS torque, acceleration, or lane curvature?

**Method:** Synchronize each active camera `0x12A` with `carState`, `carControl`, and the latest `0x1B5` lane geometry. Measure direct correlations, then remove a cubic speed fit and inspect residual correlations so that curves at different speeds do not create false relationships.

**High-speed reference:** Active damping was 74..82. Direct Pearson correlation with speed was 0.9128. The next-largest simple correlations were absolute wheel angle 0.2570, absolute camera torque 0.2496, absolute EPS torque 0.2136, and signed driver torque -0.2334. Acceleration was 0.0824 and steering rate -0.0147. Yaw/curvature fields from `carState` were unpopulated in that integration and were not treated as evidence.

**Local-road reference:** 11,246 active samples covered approximately 0..14.19 m/s and damping 10..35.

- Damping was exactly 10 for every sufficiently populated 0.5 m/s bin from standstill through 5.5 m/s.
- At 10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, and 13.5 m/s, mean damping was respectively 20.23, 22.13, 24.77, 26.58, 28.83, 30.35, 31.86, and 33.97.
- Direct damping/speed correlation was 0.9332.
- A cubic speed-only model left 0.841 damping-count RMS residual, close to one integer count.
- After removing that speed curve, correlations were: acceleration 0.024, absolute wheel angle 0.009, absolute steering rate -0.012, signed requested torque -0.063, absolute requested torque 0.111, absolute driver torque -0.119, absolute EPS torque 0.074, absolute lane curvature 0.123, absolute lane heading 0.140, and lane-width estimate 0.150.

**Conclusion:** Speed is overwhelmingly the dominant input, with a floor of 10 at low speed and a nonlinear rise toward the high-speed 70s/low 80s. The present data does not support the hypothesis that damping materially increases merely because the wheel is farther from center. Small residual lane/torque relationships remain research-only because they are near the quantization/model error and are confounded by route segments and camera filtering.

A final independent 398.75-second capture added 9,525 active samples at 0.16..11.14 m/s and damping 10..24. Its speed-only cubic residual was smaller at 0.661 counts. Residual correlations were acceleration -0.070, absolute wheel angle -0.039, absolute steering rate 0.004, absolute driver torque -0.002, absolute EPS torque 0.051, signed requested torque -0.082, and absolute requested torque 0.087. This second route segment strengthens the speed-dominant conclusion and weakens the apparent wheel-angle/driver-torque effects seen in uncorrected correlations.

**Raw-wheel refinement:** A later 900-second recorder captured the four raw `0xA0` wheel speeds at 100 Hz together with every physical camera `0x12A`, vehicle/control state, and lane geometry. This resolves most of the earlier one-count ambiguity:

- The best stable speed source is the arithmetic mean of all four raw wheel speeds. Individual wheels, front/rear axle averages, and the minimum/maximum wheel were less consistent across routes. Fitted per-wheel weights changed substantially between routes, which identifies those small apparent improvements as turn/slip overfit rather than a credible alternate source. CAN-FD `carState.vEgoCluster` is not populated on this platform, and the camera directly receives `0xA0`; the present evidence therefore supports wheel speed, not displayed cluster speed. An expanded recorder then captured every frame forwarded toward the camera and tested the other highly speed-correlated payloads (`0x45`, `0x1AA`, `0x1C5`, `0x2D3`, `0x340`, `0x3FA`, and `0x435`). Across 107 independent damping transitions, a filtered `0xA0` four-wheel average was best at 0.089 km/h threshold RMS; its fitted scale was 0.9994 and offset -0.02 km/h. The best integer/cluster-like speed copy was worse and did not generalize between halves of the run. This directly rejects cluster speed as the damping source.
- Every one of 92 damping changes in the clean continuous sweep occurred on an even `0x12A` counter. Damping is carried at 100 Hz but recomputed only every second frame, effectively 50 Hz, and held on the intervening frame.
- A first-order filtered four-wheel average explains the acceleration/deceleration lag. Coefficients around `0.09..0.105` at 100 Hz have aligned the independent runs (equivalently about `0.17..0.20` across two frames); the camera-input-side capture favors approximately `0.09`. Thus acceleration affects damping indirectly by changing the lag between raw and filtered speed; no separate acceleration term is supported. The small coefficient spread is now a phase/fixed-point question, not evidence for an acceleration term.
- The inferred static schedule is piecewise linear before integer truncation:

  ```text
  v <= 30:       10
  30 < v <= 40: floor(10 + 1.5 * (v - 30))
  40 < v <= 50: floor(25 +       (v - 40))
  50 < v <= 60: floor(35 + 0.5 * (v - 50))
  60 < v <=100: floor(v - 20)
 100 < v <=108: floor(80 + 0.25 * (v - 100))
  v > 108:       82
  ```

  Here `v` is the filtered four-wheel-average speed in km/h. The middle knots are directly crossed in the new raw-wheel sweep; the 100-108 km/h taper and ceiling remain supported by the earlier highway reference rather than the latest local-road run.
- Repeated clean threshold crossings are approximately 0.06 km/h above the round-number schedule, close to exactly `1/16 km/h`. A fixed-point implementation with knots at 30.0625, 40.0625, 50.0625, and so on is therefore the leading refinement, but it remains a candidate until more repeated up/down crossings distinguish a true fixed offset from sampling overshoot.
- With a `0.105` exploratory filter coefficient and the fixed schedule above, 1,110 of 1,212 even-counter active samples above 30.5 km/h matched exactly; all remaining 102 differed by exactly one damping count. The observed transition speeds were within roughly 0.09 km/h RMS of the schedule's theoretical thresholds. This is strong structural validation, but the filter coefficient/phase and the high-speed taper still require another independent steady-speed/highway pass before the implementation should be called byte-exact.
- The final all-camera-input capture ran 501.09 seconds and contained 50,099 forwarded `0xA0` wheel frames, 49,798 physical `0x12A` frames, 226 usable one-count damping transitions, and active damping 10..53 through 73.9 km/h. Combining its transitions with the prior independent raw-wheel route favors a common coefficient of `0.0986`, effectively `0.10`, with route threshold offsets of 0.0444 and 0.0455 km/h. Using `alpha=0.10` and a 0.045 km/h candidate knot offset predicts 92.4% of 7,074 above-threshold even-counter samples exactly across both routes; every remaining sample is exactly one count away. The exact residual is therefore a fixed-point/update-phase detail. Every decoded `0x162` fault tuple in the final capture remained `(0,0,0)`.
- After conditioning on filtered speed and update phase, requested torque, requested-torque rate, driver torque, EPS torque, steering angle, steering rate, wheel-speed spread, lane center/width, heading, and curvature did not produce a stable cross-route improvement. Small single-route correlations changed with the road segment and were comparable to the remaining one-count quantization error. They remain explicitly rejected as formula inputs unless a controlled repeated-speed test produces a repeatable residual.
- Driver override was checked separately rather than only as a correlation. The highway reference contains a 3.618-second interval above the CAN-FD `STEER_THRESHOLD=250`, a 1.400-second interval, and several shorter intervals. While `ActToiSta` remained active, damping did not step at override entry or release and had no observable post-release hold: during the longest event it moved from 80 to 79 only while speed fell from about 100.3 to 99.2 km/h, then remained 79 after release. A later raw-wheel run exposed the separate stock torque-cut behavior. Four high-driver-torque events with `LKA_RcgSta=3` briefly changed the complete request tuple to `(StrTqReqVal, ActToiSta, Damping_Gain)=(0,0,100)`; three lasted 3 LFA frames (28.8..29.1 ms) and one lasted 6 frames (59.5 ms). Each resumed the normal speed-based damping (10, 14, 24, or 26) while driver torque was still high. Therefore override is not an input to the active damping formula and has no damping decay timer, but it can trigger a short inactive request pulse whose stock inactive damping is 100.

**2026-08-04 extended validation:** A later recorder ran for 1,448.8 seconds. The complete 448 MB device file contains 144,803 raw wheel frames and has SHA-256 `0f72213c80779aad482b5ca4bb0318ccb6de059f689e8fdcae3e2b75d6a5c54d`. It was copied off-device and its local hash independently matched. The initial 245 MB prefix used for the fit contains the entire useful stock-active/high-speed portion: 79,494 wheel frames and 26,152 active even-counter LFA samples. Direct coverage by filtered-speed bracket was:

| Filtered four-wheel speed | Samples | Observed gain | Confidence in static bracket |
|---:|---:|---:|---|
| 0-30 km/h | 5,549 | 10 | Near certain |
| 30-40 km/h | 6,348 | 10..25 | Very high |
| 40-50 km/h | 1,261 | 24..34 | Very high; adjacent boundary samples account for 24 |
| 50-60 km/h | 1,380 | 34..39 | High; every interior step repeated |
| 60-100 km/h | 11,614 | 39..73 | Very high through approximately 93 km/h |
| 100-108 km/h | 0 in this run | Earlier 80..81 | Moderate; earlier highway data only |
| >=108 km/h | 0 in this run | Earlier ceiling 82 | Moderate for exact onset; high that 82 is the observed ceiling |

For the 20,457 samples above 30.25 km/h, a phase-aware unconstrained fit selected `alpha=0.12`, a -0.08 km/h schedule offset, and a three-wheel-frame phase shift: 19,398 samples (94.82%) were exact and all other 1,059 were off by exactly one count. Holding the simpler cross-route candidate at `alpha=0.10` selected a two-frame shift and the same -0.08 km/h offset: 19,390 samples (94.78%) were exact and all remaining 1,067 were off by one. The earlier independent 900-second and 501-second routes prefer approximately the same `alpha=0.10` but different apparent host-side phase/offset combinations. That variation is consistent with observation/batching phase across buses, so a fixed two-frame delay would overfit the recorder rather than prove the camera's internal arithmetic.

The current implementation therefore uses the smallest model supported across routes: a 100 Hz first-order filter with `alpha=0.10`, the piecewise schedule above, positive integer truncation, and a 50 Hz update on even LFA counters. Its static bracket structure is high confidence and every observed prediction error is at most one byte count. The exact fixed-point offset and physical update phase remain medium confidence until Panda-internal timestamps remove host observation phase. No evidence supports adding acceleration, torque, torque rate, driver override, angle, steering rate, lateral position, or lane-geometry terms merely to fit that final one-count residual.

The later capture also corrected an over-broad interpretation of stock inactive pulses: high driver torque can cause them, but short `(0,0,100)` pulses also occurred at low driver torque when the camera/lane state changed. Such pulses describe complete stock request withdrawal, not a separate damping response. They must be excluded from the active damping fit rather than attributed automatically to driver override.

### 22. Inventory and correlate every camera-segment message

**Question:** What state outside `0x12A` distinguishes natural active steering from a forced request?

**Method:** Record every bus-2 frame, align it to 50,851 synchronized `0x12A`/vehicle-state samples, and compare each non-checksum/non-counter bit between natural active and inactive intervals.

**Result:** Only the lane-model/display family gave near-deterministic activation separation:

- `0x161`: `CENTERLINE` 0 to 1; left/right lane-line values 0 to 2; `LFA_ICON` 1 to 2.
- `0x1E0`: `LFA_ICON` 1 to 2.
- `0x1B5`: both lane quality values normally become 3 and geometry becomes valid.
- `0x162`: every monitored fault bit stayed zero; no separate ownership field was found.

**Conclusion:** The strongest current root-cause hypothesis for no-line force faults is a contradiction among camera-owned messages. The next test should synchronize only the confirmed ownership/display fields first, without fabricating geometry or touching fault fields, and observe whether CCNC accepts the request.

Natural transition timing further constrains that test. Across seven captured activations, `0x161` reached its active tuple between 9.6 ms before and 37.9 ms after the `0x12A` activation; `0x1E0` changed between 19.0 ms before and 44.3 ms after. `0x1B5` quality was already valid or changing within the same 20 Hz interval. Deactivation was intentionally phased: lane lines/icons moved through transitional values and the final inactive display tuple could lag `0x12A` by roughly 0.3-1.45 seconds. This asymmetry argues for a two-phase force experiment that pre-arms the active ownership/display state before asserting the torque-overlay request, then uses a separately observed natural-style release sequence.

### 23. Resolve the apparent car/openpilot engagement mismatch

**Question:** Why can the car appear engaged and steer while openpilot reports disengaged?

**Method:** Correlate the physical LFA button, `selfdrived` state, `carControl.enabled`, and `carControl.latActive`.

**Result:** The steering-wheel LFA button toggles sunnypilot MADS lateral-only state. Captures repeatedly showed `enabled=false` with `latActive=true`; they also showed the stock camera continuing clean steering while `latActive=false`. This is expected separation between full openpilot engagement and lateral availability/stock camera behavior.

**Conclusion:** The UI mismatch was not a CAN state parsing error and does not explain the faults.

### 24. Final stock/pass-through wrap capture

**Question:** Can the expanded camera-message and damping dataset be collected without changing vehicle behavior, and does the proven forwarding path remain healthy through the final drive interval?

**Method:** Run one streaming recorder only, with no firmware or control change and no in-memory frame history. Record every camera-segment frame plus bus-0 MDPS, physical bus-0 LFA output, and synchronized state. `carControl.latActive` was false for almost the entire interval, making this a stock/pass-through reference rather than an openpilot-torque validation.

**398.75-second result:**

- 39,738 camera `0x12A` inputs and 39,738 physical outputs.
- Zero source counter gaps, zero output counter gaps, zero sequence offset, and 39,738/39,738 byte-exact payloads.
- 9,525 active source frames: 8,928 with recognition 3 and 597 with recognition 0 during natural boundary/grace behavior.
- 30,191 inactive frames with recognition 0 plus 22 brief inactive/recognition-3 boundary frames.
- MDPS states: 30,306 inactive healthy and 9,567 active healthy; unable/fault/fail were always zero.
- All 7,947 `0x162` samples had `(FAULT_LSS, FAULT_LFA, FAULT_DAS)=(0,0,0)`.
- All 11 expected camera-segment addresses remained present at their established rates.

**Conclusion:** The final reference is internally consistent, fault-free, and long enough to support the signal/correlation findings. Because it is byte-exact stock pass-through, it supplements rather than replaces the modified-torque Stage 1 evidence.

### 25. Pre-arm the visible camera ownership state

**Question:** Can forced no-line steering be made consistent by transforming the physical camera's `0x161` and `0x1E0` display/ownership messages before asserting `0x12A`?

**Method:** While force mode is requested, transform only `CENTERLINE`, both lane-line states, and `LFA_ICON` on camera `0x161`, plus `LFA_ICON` on camera `0x1E0`. Preserve each physical counter, cadence, unrelated bit, and recompute its checksum. Do not assert forced `0x12A` until one transformed copy of both companion messages has crossed Panda.

**Result:** MDPS accepted forced ownership and stayed healthy, but `0x162` still asserted LSS/DAS. An initial comma-only `controlsMismatchLateral` alert was separately traced to removing the only relay-check entry that drove the fork's MADS state update; restoring a non-blocking `0x1E0` relay-check entry made Panda lateral permission and `carControl.latActive` transition together. That software-state bug did not explain the vehicle fault.

**Conclusion:** The visible `0x161`/`0x1E0` state is not sufficient. The pre-arm mechanism works as designed, but the rejecting observer sees another contradictory input.

### 26. Fabricate valid lane geometry and valid-checksum empty MDPS feedback

**Question:** Does the rejecting ECU require a plausible camera lane model, and can an otherwise-empty but counter/checksum-valid `0xEA` hide the MDPS acknowledgement?

**Method:** On physical camera `0x1B5`, preserve cadence, counter, lead data, unknown bits, and checksum behavior while substituting high-confidence straight lanes at the median natural active positions: left `-1.63255 m`, right `+1.5493375 m`, zero heading/curvature/derivative, and no departure state. In the opposite forwarding direction, retain the physical MDPS counter but set all other `0xEA` payload bytes to zero and recompute its checksum.

**Result:** At `59735.457`, forced mode and real MDPS active acknowledgement appeared with all MDPS unable/fault/fail fields zero. LSS/DAS changed from `(0,0)` to `(1,1)` at `59735.493`, about 36 ms later. A second engagement reached MDPS active at `59754.184` and LSS/DAS at `59754.670`. Disengagement cleared MDPS active and the `0x162` faults within roughly 100 ms. The result repeated whether the honest source `0x1B5` qualities were `(0,0)`, one-sided, or naturally `(3,3)`.

**Conclusion:** Car-facing fabricated geometry does not cure the rejection. A valid checksum and counter are not enough to make an otherwise empty `0xEA` semantically valid.

### 27. Send a literal all-zero `0xEA` copy toward the camera side

**Question:** Does the camera-side observer treat a missing/invalid-looking MDPS frame differently from the valid-checksum empty variant?

**Method:** During forced control only, replace all 24 bytes of the forwarded bus-0-to-bus-2 `0xEA` copy with zero. The real car-side MDPS frame and all dashboard faults remain unmodified.

**Result:** This was strictly worse. MDPS became active at `59842.939`; by `59842.958`, `0x162` asserted `(FAULT_LSS, FAULT_LFA, FAULT_DAS)=(1,1,1)`. The second engagement repeated the full tuple within about 81 ms. Disengagement cleared the tuple in stages over roughly 160 ms.

**Conclusion:** `0xEA` is actively validated. Destroying its counter, checksum, and configuration/status fields creates the additional LFA fault, while the original LSS/DAS contradiction remains.

### 28. Hide only the two-bit MDPS active acknowledgement

**Question:** Is the contradiction specifically between the camera-side segment's honest inactive `0x12A` and the forwarded `MDPS_LkaToiActvSta=1` acknowledgement produced by Panda's car-side forced request?

**Method:** Preserve the complete physical `0xEA` payload, counter, cadence, plugin/configuration states, torque, angle, and all MDPS fault fields. Clear only `MDPS_LkaToiActvSta` at bits 48..49 in the copy forwarded toward bus 2, then recompute the checksum. The real bus-0 MDPS frame remains untouched and observable. Keep `0x162` unmodified.

**Initial moving result:** Two forced engagements remained at `(FAULT_LSS, FAULT_LFA, FAULT_DAS)=(0,0,0)` throughout the capture. The first began at `60259.480`; the second began at `60271.899` and remained clear through the end of the monitor at `60295`. Real bus-0 MDPS reported active with unable/fault/fail continuously zero. Openpilot requested torque from approximately `-5` through `-113`; some transition commands were rejected by Panda's torque-rate safety and driver override subsequently reduced requested torque to zero. Thus the semantic-fault result is positive, while sustained physical openpilot actuation still needs a clean handoff validation.

**Conclusion:** This is the first forced no-line experiment that kept both enforcement layers healthy without suppressing any diagnostic. The result strongly identifies the contradictory MDPS active acknowledgement—not lane geometry—as the camera-side fault trigger. It is still an initial checkpoint: the next test must fix/characterize the torque handoff, demonstrate sustained accepted nonzero torque, exceed the previous misleading clean windows continuously, and repeat several engagements.

### 29. Recover the torque handoff at the camera active-to-inactive transition

**Question:** Why can forced steering work after a quick LFA-button double tap, yet sometimes give up immediately during an ordinary engagement?

**Method:** Monitor the physical camera `0x12A`, transformed car-side `0x12A`, real MDPS `0xEA`, openpilot torque output, Panda `safetyTxBlocked`, and all three decoded `0x162` fault fields through the same engagement. Then repeat after two changes: use the measured stock active damping schedule instead of the inactive value 100, and hold a zero command for five control frames whenever the physical camera changes from active to inactive while openpilot remains laterally active.

**Failed handoff result:** The first trace began while the camera was still naturally active and its stock torque was changing rapidly. Openpilot and Panda sampled adjacent rather than identical 100 Hz camera frames. The first takeover step violated Panda's torque-transition check, which reset Panda's desired and real-time torque histories to zero and returned force mode to pass-through. Openpilot continued ramping from its own older nonzero history. `safetyTxBlocked` then increased on effectively every 100 Hz internal command, the physical camera became inactive, the transformed frame also returned to inactive/damping 100, and MDPS stopped accepting torque. All `0x162` fields nevertheless remained zero; this was a safe local rejection, not the original vehicle fault.

**Double-tap interpretation:** A quick LFA-button cycle forces a genuine inactive camera frame with zero requested torque. That accidentally gives both Panda and openpilot the same baseline, explaining why the next engagement can work. No evidence indicates that the button message itself must be blocked or fabricated. Doing so would create a new camera/cluster/driver state mismatch.

**Initial automatic recovery:** When the physical camera reports inactive during an openpilot lateral request, hold zero for 50 ms, then ramp from zero under the unchanged Hyundai driver-torque, rate, and real-time safety limits. This was the first attempt to automate the synchronization effect of the double tap without changing button traffic. It produced sustained no-line steering, but the later camera-state flicker proved that active-zero is not equivalent to a real pass-through transition.

**Recovery results:** The next no-line trace began with the physical camera continuously inactive at `(torque, ActToiSta, Damping_Gain)=(0,0,100)`. Panda transformed each physical frame to openpilot torque with `ActToiSta=1` and speed-based damping that moved from approximately 36 to 10 as speed fell from 14.3 to 0.3 m/s. Real MDPS remained active. Openpilot torque reached at least -96 through +54 during the roughly 27-second active interval. `safetyTxBlocked` remained exactly zero from startup, and `(FAULT_LSS, FAULT_LFA, FAULT_DAS)` remained `(0,0,0)`. Disengagement returned the car-side frame to exact inactive stock bytes and MDPS inactive with the fault tuple still clear.

A second no-line engagement from the same continuously inactive camera state initially succeeded. The observed car-side frame stayed inactive/zero briefly after `latActive` rose, began the torque ramp at -2, and MDPS acknowledged active about 20 ms later. `safetyTxBlocked` remained zero. However, roughly 86 seconds into the overall monitor, the physical camera briefly changed active and then inactive. At that edge, `0x162` transiently asserted `(1,1,0)`, then `(1,0,1)`, then `(0,0,1)` before clearing about two seconds later. Panda continued accepting commands and MDPS continued steering. Therefore the five-frame active-zero hold solved the torque-history lockout but did not reproduce the genuine inactive vehicle state needed at a camera-state transition.

**Revised recovery:** During the five-frame synchronization window, send Panda's pass-through mode rather than force-active with zero torque. This preserves the physical camera's complete inactive `0x12A`—including `ActToiSta=0` and damping 100—long enough for MDPS and CCNC to observe the same real inactive transition. After that window, begin forced active torque from zero. This is the actual protocol equivalent of the useful state transition produced by the driver's double tap.

Moving validation exposed one more transition-specific detail. Keeping force mode and hiding `MDPS_LkaToiActvSta` while the physical camera itself briefly became active caused the camera to see its own active request without the corresponding MDPS acknowledgement. The camera withdrew the request and transiently asserted `0x162`. The minimal correction is dynamic ownership: when source `ActToiSta=1`, use ordinary lane-active substitution and forward authentic MDPS state; when source `ActToiSta=0`, use force mode and clear only the camera-facing MDPS active acknowledgement. Openpilot supplies the requested torque in both cases.

**Validated transition result:** A 163.808-second moving capture after that correction contained 16,241 physical camera `0x12A` frames and exactly 16,241 returned car-facing frames. Both streams had zero counter discontinuities and zero paired-counter mismatches. The physical camera changed active state 20 times; real MDPS followed with 20 active-state edges. All 16,382 MDPS samples had fault/fail `(0,0)`, and all 3,248 `0x162` samples had `(FAULT_LSS, FAULT_LFA, FAULT_DAS)=(0,0,0)`. Of the paired LFA frames, 1,611 were exact pass-through and 14,630 were transformed; no transformed frame changed a bit outside checksum, torque, request, or the intentional force-mode damping field. Openpilot torque reached the 270 safety limit during no-line force mode without producing a Hyundai fault.

The same capture exposed a short-edge state-machine bug without producing a vehicle fault. If the physical camera went inactive and active again in less than the 50 ms pass-through window, MDPS was naturally active before the inactive wait completed and the controller could remain in pass-through indefinitely. The minimal correction is to cancel the synchronization phase immediately when the physical camera is active again; ordinary lane-active substitution then resumes from the authentic source state. Subsequent ordinary transitions remained clean. Capturing another sub-50 ms inactive/active edge is still needed to validate that exact branch directly.

The Panda-wide blocked counter increased from 2 to 6 during the capture, one increment at four observed active-to-inactive handoffs. Each rejection was isolated: pass-through still reached MDPS, the zero-active phase completed, torque resumed, and no rejection cascade or vehicle fault followed. Because this counter is global and the recorder did not retain the rejected address, exact attribution remains pending. The safety boundary behaved fail-safe even if all four were virtual steering commands.

**Damping status:** Raw `0xA0` capture replaced the earlier provisional `floor(vEgo_kph - 20)` hypothesis with the piecewise schedule and effective first-order speed filter documented in section 21. The newest middle-speed sweep directly covered damping 10..54 and every step from roughly 30 through 74 km/h. Every clean-run change was on an even LFA counter, and every model miss was only one count. No production curve change is justified until the remaining filter phase/coefficient and 100-108 km/h taper are independently repeated.

**Conclusion:** The no-line semantic fault and the intermittent no-line torque loss were separate. Clearing only the camera-side MDPS active acknowledgement prevents the steady-state CCNC semantic fault. Resetting the takeover to the physical camera's inactive zero-torque state prevents the Panda safety lockout. The transition must be true pass-through, not merely an active request with zero torque, and the camera-facing MDPS acknowledgement must be hidden only while the physical camera is inactive. This dynamic protocol has now survived 20 natural lane-state transitions and sustained no-line steering with diagnostics untouched. None of these mechanisms requires blocking LFA-button traffic or suppressing `0x162`.

### 30. Explain the repeated steering cut during a sustained turn

**Observation:** The driver felt steering repeatedly release mid-turn. During each event, openpilot was still laterally active and requested substantial torque, but the car-facing `0x12A ActToiSta` briefly changed from 1 to 0. Real MDPS fault/fail remained zero and every decoded `0x162` fault field remained zero. This was therefore not a recurrence of either Hyundai fault mechanism.

**Cause:** `CarController` still applied openpilot's legacy Hyundai `common_fault_avoidance` rule. Above 85 degrees of steering angle, it counts 89 control frames and then clears the steering request bit for two frames. In a sustained turn that creates a roughly 20 ms request withdrawal about every 0.9 seconds. The trace matched this exact periodicity and state change. The camera-synchronized handoff did not amplify the cut; it forwarded the deliberately cleared request.

**Bypass experiment:** The periodic withdrawal was disabled only for `CANFD_LFA_CAMERA_SYNC` and tested on the same car. No-line steering remained continuously healthy for approximately four minutes, including repeated torque saturation at 270 and curves through 46.3 degrees. On the first sharper turn, angle crossed 85 degrees and reached approximately -145 degrees while the request remained active. At `t=243.987`, real MDPS changed to unable/fault `(1,1)`; openpilot lateral control dropped 10 ms later; at `t=244.097`, `0x162` asserted `(FAULT_LSS, FAULT_LFA, FAULT_DAS)=(1,0,1)`. Panda reported zero blocked transmissions. The failure sequence and roughly one-second high-angle dwell match the purpose of `common_fault_avoidance` directly.

**Conclusion:** The high-angle request reset is required on this CCNC MDPS and was immediately restored. This is separate from the host-relay timing fault and the no-line camera/MDPS acknowledgement contradiction. Continuous request above the MDPS angle/time limit cannot be used as the solution. The restored guard has now been recaptured through matched sharp turns. Any later smoothing must preserve two complete inactive frames; torque tapering before the reset and ramping afterward remains an optional comfort refinement, not a fault fix.

**Restored-guard checkpoint:** The reverted build then completed a separate 300-second moving capture with MDPS fault/fail always zero, every decoded `0x162` tuple zero, and `safetyTxBlocked=0`. It survived multiple natural camera active/inactive handoffs and a strong-driver-override withdrawal including a complete MDPS inactive/active synchronization. That route did not contain another angle above 85 degrees, so it validates that the revert preserved the main ownership solution but is not a matched sharp-turn comparison.

**Matched two-frame result:** A follow-up sharp turn reached -109 degrees with the original two-frame reset. The first reset occurred at -99 degrees and the second approximately 0.92 seconds later at -109 degrees. Each changed the car-facing tuple from active damping to `ActToiSta=0, Damping_Gain=100` while leaving requested torque near -270, briefly made MDPS inactive, and then restored MDPS active. MDPS and `0x162` faults stayed zero and the Panda blocked counter remained at its starting value. This directly proves the reset prevents the bypass failure, while the full-torque active/inactive edge explains the perceived steering release.

**One-frame refinement rejected:** For `CANFD_LFA_CAMERA_SYNC` only, `max_mismatching_frames` was temporarily reduced from two to one. An initial low-speed sharp-turn capture produced five apparently successful resets approximately 0.9 seconds apart at angles from about -206 through +457 degrees. The longer road test disproved that result. At `t=47.642`, while moving 3.21 m/s at -226.1 degrees with requested and car-facing torque both -270, MDPS entered unable/fault `(1,1)` before openpilot disengaged. Approximately 132 ms later, `0x162` asserted all three decoded fault fields. The preceding trace contains accepted one-frame resets at `t=45.759` and `t=46.660`, but no observed reset before the failure at `t=47.642`. A single 10 ms inactive frame can therefore be missed by this MDPS even though adjacent pulses are accepted. The one-frame change was reverted during the drive.

**Long restored-guard result:** The same continuous recorder ran for another 710.644 seconds after restart with the original two-frame guard. It reached 24.37 m/s (87.7 km/h), crossed 27 physical-camera active-to-inactive and 27 inactive-to-active edges, and recorded 34 complete MDPS inactive/active cycles. Every summarized MDPS fault/fail sample and every `0x162` tuple remained zero. A final matched sharp-turn sequence reached -140.1 and +207.3 degrees with torque at both -270 and +270. Six two-frame watchdog resets were observed across the two sustained turns; within each turn their start times were separated by 0.900 to 0.921 seconds. MDPS observed every reset, went inactive, and re-entered without fault. The two-frame value is now the minimum validated setting for this vehicle.

Seven seconds after disengagement at standstill, Panda's global blocked counter rose from 0 to 24 in approximately 0.2 seconds. No vehicle message or diagnostic changed and the burst was not concurrent with a high-angle reset. Because the aggregate counter does not identify the rejected address, this is retained as a separate disengagement-race observation rather than attributed to the one-frame change.

### 31. Separate camera lane-state handoffs from high-angle watchdog cuts

**Observation:** The restored two-frame road trace exposed frequent steering withdrawals at small angles where the high-angle watchdog was not active. While openpilot remained laterally active, the physical camera repeatedly changed `ActToiSta` from 1 to 0 as stock lane confidence flickered. The current ownership state machine treats every such edge as a fresh synchronization event: it forwards the authentic inactive camera frame, waits for MDPS to report inactive, then sends zero-active and ramps openpilot torque again. Typical withdrawals lasted about 100 to 130 ms, substantially longer than the 20 ms high-angle reset.

**Evidence:** In the post-restart 710.644-second segment, the physical camera produced 27 active-to-inactive and 27 inactive-to-active edges. Car-facing `0x12A` produced 35 inactive edges and 34 active edges, while MDPS completed 34 inactive/active cycles. Several transitions occurred during substantial requested torque, including the camera withdrawal near `t=777.338` while openpilot requested -270. All MDPS and `0x162` fault fields stayed zero. Panda's aggregate blocked counter rose only from 0 to 14, in isolated single increments at ownership boundaries rather than sustained rejection.

**Conclusion:** The remaining ordinary-road steering cut is not damping, host timing, or the >85-degree watchdog. It is a deterministic consequence of coupling openpilot ownership to every stock camera lane-state edge. The next experiment should test sticky openpilot ownership: perform the real pass-through synchronization only when openpilot first takes ownership from an inactive camera, but do not repeat it merely because stock lane confidence drops while openpilot is already engaged. This must be tested cautiously because earlier work proved that camera-facing MDPS acknowledgement still has to switch dynamically: authentic while the camera is active, cleared while it is inactive. No change to fault reporting or `0x162` is proposed.

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
- Panda-timed lane-gated substitution preserves one physical output per source frame and remains free of MDPS and CCNC faults across windows longer than every prior replay failure.
- `0x12A`-only forced ownership makes MDPS active and produces physical steering without MDPS fault/fail, but CCNC faults immediately when the camera is otherwise lane-inactive.
- Forcing `LKA_RcgSta` and cached natural active damping does not prevent that CCNC fault.
- Natural activation changes `0x161` and `0x1E0` icon/lane state and `0x1B5` lane validity together with `0x12A`.
- `Damping_Gain` is dominated by vehicle speed: inactive is 100, active floors at 10 at low speed and rises nonlinearly into the low 80s at highway speed.
- An active-camera takeover can fail when the host and Panda start from adjacent, rapidly changing stock torque samples; after the first rejection their torque histories diverge and every later command can be rejected.
- A physical inactive camera frame provides an authoritative zero-torque synchronization point. Active-zero alone can produce sustained accepted torque, but a later camera-state flicker proved that a real pass-through transition is required before the zero-active takeover.
- Removing the legacy >85-degree request reset causes this exact MDPS to enter unable/fault during a sustained sharp turn; the reset is required even with perfect Panda-timed camera cadence.
- Clearing only the camera-facing `MDPS_LkaToiActvSta` acknowledgement while the physical camera is inactive removes the no-line CCNC contradiction; authentic acknowledgement is restored whenever the camera is active.
- Dynamic ownership survived 163.808 seconds, 20 camera/MDPS state edges, and torque to +/-270 with all 16,382 MDPS fault/fail samples and all 3,248 `0x162` samples clear.
- The periodic release during sustained turns above 85 degrees is openpilot's required request-bit watchdog reset, not an MDPS or CCNC fault; one inactive frame can be missed, while two frames passed the matched long validation.
- A second, longer 100-to-130 ms release occurs at ordinary steering angles because the current ownership state machine repeats its pass-through synchronization whenever the stock camera loses lane-active state. Those edges remained diagnostically clean but are the main remaining continuity problem.

### Strong inference

The original relay fault and the no-line force fault have different reporters and should not be conflated:

- MDPS enforces a continuity, freshness, timing, or source-stream rule that is violated when the camera's 100 Hz LFA traffic is terminated and recreated through the host application path. The Panda-timed lane-gated result strongly validates this causal model, although the exact internal timer is still unknown.
- CCNC enforces semantic consistency among the camera's steering request, lane/display state, and possibly MDPS acknowledgement. The exact compared fields are not proven, but `0x161`/`0x1E0`/`0x1B5` are now concrete candidates rather than an unspecified checksum or torque problem.

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

Status: the `0x12A`-only, recognition/damping, companion-display, and fabricated-geometry phases were rejected. Whole-frame `0xEA` hiding was also rejected. Clearing only the forwarded camera-side copy of `MDPS_LkaToiActvSta`, restoring authentic acknowledgement during natural camera-active state, and performing a real `PASS_THROUGH -> ZERO_ACTIVE -> RAMP` handoff passed the 163.808-second/20-transition moving validation. The next simplification is to remove the now-unproven companion-state transforms one group at a time; the high-angle request-cut bypass also requires moving validation.

The next phase should isolate the smallest cross-message consistency set:

1. Keep the proven physical `0x12A` cadence and every `0x162` diagnostic unchanged.
2. Re-enable force mode with zero target torque first.
3. Enter a pre-arm phase: on physical `0x161`, change only `CENTERLINE=1`, `LANELINE_LEFT=2`, `LANELINE_RIGHT=2`, and `LFA_ICON=2`; on physical `0x1E0`, change only `LFA_ICON=2`. Preserve both messages' physical counter, timing, every other bit, and checksum behavior.
4. After Panda has physically forwarded at least one transformed copy of both messages, assert a zero-torque `0x12A` request. This avoids exposing CCNC to the exact long-lived contradiction produced by the failed force tests.
5. Do not fabricate `0x1B5` lane geometry in the first test. This distinguishes a display/ownership consistency check from a requirement for a false lane model.
6. Confirm whether CCNC remains clear and whether MDPS stays active at zero torque. Only then ramp a very small safety-limited torque.
7. Release in a controlled phase rather than instantly dropping every companion state; record the natural transitional icon values before choosing the final release sequence.
8. If CCNC still faults, add controlled instrumentation around `0x1B5` quality transitions and the forwarded MDPS `0xEA` acknowledgement. Do not block either message.
9. Revert immediately to exact stock pass-through on a stale command, invalid source checksum, disengagement, or any fault.

This sequence tests one hypothesis at a time and avoids jumping directly to invented lane geometry. Modes 3 and 4 remain disabled in normal `CarController` operation.

## Open questions

1. What exact MDPS monitor triggers the original host-relay failure: maximum inter-frame gap, source-clock jitter, duplicate arrival, missing counter deadline, bus-level transmitter identity assumptions, or a combination?
2. Are the `0x161`/`0x1E0`/`0x1B5` transforms needed at all once only the camera-side MDPS active acknowledgement is hidden?
3. Which internal command is responsible for each isolated handoff increment in Panda's global blocked counter?
4. Does removing the legacy >85-degree periodic request cut remain fault-free through repeated sustained turns?
5. Are other recent CCNC non-HDA2 Hyundai/Kia/Genesis platforms subject to the same strict stream behavior?
6. What are the semantics of the still-unnamed live bits in `0x11A`, `0x160`, `0x200`, `0x1FA`, and the undefined `0x38C` message?
7. What exact fixed-point filter phase/offset explains the last one-count damping residual when measured entirely inside Panda rather than across host-observed buses?

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

## Preserved research artifacts

The live data was copied off-device before any optional reboot to:

`/Users/samaritan/ccnc-research/2026-08-03-live/`

| File | Purpose | SHA-256 |
|---|---|---|
| `lfa_timeline.csv` | 13 MB transition timeline for camera/output `0x12A`, MDPS `0xEA`, CCNC `0x162`, and control state | `046f095156332361e3e3625b7ae2a9960f6455a23f283141c98f9a7e625eb7a4` |
| `ccnc_lfa_recognition_20260803.jsonl.gz` | high-speed synchronized LFA/vehicle reference and decode/repack validation | `b80fa33fa9715a0c217b1b4e8108e7db02561496f222d983e5ba4a1684f29fa0` |
| `ccnc_lfa_speed_sweep_20260803.jsonl.gz` | low-speed synchronized LFA, lane geometry, MDPS/CCNC, `carState`, and `carControl` | `f968e1f94ec060d8218a79777d31b103f4fb5c70ec0410f190f8287fa1cec0ab` |
| `ccnc_camera_bus_20260803.jsonl.gz` | exhaustive 11-address raw camera-segment capture | `e6b9adfb6a9696887eb8ba24482be707537abf57d6742fcf1a9b19ae75778815` |
| `ccnc_final8_20260803.jsonl.gz` | final 398.75-second all-camera-message plus synchronized vehicle/control/MDPS reference | `66eb9cbbb86782286555736ab0f0b3deb91ee3f407635c3e34e15bed6a0bc250` |
| `2026-08-04-live/ccnc_live_20260804_1607.jsonl` | 163.808-second dynamic lane/no-line ownership capture with 20 camera and MDPS active-state edges | `b8f8a4007aa1577d4b139bb57319aea7d16782b42a0cb846d591bf5653cfe1d7` |
| `2026-08-04-live/ccnc_live_20260804_1610.jsonl` | 414-second raw-wheel/local-road reference and repeated low/middle damping transitions | `b79d4d0bb2bb76f8d80b6815633fd4123dea07abaabf970c1d8b402bf7cd9a47` |
| `2026-08-04-live/ccnc_live_20260804_1625.jsonl` | 900-second synchronized raw-wheel sweep, damping 10..54, state/torque/lane variables, and fault monitoring | `a6546b2cf36ef7e36dc05c1b046c3e5df2f7773ab852718d46209f41214be53b` |
| `2026-08-04-live/ccnc_live_20260804_1700_all_camera_inputs.jsonl` | 501-second all-camera-input capture proving the `0xA0` four-wheel-average speed source; 226 damping transitions and override-cut timing | `c2357476ccc0d2ad66a456c2bba8cb990bd356cbb8ecc31645d776bdb0dc7366` |
| `2026-08-04-live/ccnc_live_20260804_resume_damping_partial.jsonl` | 245 MB locally preserved prefix of the 1,448.8-second final run; contains the complete useful stock-active/high-speed section, 79,494 wheel frames, and 26,152 active even-counter damping samples | `e1bca48102973e18959ac15507a3610a5ad894ba2c427943764535c3f2d3d6cc` |
| `2026-08-04-live/ccnc_live_20260804_resume_damping.jsonl` | Complete 1,448.8-second final run copied from the comma and hash-verified locally | `0f72213c80779aad482b5ca4bb0318ccb6de059f689e8fdcae3e2b75d6a5c54d` |
| `2026-08-04-live/ccnc-port-lite-device-source-20260804.tar.gz` | Exact comma-side copies of every modified/untracked opendbc and Panda source file at wrap-up; device openpilot `14f5b065`, opendbc base `51bba734`, Panda base `61b050f1` | `24d02c138e5ec5ed3bc48aa6b05e780ac43a51b530b14b56d6d17d18007c6e14` |
| `2026-08-04-live/ccnc_live_high_angle_fix_20260804.jsonl` | 245-second moving bypass experiment proving the >85-degree request reset is required; MDPS faulted first near -145 degrees, followed by `0x162` LSS/DAS | `fd5308f2bbc3da13adfabd160a2a1e2293f962f7f6443bb89a727ae6f38a0e21` |
| `2026-08-04-live/ccnc_live_high_angle_guard_20260804.jsonl` | 300-second restored-guard run with multiple camera/MDPS handoffs, zero MDPS/`0x162` faults, and zero Panda blocks; no matched >85-degree turn occurred | `5f251cf61177838213f89707d72fae31bb82d267def86734e558cb579165a8bc` |
| `2026-08-04-live/ccnc_live_high_angle_guard_followup_20260804.jsonl` | Matched original two-frame high-angle reset at -99/-109 degrees; MDPS briefly inactive but all fault fields clear | `b5093e9cfa70da1278dee3e4d96974be20ec7b6909352f009086897b566de22f` |
| `2026-08-04-live/ccnc_live_high_angle_one_frame_20260804.jsonl` | Five CCNC-only one-frame resets through approximately -206..+457 degrees with no MDPS/`0x162` fault; separate post-disengagement Panda blocked burst retained | `5bc0e0d14e656d4ca075adfbee666c5bd45b70a5af5a89878fe1275800a318bf` |
| `2026-08-04-live/ccnc_live_one_frame_20min_20260804.jsonl` | Continuous 1,200-second comparison: one-frame guard faulted MDPS at -226 degrees; after in-drive revert, 710.644 seconds with the two-frame guard survived 54 camera state edges, six matched high-angle resets, and all MDPS/`0x162` fault fields clear | `f7a86cbc5d5c9022cea0474563e3dd6dead31f2289217447b419d16651038ff8` |

The full final capture remains on the comma at `/data/media/0/ccnc_live_20260804_resume_damping.jsonl` and is now also preserved locally. Both device and local SHA-256 values are identical. The device recorder variants and the exact modified source snapshot were copied alongside it, so a device wipe would not remove any code or data needed to reproduce this checkpoint.

All four gzip files passed `gzip -t` before and after copy. The custom recorders streamed to disk and retained no frame history in RAM. During an attempted live correlation, one temporary reader accumulated decoded JSON objects and reached 425 MB RSS; it was terminated and is not part of openpilot. A subsequent process audit found one driving stack, no duplicate `controlsd`/`modeld`/`pandad`/`loggerd`, about 661 MB RAM available, no swap, and `/data` at 90% usage with 8.9 GB free. The user-reported restart/lag issue remains an upstream validity concern and was not modified here.

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

The evidence no longer supports one undifferentiated “Hyundai steering fault.” There are two independently observed enforcement layers:

1. MDPS rejects a camera stream recreated through the host even when its bytes, counter, and checksum are correct. Preserving the original physical 100 Hz stream and transforming it inside Panda fixes that lane-gated failure without hiding diagnostics.
2. The camera-side segment rejects forced steering when it sees the stock camera's inactive/no-line request locally but receives `MDPS_LkaToiActvSta=1` from the car side. Fabricating outward `0x161`, `0x1E0`, and `0x1B5` state did not solve it. Whole-frame `0xEA` replacement added faults. Preserving the authentic `0xEA` and clearing only its two-bit active acknowledgement produced two fault-free forced engagements while the real MDPS remained active and healthy.

The next defensible experiment is to remove the now-disproven broad geometry/whole-frame variants, retain the targeted two-bit camera-facing transformation, correct or characterize the stock-to-openpilot torque handoff, and validate sustained nonzero steering across repeated long engagements. `0x162` must remain completely untouched.
