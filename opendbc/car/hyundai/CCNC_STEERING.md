# CCNC steering: what we learned

Findings from reverse engineering how the stock Hyundai/Kia CCNC front camera steers, compared with how openpilot steers. Everything here comes from real logs unless it says otherwise.

## Data used

| Source | What it gave us |
|---|---|
| One Sonata HEV 2024, 719 segments | One stock drive (op off, LFA on) and ~5 h of op steering |
| Other CCNC cars, 1,308 segments | K5 2025, K4 2025, Kona EV/HEV 2nd gen, Tucson HEV 2025, Ioniq 5 N, two more Sonata HEVs |
| Official Hyundai DBCs (HMVS4 3Gen R3, STD_DB 2024 FD) | Official names, units and value tables for 0x12A, 0x1E0, 0xEA |

Clean stock lane-following exists on only two drives: one Sonata HEV drive (479 s, 0–100 kph) and one K4 drive (129 s, 55–77 kph). Everything else was op steering, or LFA was off.

## Two camera layouts

**Camera does LFA** (Sonata 2024, Sonata HEV 2024, K5 2025, Kona 2nd gen, Tucson 2025): the camera sits on bus 2 and sends 0x12A itself. It sends exactly 11 IDs: `0x11A 0x12A 0x160 0x161 0x162 0x1A0 0x1B5 0x1E0 0x1FA 0x200 0x38C`. The Tucson HEV adds `0xCB` (angle command, always empty) and `0x2BA` (front-lighting targets, idle). Confirmed two ways: these IDs vanish from bus 0 the moment the relay closes, and their timing runs on the camera's own clock (0.9916x nominal, all other ECUs ~1.0000x).

**HDA2-style** (K4 2025, Ioniq 5 N): the camera is alone on A-CAN and sends 31 IDs (`0x110` LKAS_ALT, `0x165`, `0x230`, `0x235`-`0x248` object slots, `0x360`-`0x366` lane messages, `0x380`). LFA comes from a separate ADAS ECU as 0x12A on bus 1. opendbc detects this from `0x110` on the camera bus and sets `CANFD_LKA_STEER_MSG_ALT`. None of the findings below carry over to these cars without new data.

## Camera faults: cause and fix

The camera needs the MDPS message (0xEA) 100 times a second. The old CCNC safety blocked the real 0xEA from reaching it, because when the camera saw the MDPS report LKA torque (op's) that it never requested, it stored a fault code. With no 0xEA at all it faulted instead, about 0.5 s after the relay closed, on every drive (`LKA_SysWrn` 15, byte 9 = 15, `FAULT_LFA`/`LSS`/`DAW` set), and `create_ccnc` cleared those faults and alerts before they reached the cluster.

What fixes it, with nothing hidden:

1. op sends the camera a copy of the real 0xEA every frame, checksum recomputed, with two fields changed: `MDPS_LkaToiActvSta` 0 (no LKA torque active) and `MDPS_LkaToiUnblSta` 1 (LKA torque unavailable). Every other bit is the real message.
2. Panda safety forwards the real 0xEA to the camera until op sends its first copy, then blocks it (`.disable_static_blocking` on the 0xEA TX entry plus a forward hook). Without this the camera faulted for a few seconds after every ignition, before card started.
3. 0x7C4 (camera diagnostics) is no longer blocked. op never sends it while driving.

Why "unavailable": once the camera was healthy, its own LFA engaged alongside op and ran the stock hands-on-wheel timer. op steers, so the MDPS sees almost no driver torque, and the camera warned (`ALERTS_2` 1), chimed (`ALERTS_2` 2, `SOUNDS_2` 3) and cancelled (`ALERTS_2` 3, `SOUNDS_2` 6) about 28, 43 and 68 s after engaging. Its LFA can never steer under op anyway (op replaces its 0x12A), so reporting LKA torque unavailable is accurate: the camera's LFA stays in standby and the timer never starts.

Result on drives with op steering: camera `LKA_SysWrn` 0, LFA mode, byte 9 = 9, no fault bits, no hands-on alerts, including across ignition cycles and op lateral on/off toggles.

Known leftovers:
- Before op's first copy (a few seconds after ignition) the camera hears the real 0xEA, which says available, and its LFA can engage briefly. Nothing has shown on the cluster from it.
- The camera's normal pop-ups now reach the cluster, see "Camera messages" below.

## How the MDPS responds to a torque request

Fitted from 8 min of stock and 225 min of op steering on the Sonata:

```
MDPS output torque ~= 0.05 Nm x request  -  (0.075 x Damping_Gain/100) Nm per deg/s of steering speed
```

- Delay from request to MDPS output: 20-30 ms.
- Officially the request is 1/128 Nm (0.0078 Nm) per count, range -8 to +8 Nm. So 270 = 2.1 Nm, 384 = 3.0 Nm, stock peak 167 = 1.3 Nm. A bench MDPS faulted at 410 (~3.2 Nm).
- **Damping_Gain is a brake against steering speed, not a gain on the request.** At DG 100 and 50 deg/s it takes back ~3.8 Nm (about 75 counts of request). Below 30 kph about half of the damping is fixed and half follows DG. Officially DG is a percentage, valid 3-200.

## How stock steers vs how op steered (before this branch)

| | Stock LFA | Stock LKA-only | op (old) |
|---|---|---|---|
| Torque request, median / p95 / max | 17 / 73 / 141 | 16 / 113 / 167 | 30 / 174 / 270 |
| Torque at 0-20 kph, median / p95 | 12 / 58 | | 60 / 225 |
| Rise and fall per 10 ms frame | median 3, p90 4, rare 9-11 | same | capped at 2 up, 3 down |
| Damping_Gain while steering | speed table (below) | same | always 100 |

What held op back:
- The 2-per-frame rise limit. Torque sat at the cap on 84% of rising frames; 6,600+ ramps were held at the cap for 100 ms or more.
- Damping_Gain 100 at all speeds. At 0-20 kph stock gets ~52% more output per count than op did.
- The 270 ceiling at low speed (hit on 2-3% of frames below 60 kph).
- The torque tune: `HYUNDAI_SONATA_HEV_2024` borrows `HYUNDAI_SONATA_HYBRID`'s tune, which was measured on a car running STEER_MAX 384. At 270 every command came out ~30% weaker than the tune expected.

### Damping_Gain rule (stock camera)

```
DG = floor(interp(kph, [0, 30, 40, 50, 60, 100], [10, 10, 25, 35, 40, 80]))   # while steering
DG = 100                                                                       # not steering
```

Matches the Sonata camera exactly on 92.8% of 50k frames and within 1 on all of them (misses sit at step edges). A second Sonata HEV matched 95.2%. Same in LFA and LKA-only mode. Unknown above 100 kph (stock data stops at 105 kph, value 81). The K4's ADAS ECU uses a different, higher table (~63-65 at 60-80 kph).

### LKA-only vs LFA

Same message and path. The MDPS responds the same per count in both. LKA-only mode just makes short, larger corrections near the lane edge (2-8 s interventions).

### What op wanted, by speed (before this branch)

Rise rate op's controller wanted, averaged over 250 ms, in counts at 270:

| Speed | p50 | p90 | p99 | above 4/frame |
|---|---|---|---|---|
| 0-20 kph | 0.7 | 3.8 | 12.3 | 9.3% |
| 20-40 | 0.5 | 2.3 | 7.0 | 4.1% |
| 40-60 | 0.4 | 1.6 | 4.7 | 1.6% |
| 60-80 | 0.4 | 1.5 | 4.3 | 1.3% |
| 80-100 | 0.5 | 1.6 | 4.3 | 1.2% |
| 100-140 | 0.5 | 1.4 | 3.2 | 0.5% |

Above 40 kph a rise of 4 covers ~99%. Tight low-speed turns want more.

## What this branch sends (ccnc-port-damping)

Applies to CCNC cars where the camera does LFA (`CCNC` and not `CANFD_LKA_STEER_MSG`). Other cars are unchanged.

| Item | Before | Now |
|---|---|---|
| STEER_MAX | 270 | 384 |
| Rise / fall per frame | 2 / 3 | 4 / 4 (stock is symmetric) |
| Panda safety | 270 / 2 / 3 | 384 / 4 / 4 for the CCNC param, real-time limit unchanged at 112 |
| Damping_Gain | 100 | stock speed table while steering, 100 otherwise |
| 0x12A `LKA_OptUsmSta` | 2 | 0 (stock always sends 0) |
| 0x12A `LKA_SysIndReq` | LKAS icon | 0 (stock sends 2/3 only in LKA-only mode) |
| 0x12A `LKA_RcgSta` | 0 | 1 left, 2 right, 3 both lines while steering, 0 otherwise |
| 0x12A byte 9 | 0 | 9 (`LKA_OnOffEquip2Sta` 1 = LFA, `NEW_SIGNAL_1` 2) |
| 0x1E0 | not sent on CCNC cars | sent at 20 Hz: `HDA_LFA_SymSta` from MADS, `HDA_OptUsmSta` 2, `HDA_CntrlModSta` 0 |
| Camera fault hiding | always on | removed; the camera stays healthy (see "Camera faults: cause and fix") |

Against real stock frames from the stock Sonata drive, our 0x12A matches on 2,560 of 2,562 frames (excluding checksum, counter, torque, damping and the DAW chime bit). 0x1E0 matches except the camera's own hands-off pop-ups and LFA auto-off, which op leaves to its own driver monitoring.

### Limits and headroom

comma's `test_lateral_limits.py` turns rates into lateral jerk over 0.5 s: (rise / STEER_MAX) x 50 frames x max lateral accel / 0.5 s must be <= 3.0 m/s^3, fall <= 5.0. All CCNC cars pass at 384 / 4 / 4:

| Car | Max lat accel | Rise jerk | Fall jerk |
|---|---|---|---|
| Sonata HEV 2024 | 2.06 | 2.15 | 2.15 |
| Sonata 2024 | 2.13 | 2.21 | 2.21 |
| Kona / K5 / K4 / Sorento / Santa Cruz / Ioniq 5 N | 2.50 | 2.60 | 2.60 |
| Tucson 2025 family | 2.86 | 2.98 | 2.98 |

- Tucson is at its ceiling with a rise of 4. Sonata HEV could go to 5 (2.68); 6 fails (3.22).
- Once a rate reaches full torque within 0.5 s the test caps it at 2 x max lat accel, so the Sonata HEV's fall rate can be anything (4.12 <= 5.0). Tucson's fall is capped at 6.
- Panda's real-time limit (112 counts per 250 ms away from zero) caps the sustained rise at ~4.5 per frame. A rise of 5 needs it raised to 125. Falling toward zero is not limited by it.
- Raising STEER_MAX lowers jerk for the same rate, because the test uses rate / STEER_MAX.

### Driver override

op and panda use `STEER_DRIVER_ALLOWANCE` 250 and `MULTIPLIER` 2 on the hand-torque sensor (0.005 Nm per count). Up to 250 counts (1.25 Nm) op keeps full authority; beyond that its limit drops 2 counts per count. Full cancel needs 250 + STEER_MAX/2: 385 counts (1.9 Nm) at 270, 442 counts (2.2 Nm) at 384. Torque fades at the fall rate even while overridden: about 1 s from 384 to 0. A multiplier of 3 would bring full cancel back to ~1.9 Nm.

## Messages that matter for steering

| ID | Name | Role | op |
|---|---|---|---|
| 0x12A | `LFA` / `ADAS_CMD_30_10ms` | steering command | sends (camera copy blocked) |
| 0x1E0 | `LFAHDA_CLUSTER` / `ADAS_CMD_31_50ms` | cluster LFA/HDA status | sends |
| 0x161 / 0x162 | CCNC HUD | lane lines, icons, alerts, faults, vibrate | sends (camera copy with op's lane lines and position from the model) |
| 0xEA | `MDPS` / `MDPS_01_10ms` | MDPS feedback | reads; sends the camera a copy (LKA torque inactive and unavailable); the real one reaches the camera only until op's copy starts |
| 0x125 | `STEERING_SENSORS` | angle and rate | reads |
| 0x1B5 | `FR_CMR_03_50ms` | lane geometry and lead car | reads (lead distance with op longitudinal) |
| 0x7C4 | camera diagnostics | fault-code queries | passes through |

Paths that exist but are not ours on these cars: `0xCB` angle command (angle MDPS only), `0x50`/`0x110` LKAS on HDA2 cars, `ESC_StrTqReq` in 0x60 (ESC stability torque), parking angle request (private bus).

### What can be intercepted

Only traffic crossing the harness between camera (bus 2) and car (bus 0). Camera to car: the 11 camera IDs. Car to camera: everything the camera hears (0xEA, 0x7C4, wheel speeds, steering angle, buttons). Car ECU to car ECU traffic never crosses the harness (MDPS to cluster, ESC to MDPS, drive mode to MDPS) and would need a second man-in-the-middle at that ECU.

### The other camera IDs

| ID | Content | Steering use |
|---|---|---|
| 0x1B5 `FR_CMR_03_50ms` | lane line quality, departure, position, heading, curvature; lead car id, speed, distance | none |
| 0x11A `FR_CMR_01_10ms` | DAW attention level and rest warning, high beam assist lamp request, camera info and SW version | could stop DAW alerts at the source; leave HBA alone |
| 0x1FA `FR_CMR_02_100ms` | speed-limit signs, over-limit warning, `ISLA_SpdChgReq` to change cruise speed | cruise only |
| 0x200 `ADRV_0x200` | undocumented, mostly constant; two slow 3-bit values in byte 5 | none |
| 0x38C | heartbeat: checksum, counter, constant model code (F3 0F or 05) | none |

## DBC findings

### 0x12A LFA

- The official `ADAS_CMD_30_10ms` defines 16 signals, all present in ours. Ours has more (FCA, BCA, ELK, LFA button, byte 9).
- Byte 9 (bits 72-79) is not in any official file. From logs: bits 72-73 are the mode (1 LFA, 2 LKA-only, 3 fault) and bits 74-75 are model-specific (2 Sonata, 3 K5, 1 Kona HEV, 0 K4). Bit 75 is always 1 on a healthy Sonata camera. Added to the stock DBC as `NEW_SIGNAL_1 : 74|2`.
- `Info_PedtrnDst`: official is bits 84-86 little-endian, ours is 84/83/82 big-endian. Our layout fits the data (bit 82 healthy, bit 84 faulted, 85-86 never set).
- `LKA_RcgSta`: 1 left, 2 right, 3 both (official R3; the older R2 copy at 0x50 uses 1 full, 2 left, 3 right).

### 0x1E0 LFAHDA_CLUSTER

Our guessed signals were replaced with the official ones the Sonata actually uses:

| Bits | Was | Official | Values |
|---|---|---|---|
| 24-26 | `NEW_SIGNAL_4/5`, bit 26 | `HDA_OptUsmSta` | 2 HDA on; a faulted camera sends 6-7 |
| 30-31 | `NEW_SIGNAL_2`, `HDA_ICON` | `HDA_CntrlModSta` | 0 off, 1 standby, 2 active |
| 32-34 | `NEW_SIGNAL_1` | `HDA_InfoPUDis` | 5 hands-off pop-up, 6 with sound |
| 47-48 | `LFA_ICON` | `HDA_LFA_SymSta` | 0 off, 1 gray, 2 green, 3 white blink |
| 51-53 | (unknown bit) | `HDA_InfoPUDis1` | 1 LFA automatic off |

The official file also defines HDA warning sound, auto-call and 16 `HDP_*` fields; all stay 0 on the Sonata, so they were left out.

### Other DBC notes

- `CAM_0x180` is 8 bytes on every car, not 32 (fixed in the research copy only; the stock DBC still says 32, nothing uses it).
- `SCC_CONTROL` has `ZEROS_10` and `SCC_ObjSta` overlapping on bit 110; bit 110 is never set (fixed in the research copy only).
- 0x165, 0x240, 0x3C1 and 0x100 have different lengths on HDA2-style cars because the same ID carries a different message there.
- A full research DBC with every undefined camera bit mapped across six cars, plus the HDA2 camera messages, lives in `opendbc/dbc/generator/hyundai/hyundai_canfd_ccnc.dbc` (not used by code). All real frames decode and re-pack byte for byte with it.

### MDPS 0xEA (official)

- `MDPS_StrTqSnsrVal`: 0.005 Nm per count, offset -20.48.
- `MDPS_LkaToiUnblSta` (0 available, 1 unavailable) and `MDPS_LkaToiFltSta` show when the MDPS refuses torque.
- `MDPS_CurrModVal`: assist mode (0 Comfort, 1 Sport, 2 Sport+, 3 Chauffeur, 4 Mild, 5 Drift). The Sonatas never left Comfort; Ioniq 5 N, Tucson and K4 switched with the drive mode. The mode request comes from car ECUs on the MDPS's own bus, so op cannot set it.

## Camera messages with a healthy camera

Nothing in 0x161/0x162 is cleared anymore, so the camera's own messages reach the cluster. Seen so far, all stock behaviour:

| Field | Value | When | Notes |
|---|---|---|---|
| `ALERTS_3` | 8 | cruise engages or the following distance changes (every one of 111 cases across all logged routes) | the Smart Cruise distance pop-up, about 4 s |
| `ALERTS_3` | 3, 4, 7, 9, 10 | rare, also cruise-related | cleared only when op does longitudinal, since the camera's cruise is not running then |
| `ALERTS_5` | 5 | stopped in traffic with cruise holding | likely the "press resume or accelerator to go" message, about 4 s |
| `LKA_OnOffEquip2Sta` | 2 with `LKA_ICON` 3 | whole drive | camera in LKA-only mode, likely the car's lane safety setting; the gray LKA icon now shows because op no longer overrides it |

## HUD lane position and lane lines

op drives these from its own model, not the camera. `controlsd_ext` fills two `CarControlSP` fields each frame and the Hyundai car controller turns them into 0x161 values:

| Field | What it is |
|---|---|
| `laneLeftDistance` / `laneRightDistance` | model distance to the left/right lane line (m), 0 while that line is hidden |

Lane lines (`LANELINE_LEFT` / `LANELINE_RIGHT`): a line appears once its `laneLineProbs` has been above 70% for 1 s and hides once it has been below 50% for 0.5 s. On a 3.8 min drive on unmarked streets the plain 50% cut toggled the left line 148 times; this toggled it 8.

Lane position (`LANELINE_LEFT_POSITION` / `LANELINE_RIGHT_POSITION`): stock sends whole numbers 0-30 that always add up to 30 (15/15 = centred); there is no finer bit, and stock cameras only move them during a lane change. op shows the car's position whenever its LFA icon is on: `30 * left / (left + right)`, or with one line visible the last lane width seen with both. The shown value only changes once the real one is a full step away, which stops flicker between neighbouring steps without adding delay (83 changes a minute raw, 18 with it, never more than 1 step off).

Road curve (`LANELINE_CURVATURE`): not set by op; the camera's own value passes through (the Sonata camera always sends straight). Tried and dropped because none looked right on the road: stock HDA2 cameras follow their lane curvature 10 m ahead (`k + 10 * dk` from 0x1B5, x -3800: 73% exact, 94% within 1 step); the model's plan 10 m ahead (x -3250) got 59% / 85%, `desiredCurvature` 46% / 71%, and `steeringAngleDeg / 4.5` about 33%. `desiredCurvature` and `actuators.curvature` also follow the steering wheel when op is not steering, because modeld and controlsd reset them to the steering-angle curvature.

## Other cars

| Car | Notes |
|---|---|
| Sonata HEV (3 cars) | same layout and Damping_Gain table as above |
| K5 2025 | same 11 IDs; byte 9 mostly 14 while op steers; no stock LFA data |
| Kona EV / HEV 2nd gen | same 11 IDs; Kona HEV byte 9 = 6; no stock data |
| Tucson HEV 2025 | same 11 IDs plus 0xCB and 0x2BA; tightest ISO margin |
| K4 2025 | HDA2-style; stock LFA from the ADAS ECU: torque 20 / 73 / 147, different Damping_Gain |
| Ioniq 5 N | HDA2-style; all routes op-driven |

## Open questions and next tests

1. Drive with this branch (flash the panda too), reset live torque learning, check MDPS warnings near 384 and low-speed feel.
2. Max-torque step test (joystick mode, parked or walking pace): ramp rate, peak MDPS torque, `LkaToiUnblSta`/`LkaToiFltSta`, blocked frames, override feel.
3. Lateral maneuver mode (`tools/lateral_maneuvers`) on the old and new builds, compare reports.
4. Fake 0xEA to the camera: done, see "Camera faults: cause and fix". Still open: whether other CCNC cars behave the same.
5. Byte 9 bits 74-75 per model: needs op-off LFA drives from K5, Kona, Tucson.
6. Damping_Gain above 100 kph.
7. Sonata-only rise 5 / fall 7 (with panda real-time limit 125) if more response is wanted.
8. Torque tune: give the Sonata HEV 2024 its own entry if the maneuver report shows the borrowed one is off.
