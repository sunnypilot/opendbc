# SoRadGaming/opendbc: fork notes

This is sunnypilot's opendbc, carried on branch `sp-master` for one car: the 2013–2015 Honda Accord V6 (AU),
`HONDA_ACCORD_9G_AU`, platform flag `HondaFlags.ELESYS`, with a comma pedal and an aftermarket EPS-LKAS gateway board.
`SoRadGaming/sunnypilot` pins this branch through its `opendbc_repo` submodule (`.gitmodules`: this URL,
`branch = sp-master`).

The full merge guide and the per-area documents live in the sunnypilot repo, under `docs/fork/`:

* local: `S:/OP/sp-live/docs/fork/`
* GitHub: `SoRadGaming/sunnypilot` → `docs/fork/`

| document | covers |
|---|---|
| `README.md` | index, inventory with conflict risk, collision checks, merge procedure |
| `GATEWAY-UPDATE.md` | area A: updating the board's firmware from the comma over CAN |
| `LKAS-GATEWAY-PROTOCOL.md` | area B: openpilot steering the EPS through the board |
| `CAR-HONDA-ACCORD-9G-AU.md` | area C: the car |

## Where it stands (2026-09-27)

| | |
|---|---|
| upstream | `sunnypilot/opendbc` `master`, fetched to `refs/upstream/master` = `f95f996f` (2026-09-02) |
| fork point | `b9712d20` (2026-06-08, "Revert "deprecate carState.brake" for Honda Gas Interceptor (#481)") |
| fork HEAD | `cf583b37` (2026-09-22). This is `refs/heads/sp-master` on `SoRadGaming/opendbc` (checked with `git ls-remote origin sp-master`) and sunnypilot's pinned pointer. |
| fork commits | 38 |
| upstream commits not in the fork | 173 |
| files changed | 31 |

**Branches.** This fork's GitHub default branch is `master` (`fe144714`), not `sp-master`. The submodule clone fetches
only `master` (`remote.origin.fetch = +refs/heads/master:refs/remotes/origin/master`), so there is no local
`origin/sp-master`, `origin/HEAD` is `origin/master`, and the local `sp-master` has no upstream configured. Check the
remote with `git ls-remote origin sp-master` and push with `git push origin sp-master`.

```bash
git fetch --no-tags https://github.com/sunnypilot/opendbc.git +master:refs/upstream/master
git merge-base HEAD refs/upstream/master                            # b9712d20
git rev-list --count b9712d20..HEAD                                 # 38
git rev-list --count HEAD..refs/upstream/master                     # 173
git diff --name-status b9712d20 HEAD                                # the 31 files
git rev-list --count b9712d20..refs/upstream/master -- <file>       # conflict risk of one file
git grep -n -E "FORK(\(|:)" -- opendbc | wc -l                      # 18 markers (plain "FORK" also hits Tesla DBC strings)
```

## Files by area

The number after each file is the count of upstream commits touching it since the fork point. **Bold** marks a file
that conflicted in a trial merge with `f95f996f`.

### A. Gateway update: board firmware identity

* **`opendbc/car/honda/carstate.py`** (3): registers `GW_VERSION` (`0x707`) and `GW_BUILD` (`0x70F`) liveness-exempt
  (`float("nan")`). The file is also in B and C.
* `opendbc/sunnypilot/car/honda/carstate_ext.py` (1): `_update_linbus_firmware()` latches the `fw*` fields. The file is
  also in B and C.
* `opendbc/car/structs.py` (2): `CarStateSP.LinbusGateway.fwValid`, `fwGitHash`, `fwDirty`, `fwAppSlot`,
  `fwBootloader`, `fwReadOnly`, `boardUid` and `fwBuildValid`. The **names** must match sunnypilot's `custom.capnp`:
  card converts with `new_message(**asdict)`, so a missing or extra name raises on a drive. Order is not load-bearing.
  The sunnypilot test `test_capnp_and_dataclass_agree` pins the eight `fw*` names and their relative order only.
* `opendbc/dbc/generator/honda/_sunnypilot_linbus_gw.dbc` (0): `GW_VERSION` and `GW_BUILD`, including
  `BUILD_SP_FRESH`.

### B. LKAS gateway protocol

* **`opendbc/car/honda/carcontroller.py`** (4):
  * `BRAKE_RELEASE_FRAMES` / `brake_release_scale()`, the brake-release ceiling;
  * `serial_gateway` LDW bits on `0x0E4`;
  * `SP_HUD_STATUS` sent on bus 0 with `lat_ready = CC_SP.mads.enabled or CC.latActive` and `op_state`;
  * `LKAS_HUD` is not sent. The board's Stage 10 image owns `0x33D` on bus 0 (board `df42a0d`), and openpilot reads `LKAS_PROBLEM` back from it.
* **`opendbc/car/honda/hondacan.py`** (4): `create_steering_control(..., serial_gateway, ldw_left, ldw_right)`,
  `SP_HUD_PROTOCOL_VERSION = 3`, `SP_OP_STATE_*`, `SP_HUD_MAX_TORQUE = 0`, `create_sp_hud_status()`.
* **`opendbc/car/honda/carstate.py`** (3): registers `GW_ACTIVE`, `GW_STEER_GRANT` and `EPS_LIN_RAW`
  liveness-exempt, and changes the call to `CarStateExt.update(self, ret, ret_sp, can_parsers)`.
* `opendbc/sunnypilot/car/honda/carstate_ext.py` (1):
  * `_update_linbus_gateway()` decodes `0x704`. It sets `linbusGateway.present = True` every frame on every
    `HONDA_ELESYS` car, board or no board;
  * `_update_linbus_grant()` decodes `0x70B`;
  * `_update_driver_torque_validity()` and `_eps_lin_driver_torque_valid()` substitute `0x700 EPS_LIN_RAW` when
    `0x18F` is latched (`SERIAL_TORQUE_TO_CAN = -64.5`);
  * constants `LINBUS_*_STALE_FRAMES`, `STEER_TORQUE_STALE_FRAMES`, `GRANT_STATES_STEERING`, `GRANT_RETRY_KEY_CYCLE`.
* `opendbc/car/structs.py` (2): `CarControlSP.LateralControl`, `CarStateSP.driverTorqueStale`, and the
  `CarStateSP.LinbusGateway` control fields.
* `opendbc/dbc/generator/honda/_sunnypilot_linbus_gw.dbc` (0): `0x500 SP_HUD_STATUS`, `0x700 EPS_LIN_RAW`,
  `0x704 GW_ACTIVE`, `0x70B GW_STEER_GRANT`. It must not use a signal named `COUNTER` or `CHECKSUM` on the board's
  frames (a `honda_` DBC makes the parser validate a Honda checksum on those names); `0x500`, which openpilot sends,
  does use them.
* `opendbc/dbc/generator/honda/_steering_control_e.dbc` (0): `0x0E4` byte 2 (`LDW_RIGHT`, `LDW_LEFT`,
  `SET_ME_X00_3` held at zero), and `STEER_STATUS.STEER_CONTROL_ACTIVE`.
* `opendbc/safety/modes/honda.h` (0): `0x500` on bus 0 in the stand-down TX lists. The file is also in C.
* `opendbc/sunnypilot/car/honda/test_dynamic_tuning_integration.py` (0): sections [7], [8] and [10]–[15].

### C. The car

* **`opendbc/car/honda/values.py`** (8):
  * `HondaFlags.ELESYS = 1024`, `HondaSafetyFlags.ELESYS_SCM_STANDDOWN = 32`, `CAR.HONDA_ACCORD_9G_AU`,
    `HONDA_ELESYS`;
  * `STEER_THRESHOLD` 600;
  * FW query `non_essential_ecus`.
* **`opendbc/car/honda/interface.py`** (3):
  * gearbox `0x188` → automatic;
  * `longitudinalActuatorDelay` 0.6, `vEgoStopping` 0.8, `stopAccel` -0.8;
  * `steerActuatorDelay` 0.38, `steerAtStandstill`;
  * the stand-down safety parameter;
  * `minEnableSpeed` 19 mph.
* **`opendbc/car/honda/carcontroller.py`** (4):
  * `compute_gb_honda_elesys()`;
  * `brake_pump_hysteresis_elesys()` and `ELESYS_PUMP_*`;
  * dynamic-tuner hooks (`hill_accel`/`adjust_accel`, `brake_gain`, `wind_scale`) and the 32-count brake release.
    These are gated by the tuner (toggle on, and Nidec with openpilot longitudinal), not by `HONDA_ELESYS`;
  * `SCM_BUTTONS` re-sent on `CAN.camera` every 4th frame, only when `openpilotLongitudinalControl`;
  * `pcm_accel` computed from `adjust_accel` (pitch feed-forward, 0 with the tuner off), and a `FORK:` comment
    explaining why there is no PCM crossfade (history in sunnypilot `CHANGELOG-elesys.md` section 14). No upstream code
    was removed there.
* **`opendbc/car/honda/hondacan.py`** (4): the `BRAKE_COMMAND` units bit (`is_metric`) and
  `create_scm_buttons_no_cruise()`, which copies `SCM_BUTTONS` with `MAIN_ON = 0` and `CRUISE_BUTTONS = 0`.
* **`opendbc/car/honda/carstate.py`** (3): `update_gear_elesys()` / `SPORT_DWELL`; the ELESYS `stockAeb`, which also
  sets `carFaultedNonCritical = True` when stock AEB fires with `ACC_HUD.ACC_ON == 0`; `LKAS_PROBLEM` from bus 0;
  `scm_buttons` and `econ_on`.
* `opendbc/sunnypilot/car/honda/carstate_ext.py` (1): `fuelGauge` from `SCM_BUTTONS.FUEL_LEVEL` / 105.
* `opendbc/car/honda/fingerprints.py` (3): FW versions for fwdRadar and srs.
* `opendbc/car/honda/radar_interface.py` (4): Elesys radar parser and fault states.
* `opendbc/car/car_helpers.py` (3): the `skip_fw_query` argument on `fingerprint()` and `get_car()`.
* `opendbc/car/tests/routes.py` (8): test route `15646e8515eda1a7/00000019--dd0700eac9`.
* `opendbc/car/torque_data/substitute.toml` (0): `HONDA_ACCORD_9G_AU = HONDA_ACCORD`.
* `opendbc/sunnypilot/car/car_list.json` (5): `"Honda Accord 2013-15"`. Regenerate it with
  `python opendbc/sunnypilot/car/platform_list.py`.
* `opendbc/sunnypilot/car/honda/dynamic_tuning.py` (0): `HondaDynamicTuner`. Its params are `HondaDynamicTuningEnabled`
  and `HondaDynPedalGain0`–`5`, `HondaDynWindFactor`, `HondaDynBrakeGain`. `_is_applicable()` limits it to
  `openpilotLongitudinalControl and carFingerprint not in HONDA_BOSCH`.
* `opendbc/sunnypilot/car/honda/gas_interceptor.py` (0): imports `HONDA_ELESYS`; `ELESYS_GAS_BP`/`ELESYS_GAS_V`,
  `elesys_gas_multiplier()`, and the `tuner` hooks (`pedal_gain_at`, `update_pedal`).
* `opendbc/safety/modes/honda.h` (0): `ELESYS_SCM_STANDDOWN` TX lists (which deliberately leave out `0x33D`
  `LKAS_HUD`), AEB bit 43, the `pcm_gas` 198 exception, blocking `0x1A6` from bus 0 to bus 2, and
  `honda_bosch_init()` resetting `honda_elesys_scm_standdown = false`.
* `opendbc/safety/tests/test_honda.py` (0): `TestHondaElesysScmStanddownSafety`,
  `TestHondaElesysStanddownGasInterceptorSafety`.
* `opendbc/safety/tests/common.py` (3): scanned-range exceptions for these tests.
* DBC: `honda_accord_au_2015_can.dbc`, `_honda_elesys_base.dbc`, `_lkas_hud_4byte.dbc`,
  `_nidec_scm_group_a_elesys.dbc`, `_gearbox_legacy.dbc`, `honda_accord_2015au_radar.dbc` (all 0).
  `_honda_elesys_base.dbc` is a *modified* copy of `_honda_common.dbc`. The intended differences are: 7-byte
  `CAMERA_MESSAGES` (`0x35E`) and `STALK_STATUS` (`0x374`, no `WIPER_SWITCH`, `COUNTER`/`CHECKSUM` at 53/51), no
  `STEER_MOTOR_TORQUE.UNKNOWN_TORQUE_STATE_BIT`, `CM_ BO_` for 304/316, and a header comment. Anything else in
  `diff _honda_common.dbc _honda_elesys_base.dbc` is drift from upstream.
* **Shared** DBC fragments that other Nidec cars also use: `_nidec_common.dbc` (read-only `CMBS_BRAKE`,
  `CMBS_DISABLED`, `AEB_REQ_3`) and `_nidec_scm_group_a.dbc` (read-only `CMBS_BUTTON`), both 0.
* Tests: `opendbc/car/honda/tests/test_elesys.py`, `opendbc/sunnypilot/car/honda/test_dynamic_tuning.py`,
  `test_dynamic_tuning_integration.py`.

## Merging upstream: the short version

Merge opendbc **before** sunnypilot, push `sp-master`, and only then move sunnypilot's submodule pointer.

A trial merge with `f95f996f` conflicts in five files: `values.py`, `hondacan.py`, `carstate.py`, `interface.py` and
`carcontroller.py`. Resolve them in that order. The causes are upstream's:

* **`HONDA_*` sets.** Three are gone: `HONDA_NIDEC_ALT_PCM_ACCEL`, `HONDA_NIDEC_ALT_SCM_MESSAGES` and
  `HONDA_BOSCH_TJA_CONTROL`. The remaining `HONDA_BOSCH*` sets are now
  `frozenset(c for c in CAR if c.config.flags & ...)` after `DBC = CAR.create_dbc_map()`. Re-add `HONDA_ELESYS` there
  in the same style. `radar_interface.py`, `gas_interceptor.py`, `carstate_ext.py` and `test_elesys.py` import it and
  merge without conflict, so they fail to import if it is missing.
* **New signatures.** `compute_gas_brake(accel, speed, CP)` now takes `CP`, and `create_brake_command(...)` no longer
  takes `car_fingerprint`. Append the fork's `is_metric` and `elesys` arguments at the end, by keyword, and update
  `test_elesys.py`.
* **Fork lines outside the markers.** In `carcontroller.py`, keep `compute_gb_honda_elesys()` (fork side of hunk 2)
  and `adjust_accel = accel + hill_accel` (fork side of hunk 4; the later `pcm_accel` line reads it), and rewrite the
  context line `elif fingerprint in HONDA_ELESYS:` to `elif CP.carFingerprint in HONDA_ELESYS:`. In `hondacan.py`,
  change `... if car_fingerprint in HONDA_ELESYS else 1` to `... if elesys else 1`. `ruff check` (F821) catches any
  that survive.
* **Behaviour change.** Upstream `4455464a` sets `minEnableSpeed = -1` for gas-interceptor Hondas, which overrides this
  car's 19 mph. Decide on it deliberately.
* **A test that keeps passing but stops testing.** `test_dynamic_tuning_integration.py` section [15] passes
  `driver_torque_stale` positionally; after the merge it lands in upstream's new `left_edge_detected`. Pass it by
  keyword.

The complete steps, checks and on-car verification are in the sunnypilot `docs/fork/README.md`.

## Tests

Run from this directory with `PYTHONPATH=.`:

```bash
python -m unittest opendbc.car.honda.tests.test_elesys           # 52 tests
python -m unittest -k Elesys opendbc.safety.tests.test_honda     # needs a C compiler (libsafety is built on import)
python -m unittest opendbc.sunnypilot.car.tests.test_car_list
python opendbc/sunnypilot/car/honda/test_dynamic_tuning.py
python opendbc/sunnypilot/car/honda/test_dynamic_tuning_integration.py
./test.sh                                                        # uv lock check, then ruff, ty, codespell, cpplint, MISRA, unittest-parallel
```

**Known failure at `cf583b37`.** In `test_dynamic_tuning_integration.py`, section [10] fails the check "v3: enabled,
lateral available, not asking -> READY and LAT_READY". The test predates `43a98b9d`, which made `LAT_READY` follow
`CC_SP.mads.enabled`. The fix is in the test.

The two `test_dynamic_tuning*.py` files run their checks at import time and `sys.exit(1)` on failure.
`unittest-parallel` in `./test.sh` discovers and imports them, and unittest's loader reports an import-time
`SystemExit` as a failed import. By inspection, not run: the known failure above therefore fails `./test.sh` today.
Section [15] needs `openpilot` importable and prints SKIP when run from this repo alone.
