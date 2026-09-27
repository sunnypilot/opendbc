# SoRadGaming/opendbc: fork notes

This is sunnypilot's opendbc, carried on branch `sp-master` for one car: the 2013–2015 Honda Accord V6 (AU),
`HONDA_ACCORD_9G_AU`, platform flag `HondaFlags.ELESYS`, with a comma pedal and an aftermarket EPS-LKAS gateway board.
`SoRadGaming/sunnypilot` pins this branch through its `opendbc_repo` submodule (`.gitmodules`: this URL,
`branch = sp-master`).

The full merge guide and the per-area documents live in the sunnypilot repo, under `docs/fork/`:

* local: `S:/OP/sp-live/docs/fork/` (WSL: `~/sp-merge/docs/fork/`)
* GitHub: `SoRadGaming/sunnypilot` → `docs/fork/`

| document | covers |
|---|---|
| `README.md` | index, inventory with conflict risk, collision checks, merge procedure |
| `GATEWAY-UPDATE.md` | area A: updating the board's firmware from the comma over CAN |
| `LKAS-GATEWAY-PROTOCOL.md` | area B: openpilot steering the EPS through the board |
| `CAR-HONDA-ACCORD-9G-AU.md` | area C: the car |
| `UPSTREAM-2026-09.md` | the 2026-09 upstream sync: what arrived, and what it does on this car |

## Where it stands (2026-09-27, after the upstream sync)

| | |
|---|---|
| upstream | `sunnypilot/opendbc` `master`, fetched to `refs/upstream/master` = `f95f996f` (2026-09-02) |
| fork point | **`f95f996f`**: upstream's head, merged on 2026-09-27. The previous fork point was `b9712d20` (2026-06-08). |
| fork HEAD | `8bd6e314` (upstream `f95f996f` merged into the fork's `c61cfd9b`), then the review-fix commit. That becomes `sp-master` and sunnypilot's pinned pointer. |
| upstream commits not in the fork | 0 on 2026-09-27 |
| fork commits since the fork point | 40 at `8bd6e314` (39 excluding merges). A merge keeps history, so this counts every fork commit since `b9712d20`; use the diff to see what the fork carries. |
| files changed | 31, plus this file |

**Branches.** This fork's GitHub default branch is `master` (`fe144714`), not `sp-master`. The submodule clone in the
Windows checkout (`S:/OP/sp-live/opendbc_repo`) fetches only `master`
(`remote.origin.fetch = +refs/heads/master:refs/remotes/origin/master`), so there is no local `origin/sp-master`,
`origin/HEAD` is `origin/master`, and the local `sp-master` has no upstream configured. The WSL clone used for the sync
fetches every branch. Check the remote with `git ls-remote origin sp-master` and push with `git push origin sp-master`
(`GIT_LFS_SKIP_PUSH=1` in front if the LFS pre-push hook complains about objects the clone never downloaded).

```bash
git fetch --no-tags https://github.com/sunnypilot/opendbc.git +master:refs/upstream/master
git merge-base HEAD refs/upstream/master                            # f95f996f until upstream moves on
git rev-list --count refs/upstream/master..HEAD                     # 40 at 8bd6e314
git rev-list --count HEAD..refs/upstream/master                     # 0 on 2026-09-27: the next merge's size
git diff --name-status refs/upstream/master HEAD                    # the 31 files and this one
MB=$(git merge-base HEAD refs/upstream/master)
git rev-list --count $MB..refs/upstream/master -- <file>            # conflict risk of one file
git grep -n -E "FORK(\(|:)" -- opendbc | wc -l                      # 31 markers (18 before the sync)
```

## Files by area

The number after each file is the count of upstream commits touching it since the fork point, which is **0 for every
file** right after the sync; re-run the command above before the next merge. **Bold** marks a file that conflicted in
the 2026-09 merge.

### A. Gateway update: board firmware identity

* **`opendbc/car/honda/carstate.py`** (0): registers `GW_VERSION` (`0x707`) and `GW_BUILD` (`0x70F`) liveness-exempt
  (`float("nan")`). The file is also in B and C.
* `opendbc/sunnypilot/car/honda/carstate_ext.py` (0): `_update_linbus_firmware()` latches the `fw*` fields. The file is
  also in B and C.
* `opendbc/car/structs.py` (0): `CarStateSP.LinbusGateway.fwValid`, `fwGitHash`, `fwDirty`, `fwAppSlot`,
  `fwBootloader`, `fwReadOnly`, `boardUid` and `fwBuildValid`. The **names** must match sunnypilot's `custom.capnp`:
  card converts with `new_message(**asdict)`, so a missing or extra name raises on a drive. Order is not load-bearing.
  The sunnypilot test `test_capnp_and_dataclass_agree` pins the eight `fw*` names and their relative order only.
* `opendbc/dbc/generator/honda/_sunnypilot_linbus_gw.dbc` (0): `GW_VERSION` and `GW_BUILD`, including
  `BUILD_SP_FRESH`.

### B. LKAS gateway protocol

* **`opendbc/car/honda/carcontroller.py`** (0):
  * `BRAKE_RELEASE_FRAMES` / `brake_release_scale()`, the brake-release ceiling;
  * `serial_gateway` LDW bits on `0x0E4`;
  * `SP_HUD_STATUS` sent on bus 0 with `lat_ready = CC_SP.mads.enabled or CC.latActive` and `op_state`;
  * `LKAS_HUD` is not sent. The board's Stage 10 image owns `0x33D` on bus 0 (board `df42a0d`), and openpilot reads `LKAS_PROBLEM` back from it.
* **`opendbc/car/honda/hondacan.py`** (0): `create_steering_control(..., serial_gateway, ldw_left, ldw_right)`,
  `SP_HUD_PROTOCOL_VERSION = 3`, `SP_OP_STATE_*`, `SP_HUD_MAX_TORQUE = 0`, `create_sp_hud_status()`.
* **`opendbc/car/honda/carstate.py`** (0): registers `GW_ACTIVE`, `GW_STEER_GRANT` and `EPS_LIN_RAW`
  liveness-exempt, and changes the call to `CarStateExt.update(self, ret, ret_sp, can_parsers)`.
* `opendbc/sunnypilot/car/honda/carstate_ext.py` (0):
  * `_update_linbus_gateway()` decodes `0x704`. It sets `linbusGateway.present = True` every frame on every
    `HONDA_ELESYS` car, board or no board;
  * `_update_linbus_grant()` decodes `0x70B`;
  * `_update_driver_torque_validity()` and `_eps_lin_driver_torque_valid()` substitute `0x700 EPS_LIN_RAW` when
    `0x18F` is latched (`SERIAL_TORQUE_TO_CAN = -64.5`);
  * constants `LINBUS_*_STALE_FRAMES`, `STEER_TORQUE_STALE_FRAMES`, `GRANT_STATES_STEERING`, `GRANT_RETRY_KEY_CYCLE`.
* `opendbc/car/structs.py` (0): `CarControlSP.LateralControl`, `CarStateSP.driverTorqueStale`, and the
  `CarStateSP.LinbusGateway` control fields.
* `opendbc/dbc/generator/honda/_sunnypilot_linbus_gw.dbc` (0): `0x500 SP_HUD_STATUS`, `0x700 EPS_LIN_RAW`,
  `0x704 GW_ACTIVE`, `0x70B GW_STEER_GRANT`. It must not use a signal named `COUNTER` or `CHECKSUM` on the board's
  frames (a `honda_` DBC makes the parser validate a Honda checksum on those names); `0x500`, which openpilot sends,
  does use them.
* `opendbc/dbc/generator/honda/_steering_control_e.dbc` (0): `0x0E4` byte 2 (`LDW_RIGHT`, `LDW_LEFT`,
  `SET_ME_X00_3` held at zero), and `STEER_STATUS.STEER_CONTROL_ACTIVE`.
* `opendbc/safety/modes/honda.h` (0): `0x500` on bus 0 in the stand-down TX lists. The file is also in C.
* `opendbc/safety/tests/common.py` (0): `0x500` is exempt between the two `TestHondaElesys*` classes only (both
  stand-down lists carry it), tagged `FORK(LKAS-GATEWAY)`. Added in the sync; it fixed two cross-mode failures that
  predate it.
* `opendbc/sunnypilot/car/honda/test_dynamic_tuning_integration.py` (0): sections [7], [8] and [10]–[15].

### C. The car

* **`opendbc/car/honda/values.py`** (0):
  * `HondaFlags.ELESYS = 1024`, `HondaSafetyFlags.ELESYS_SCM_STANDDOWN = 32`, `CAR.HONDA_ACCORD_9G_AU`;
  * `HONDA_ELESYS = frozenset(c for c in CAR if c.config.flags & HondaFlags.ELESYS)`, after the `HONDA_BOSCH*` sets
    (it was `CAR.with_flags(...)` before the sync);
  * `STEER_THRESHOLD` 600;
  * FW query `non_essential_ecus`.
* **`opendbc/car/honda/interface.py`** (0):
  * gearbox `0x188` → automatic;
  * `longitudinalActuatorDelay` 0.6, `stopAccel` -0.8. **No `vEgoStopping`**: upstream deprecated it (assigning it
    raises), and the car's 0.8 m/s stopping speed now lives in sunnypilot's
    `openpilot/sunnypilot/selfdrive/controls/lib/stopping_tune.py`;
  * `steerActuatorDelay` 0.38, `steerAtStandstill`;
  * the stand-down safety parameter;
  * `minEnableSpeed` 19 mph, and in `_get_params_sp()` the exemption that keeps it: upstream `4455464a` sets `-1` for
    every gas-interceptor car, `candidate not in HONDA_ELESYS` keeps this one at 19 mph.
* **`opendbc/car/honda/carcontroller.py`** (0):
  * `compute_gb_honda_elesys()`, reached through `compute_gas_brake(accel, speed, CP)`'s
    `elif CP.carFingerprint in HONDA_ELESYS`;
  * `brake_pump_hysteresis_elesys()` and `ELESYS_PUMP_*`;
  * dynamic-tuner hooks (`hill_accel`/`adjust_accel`, `brake_gain`, `wind_scale`) and the 32-count brake release.
    These are gated by the tuner (toggle on, and Nidec with openpilot longitudinal), not by `HONDA_ELESYS`;
  * `SCM_BUTTONS` re-sent on `CAN.camera` every 4th frame, only when `openpilotLongitudinalControl`;
  * `pcm_accel` computed from `adjust_accel` (pitch feed-forward, 0 with the tuner off), and a `FORK:` comment
    explaining why there is no PCM crossfade (history in sunnypilot `CHANGELOG-elesys.md` section 14). No upstream code
    was removed there.
* **`opendbc/car/honda/hondacan.py`** (0): the `BRAKE_COMMAND` units bit, through two keyword arguments at the end of
  upstream's signature, `create_brake_command(..., stock_brake, CP_SP, is_metric=True, elesys=False)`, and
  `create_scm_buttons_no_cruise()`, which copies `SCM_BUTTONS` with `MAIN_ON = 0` and `CRUISE_BUTTONS = 0`.
* **`opendbc/car/honda/carstate.py`** (0): `update_gear_elesys()` / `SPORT_DWELL`; the ELESYS `stockAeb`, which also
  sets `carFaultedNonCritical = True` when stock AEB fires with `ACC_HUD.ACC_ON == 0`; `LKAS_PROBLEM` from bus 0,
  inside upstream's `if not (self.CP.flags & HondaFlags.BOSCH):`; `scm_buttons` and `econ_on`.
* `opendbc/sunnypilot/car/honda/carstate_ext.py` (0): `fuelGauge` from `SCM_BUTTONS.FUEL_LEVEL` / 105.
* `opendbc/car/honda/fingerprints.py` (0): FW versions for fwdRadar and srs.
* `opendbc/car/honda/radar_interface.py` (0): Elesys radar parser and fault states.
* `opendbc/car/car_helpers.py` (0): the `skip_fw_query` argument on `fingerprint()` and `get_car()`.
* `opendbc/car/tests/routes.py` (0): test route `15646e8515eda1a7/00000019--dd0700eac9`.
* `opendbc/car/torque_data/substitute.toml` (0): `HONDA_ACCORD_9G_AU = HONDA_ACCORD`.
* `opendbc/sunnypilot/car/car_list.json` (0): `"Honda Accord 2013-15"`. Regenerate it with
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
* `opendbc/safety/tests/common.py` (0): scanned-range exceptions for these tests (`0x30C`, `0x1A6`).
* DBC: `honda_accord_au_2015_can.dbc`, `_honda_elesys_base.dbc`, `_lkas_hud_4byte.dbc`,
  `_nidec_scm_group_a_elesys.dbc`, `_gearbox_legacy.dbc`, `honda_accord_2015au_radar.dbc` (all 0).
  `_honda_elesys_base.dbc` is a *modified* copy of `_honda_common.dbc`. The intended differences are: 7-byte
  `CAMERA_MESSAGES` (`0x35E`) and `STALK_STATUS` (`0x374`, no `WIPER_SWITCH`, `COUNTER`/`CHECKSUM` at 53/51), no
  `STEER_MOTOR_TORQUE.UNKNOWN_TORQUE_STATE_BIT`, `CM_ BO_` for 304/316, and a header comment. Anything else in
  `diff _honda_common.dbc _honda_elesys_base.dbc` is drift from upstream; there was none at `f95f996f`.
* **Shared** DBC fragments that other Nidec cars also use: `_nidec_common.dbc` (read-only `CMBS_BRAKE`,
  `CMBS_DISABLED`, `AEB_REQ_3`) and `_nidec_scm_group_a.dbc` (read-only `CMBS_BUTTON`), both 0.
* Tests: `opendbc/car/honda/tests/test_elesys.py`, `opendbc/sunnypilot/car/honda/test_dynamic_tuning.py`,
  `test_dynamic_tuning_integration.py`.

## Merging upstream: the short version

Merge opendbc **before** sunnypilot, push `sp-master`, and only then move sunnypilot's submodule pointer.

**What the 2026-09 merge did.** Merging `f95f996f` into `c61cfd9b` conflicted in five files: `values.py`,
`hondacan.py`, `carstate.py`, `interface.py` and `carcontroller.py`, resolved in that order. The causes were upstream's,
and the fork's code now has this shape:

* **`HONDA_*` sets.** Upstream removed `HONDA_NIDEC_ALT_PCM_ACCEL`, `HONDA_NIDEC_ALT_SCM_MESSAGES` and
  `HONDA_BOSCH_TJA_CONTROL`, and made the remaining `HONDA_BOSCH*` sets `frozenset(c for c in CAR if c.config.flags & ...)`
  after `DBC = CAR.create_dbc_map()`. `HONDA_ELESYS` is re-created there in the same style. `radar_interface.py`,
  `gas_interceptor.py`, `carstate_ext.py` and `test_elesys.py` import it and merge without conflict, so they fail to
  import if it is missing.
* **New signatures.** `compute_gas_brake(accel, speed, CP)` takes `CP`, and `create_brake_command(...)` no longer
  takes `car_fingerprint`. The fork's `is_metric` and `elesys` are at the end, by keyword, and `test_elesys.py` uses
  both signatures.
* **Fork lines outside the markers.** In `carcontroller.py`, `compute_gb_honda_elesys()` (fork side of hunk 2) and
  `adjust_accel = accel + hill_accel` (fork side of hunk 4; the later `pcm_accel` line reads it) were kept, and the
  context line `elif fingerprint in HONDA_ELESYS:` became `elif CP.carFingerprint in HONDA_ELESYS:`. In `hondacan.py`,
  `... if car_fingerprint in HONDA_ELESYS else 1` became `... if elesys else 1`. `ruff check` (F821) catches any that
  survive.
* **A deprecated field.** `ret.vEgoStopping = 0.8` would raise on upstream's `car.capnp`; it is deleted, and sunnypilot
  carries the car's stopping speed and ramp in `stopping_tune.py`.
* **Behaviour.** Upstream `4455464a` sets `minEnableSpeed = -1` for gas-interceptor Hondas; `_get_params_sp()` exempts
  `HONDA_ELESYS`, so this car keeps 19 mph.
* **A test that kept passing but stopped testing.** `test_dynamic_tuning_integration.py` section [15] passed
  `driver_torque_stale` positionally, where upstream now has `left_edge_detected`. It passes it by keyword.

Next time: the same five files are where upstream's Honda work lands. Re-run the conflict-risk command first. The
complete steps, checks and on-car verification are in the sunnypilot `docs/fork/README.md`.

## Tests

Run from this directory with `PYTHONPATH=.` (in the sunnypilot venv, which has opendbc's dependencies):

```bash
python -m unittest opendbc.car.honda.tests.test_honda opendbc.car.honda.tests.test_elesys   # 53 tests (52 in test_elesys)
python -m unittest opendbc.safety.tests.test_honda                                          # builds libsafety; 942 run, OK (skipped=69)
python -m unittest opendbc.car.tests.test_car_interfaces -k HONDA_ACCORD_9G_AU
python -m unittest discover -s opendbc/sunnypilot/car -t .                                  # 23 tests, including the integration script
python opendbc/sunnypilot/car/honda/test_dynamic_tuning.py
python opendbc/sunnypilot/car/honda/test_dynamic_tuning_integration.py                      # §15 SKIPs without openpilot on PYTHONPATH
python -m unittest discover                                                                 # 9493 run, OK (skipped=1268) after the sync
./test.sh                                                                                   # uv lock check, then ruff, ty, codespell, cpplint, MISRA, unittest-parallel
```

**No known failure after the sync.** The section [10] check that failed before it ("v3: enabled, lateral available,
not asking -> READY and LAT_READY") predated `43a98b9d`, which made `LAT_READY` follow `CC_SP.mads.enabled`. The test
now sets `mads.enabled` for that case and adds a MADS-off case that expects `LAT_READY` clear. The two cross-mode
failures of `test_tx_hook_on_wrong_safety_mode` on `0x500` between the Elesys stand-down modes are fixed by the
`common.py` exemption.

**The tuner scripts still run at import.** `test_dynamic_tuning_integration.py` runs its checks when it is imported,
but calls `sys.exit(1)` only under `__main__`, and `TestDynamicTuningIntegration.test_all_checks_pass` asserts that
nothing failed, so `unittest-parallel` discovery reports a real test instead of an import error. It still monkeypatches
`dt._open_params` at import. `test_dynamic_tuning.py` still calls `sys.exit(1)` at import on a failure; it passes today,
but a failure would show up as a module that failed to import.

`ty check` reports 4 `invalid-assignment` errors in the tuner scripts; they predate the sync.
