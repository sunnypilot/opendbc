# Experimental WP MAIN-on holdoff

Status, September 29, 2026: **offline prototype, not a validated fix or installation candidate**. A second kind of captured EPS fault is outside this timer's coverage. No comma or WP firmware has been installed or flashed. No upstream PR has been submitted.

## Purpose and scope

On a WP-equipped `JEEP_GRAND_CHEROKEE_2019` platform, the existing controller can request LKAS immediately after ACC MAIN becomes available when its older falling-edge guard has already expired. This prototype requires 0.7 seconds of continuous, valid MAIN availability before it permits that request.

The runtime change is limited to the Chrysler controller and its Sunnypilot extension. It applies only when that platform also has `NO_MIN_STEERING_SPEED`, the existing WP detection flag. The generic `0x4FF` beacon cannot distinguish Basic, Advanced, and custom firmware. This branch is an explicit experiment for the reported Advanced installation, not a claim of broad WP compatibility.

The timer observes every 100 Hz control update, including ticks without a steering transmission. MAIN off, invalid CAN, or an EPS fault resets it. `CC.latActive` is still required. While waiting, normal 50 Hz steering packets continue with request off and torque zero; counters and checksums continue normally. Existing torque limits, torque ramping and the greater-than-200-frame LKAS falling-edge guard remain in force. CANCEL/SET with MAIN still available and MADS active does not restart the timer.

The [jvePilot reference](https://github.com/j-vanetten/openpilot/blob/65b85054245b4acb05ff933d266999439376ed18/opendbc_repo/opendbc/car/chrysler/carcontroller.py#L115-L144) delays a requested LKAS activation by 70 control frames. Its timer starts from a different event and its paired WP protocol also differs. The 0.7-second value here is provisional; it is not an EPS readiness specification or a proven cure.

## New evidence limits the hypothesis

Original first-segment logs showed a moving MAIN-on fault and a successful stationary comparison. Direct device access subsequently supplied compact logs for all 16 segments of those four routes, plus full-resolution segments around two additional faults. Route labels below are anonymized; raw logs and identifiers are not included in either repository.

| Recording | Observation in the original, unmodified software |
| --- | --- |
| A | MAIN off for about 16.3 seconds, then on at about 10.5 mph. LKAS request follows MAIN by 22.6 ms; raw EPS state 4 follows MAIN by 0.840 s. |
| B | Moving MAIN-on at about 4.1 mph. Request follows MAIN by 23.1 ms; raw EPS state 4 follows MAIN by about 0.891 s. |
| C | Stationary MAIN transitions, followed by driving. No temporary or permanent EPS fault in 3,934 compact-log car-state samples spanning about 393 seconds. This is a comparison, not a controlled experiment. |
| D | MAIN on while parked; reverse, then Drive. First LKAS request is **25.683 seconds after MAIN-on**, at about 0.72 mph and 464 degrees of steering angle. Raw EPS state 4 follows the request by 0.777 s. |

The MAIN-only holdoff has already expired when D first requests steering. Offline replay confirms identical request timing in D with and without this patch. A MAIN-only explanation therefore cannot account for every captured failure. High steering load, the reverse-to-Drive transition and other differences in D are observations, not diagnosed causes. Its internal panda also reports an existing `interruptRateCan1` fault; safety transmit rejections remain zero around the EPS fault. That additional condition needs investigation rather than being silently dismissed.

## Offline validation

Development library base: `sunnypilot/opendbc` at `f95f996f5917dcbbf2e32fe51b606a24cf836af6`. Its two edited Chrysler runtime files are byte-identical at baseline to those in the installed Sunnypilot release. The same patch and regression tests are mirrored into `sunnypilot`'s vendored library at release commit `6a17f75c6bcb67c85f252a1acc342d94d5b8a4d2` (`2026.002.002`, `release-mici`), and tested there separately.

- The moving MAIN-on regression was first run against unmodified controller code and failed because LKAS was requested during the required holdoff.
- Both library baselines pass **192 Chrysler safety/controller tests, with 21 existing skips and 234 passing subtests**. These comprise 180 safety tests and 12 new controller tests. The new tests use the real interface, controller, CAN packer and checksum/counter parser.
- Both baselines additionally pass all **10 selected Chrysler/Jeep/Ram/Durango interface tests**.
- Changed Python files pass Ruff and both diffs pass `git diff --check`.
- The exact release's old cross-mode safety-test discovery raised `AttributeError: TestBuild has no TX_MSGS` in three tests before assertions ran. The release branch backports the current opendbc predicate that selects actual `SafetyTest` subclasses. All three checks then execute and pass. This is a test-helper change, not a safety-policy or firmware change.

Controller coverage includes moving MAIN-on after long inactivity, MAIN off with a stale lateral request, toggling during the wait, a MAIN-off sample between steering transmissions, startup and existing re-enable protection, CANCEL/SET with MADS, loss of lateral eligibility, invalid CAN, temporary/permanent EPS faults, output cadence, counters, checksums, torque ramp and unaffected platforms.

Frozen-input replay results, measured from logged car-state MAIN availability:

| Sequence | Baseline first request | Candidate first request |
| --- | ---: | ---: |
| A moving restart | 5 ms | 705 ms |
| B moving activation | 16 ms | 716 ms |
| C stationary activation followed by motion | 4,056 ms | 4,056 ms |
| D delayed first engagement after reversing | 25,666 ms | 25,666 ms |

Replay uses recorded car-state and lateral-control inputs with the real controller and packer; it does not model the EPS or regenerate vehicle behavior. Recorded permanent faults remain present, and the candidate disables output when they occur. Replay phase/timestamp alignment differs from actual send-CAN timing, so its baseline millisecond values must not replace the raw-CAN observations above. A's excerpt also begins mid-drive; only its later restart is used for the comparison.

Reproduce the library checks in an environment with this repository's testing dependencies, pytest, and parameterized:

```sh
python -m pytest -q opendbc/safety/tests/test_chrysler.py opendbc/sunnypilot/car/chrysler/tests/test_wp_main_engagement.py
python -m pytest -q opendbc/car/tests/test_car_interfaces.py -k 'CHRYSLER or JEEP or RAM or DODGE_DURANGO'
```

For the exact release, run inside `opendbc_repo`, add `-c pyproject.toml --confcutdir=.` to isolate these library tests from openpilot's top-level pytest hooks, and use its declared Hypothesis 6.47.x dependency.

## Remaining work before an installation candidate

Obtain the EPS diagnostic trouble code and an EPS-side WP capture in a controlled setup, covering both moving MAIN-on and first lateral engagement after reversing. Establish whether faults follow a speed-mode transition, request timing, steering load, message integrity, or another condition. The comma-side logs cannot directly prove which rewritten speed the EPS received, and they do not identify the exact WP binary.

A request-based holdoff, such as jvePilot's, is a separate experiment worth evaluating after that evidence; do not automatically substitute it or guess an angle threshold. No safety limits should be relaxed and no fault should be hidden to make testing pass.

There is no new waiting indicator: this is a controller-only prototype, and `CC.latActive` and the main UI can still indicate engagement during the holdoff. Resolve driver-visible readiness before any road validation. The existing project plan calls for the design-md visual contract when UI changes begin; no visual design is introduced here.

The comma-four application has not been fully built or run with this patch. Bench behavior, manual steering feel, exact Advanced-firmware compatibility and road behavior remain unverified. Keep the official release installed while this investigation continues.
