# **How custom longitudinal tuning works for Hyundai/Kia/Genesis vehicles in sunnypilot.**

The controller (`controller.py`) sits between LongControl and SCC12 / SCC14 (SCC_CONTROL on CAN-FD) and runs with every SCC12 frame
(50 Hz), as the radar's own SCC does. With `HyundaiLongitudinalTuning` on it sends what the car's own radar sends the ESC for the
same request.

**What it sends**

*   `aReqRaw` (`desired_accel`): LongControl's accel, clipped to `CarControllerParams`; 0 while stopping (StopReq) where
    `aReqValue` ramps, LongControl's stopping accel where it is the request.
*   `aReqValue` (`actual_accel`): a jerk-limited integrator toward `aReqRaw` at the jerk fields sent.
*   `JerkUpperLimit` / `JerkLowerLimit`: the rate `aReqValue` is allowed to move at.
*   `ComfortBandUpper` / `ComfortBandLower`: how far the ESC may sit from the request before it corrects.

**Jerk fields** (SCC14 `rate` family, the Kia Niro EV's)

*   Close the gap between `aReqRaw` and `aReqValue` in 0.9 s up / 0.5 s down, and carry the request's own rate (over 0.1 s) only
    when releasing braking (`aReqValue` below -0.5) or launching (below 2 m/s).
*   Floors: 0.3 up; 0.5 down, 1.0 with a lead inside the following distance the gap setting asks for
    (20 m + 0.44 s x distance bars x speed).
*   Caps: the lower field at ISO 15622 / 22179's negative jerk (5.0 m/s³ below 5 m/s to 2.5 above 20 m/s), the upper at the
    speed-based limit.
*   Under StopReq the fields are the floors, so `aReqValue` rises to 0 at 0.3 m/s³ while the ESC holds the car.

**Comfort bands** only with a lead and not under StopReq, by `aReqValue`: lower 0.15-0.19 while braking, upper to 0.085 accelerating.
On CAN-FD the car's own fixed bands with a lead that isn't pulling away.

**Other radars** (`config.py`, `CAR_SPECIFIC_CONFIGS`): per-car floors, and the SCC14 family each car's radar sends: `fixed` jerk
fields, `zeros` or `none` (`aReqValue` is the request, no SCC14 sent for `none`), or CAN-FD `stepped` fields that switch on the
request's sign.

**Off** sends upstream's fixed values: jerk 3.0 up (1.0 outside PID) and 5.0 down, `aReqValue` = `aReqRaw`, no bands.
