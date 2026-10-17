#!/usr/bin/env python3
# 2027 Kia Seltos uses the standard hyundai_canfd pt DBC, except GEAR_ALT (0x40) and
# ACCELERATOR_BRAKE_ALT (0x100) carry a 7-bit counter at bit17 (bit16 is a constant flag),
# not the shared 8-bit COUNTER at bit16. Derive a Seltos pt DBC that only patches those two,
# so the shared DBC (every other HKG CAN-FD car) is untouched.
import os

# messages whose COUNTER is 7-bit @ bit17 on this platform
PATCH_MSGS = {"64", "256"}
OLD = " SG_ COUNTER : 16|8@1+ (1,0) [0|255] \"\" XXX"
NEW = " SG_ COUNTER : 17|7@1+ (1,0) [0|127] \"\" XXX"


def generate() -> dict[str, str]:
  src = os.path.join(os.path.dirname(__file__), "hyundai_canfd.dbc")
  with open(src, encoding="utf-8") as f:
    lines = f.read().splitlines(keepends=True)

  out, cur = [], None
  for line in lines:
    s = line.strip()
    if s.startswith("BO_ "):
      cur = s.split()[1]
    if cur in PATCH_MSGS and line.rstrip("\n") == OLD:
      line = NEW + ("\n" if line.endswith("\n") else "")
    out.append(line)

  return {"hyundai_canfd_seltos.dbc": "".join(out)}


if __name__ == "__main__":
  print(generate()["hyundai_canfd_seltos.dbc"][:500])
