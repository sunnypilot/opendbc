import math
import numbers
from collections import defaultdict, deque
from collections.abc import Sequence
from dataclasses import dataclass, field

from opendbc.car.carlog import carlog
from opendbc.can.dbc import DBC, Signal


MAX_BAD_COUNTER = 5
CAN_INVALID_CNT = 5


@dataclass
class MessageState:
  address: int
  name: str
  size: int
  signals: list[Signal]
  ignore_alive: bool = False
  ignore_checksum: bool = False
  ignore_counter: bool = False
  frequency: float = 0.0
  timeout_threshold: float = 1e5  # default to 1Hz threshold
  vals: list[float] = field(default_factory=list)
  all_vals: list[list[float]] = field(default_factory=list)
  timestamps: deque[int] = field(default_factory=lambda: deque(maxlen=500))
  counter: int = 0
  counter_fail: int = 0
  first_seen_nanos: int = 0
  last_warning_log_nanos: int = 0
  track_all_vals: bool = True

  def __post_init__(self):
    self._has_checksum = not self.ignore_checksum and any(s.calc_checksum is not None for s in self.signals)
    self._has_counter = not self.ignore_counter and any(s.type == 1 for s in self.signals)
    self._sig_names = tuple(s.name for s in self.signals)

  def rate_limited_log(self, last_update_nanos: int, msg: str) -> None:
    if (last_update_nanos - self.last_warning_log_nanos) >= 1_000_000_000:
      carlog.warning(f"CANParser: {hex(self.address)} {self.name} {msg}")
      self.last_warning_log_nanos = last_update_nanos

  def parse(self, nanos: int, dat: bytes) -> bool:
    signals = self.signals
    n_sigs = len(signals)
    if self.first_seen_nanos == 0:
      self.first_seen_nanos = nanos

    if not self._has_checksum and not self._has_counter:
      if not self.vals:
        self.vals = [0.0] * n_sigs
        if self.track_all_vals:
          self.all_vals = [[] for _ in range(n_sigs)]
      vals = self.vals
      if self.track_all_vals:
        all_vals = self.all_vals
        for i, sig in enumerate(signals):
          tmp = sig.get_raw_value(dat)
          if sig._sign_bit and tmp >= sig._sign_bit:
            tmp -= sig._sign_sub
          v = tmp * sig.factor + sig.offset
          vals[i] = v
          all_vals[i].append(v)
      else:
        for i, sig in enumerate(signals):
          tmp = sig.get_raw_value(dat)
          if sig._sign_bit and tmp >= sig._sign_bit:
            tmp -= sig._sign_sub
          vals[i] = tmp * sig.factor + sig.offset
    else:
      tmp_vals: list[float] = [0.0] * n_sigs
      checksum_failed = False
      counter_failed = False
      counter = 0
      counter_sig = None

      for i, sig in enumerate(signals):
        tmp = sig.get_raw_value(dat)
        if sig._sign_bit and tmp >= sig._sign_bit:
          tmp -= sig._sign_sub

        if not self.ignore_checksum and sig.calc_checksum is not None:
          expected_checksum = sig.calc_checksum(self.address, sig, bytearray(dat))
          if tmp != expected_checksum:
            checksum_failed = True
            self.rate_limited_log(nanos, f"checksum failed: received {hex(tmp)}, calculated {hex(expected_checksum)}")

        if sig.type == 1:  # COUNTER, which may be split across signals
          counter |= tmp << sig.counter_shift
          counter_sig = sig

        tmp_vals[i] = tmp * sig.factor + sig.offset

      if not self.ignore_counter and counter_sig is not None:
        counter_failed = not self.update_counter(counter, counter_sig.max_counter)

      # must have good counter and checksum to update data
      if checksum_failed or counter_failed:
        return False

      if not self.vals:
        self.vals = [0.0] * n_sigs
        if self.track_all_vals:
          self.all_vals = [[] for _ in range(n_sigs)]

      vals = self.vals
      if self.track_all_vals:
        all_vals = self.all_vals
        for i, v in enumerate(tmp_vals):
          vals[i] = v
          all_vals[i].append(v)
      else:
        self.vals = tmp_vals

    self.timestamps.append(nanos)

    if self.frequency < 1e-5 and len(self.timestamps) >= 3:
      dt = (self.timestamps[-1] - self.timestamps[0]) * 1e-9
      if (dt > 1.0 or (self.timestamps.maxlen is not None and len(self.timestamps) >= self.timestamps.maxlen)) and dt != 0:
        self.frequency = min((len(self.timestamps) - 1) / dt, 100.0)
        self.timeout_threshold = (1_000_000_000 / self.frequency) * 10
    return True

  def update_counter(self, cur_count: int, max_counter: int) -> bool:
    if (self.counter + 1) % (max_counter + 1) != cur_count:
      self.counter_fail = min(self.counter_fail + 1, MAX_BAD_COUNTER)
    elif self.counter_fail > 0:
      self.counter_fail -= 1
    self.counter = cur_count
    return self.counter_fail < MAX_BAD_COUNTER

  def valid(self, current_nanos: int, bus_timeout: bool) -> bool:
    if self.ignore_alive:
      return True
    if not self.timestamps:
      return False
    if (current_nanos - self.timestamps[-1]) > self.timeout_threshold:
      return False
    return True


class VLDict(dict):
  def __init__(self, parser):
    super().__init__()
    self.parser = parser

  def __missing__(self, key):
    self.parser._add_message(key)
    return dict.__getitem__(self, key)


class CANParser:
  def __init__(self, dbc_name: str, messages: Sequence[tuple[str | int, float]], bus: int,
               signals: set[str] | None = None, track_all_vals: bool = True):
    self.dbc_name: str = dbc_name
    self.bus: int = bus
    self.dbc = DBC(dbc_name)
    self.signals = signals
    self.track_all_vals = track_all_vals

    self.vl: dict[int | str, dict[str, float]] = VLDict(self)
    self.vl_all: dict[int | str, dict[str, list[float]]] = {}
    self.ts_nanos: dict[int | str, dict[str, int]] = {}
    self.addresses: set[int] = set()
    self.message_states: dict[int, MessageState] = {}
    self._dirty_vl_all_addrs: set[int] = set()
    self._ignore_alive: bool = True
    self._bus_timeout_threshold: float = 500 * 1_000_000

    for name_or_addr, freq in messages:
      if isinstance(name_or_addr, numbers.Number):
        msg = self.dbc.addr_to_msg.get(int(name_or_addr))
      else:
        msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        raise RuntimeError(f"could not find message {name_or_addr!r} in DBC {dbc_name}")
      if msg.address in self.addresses:
        raise RuntimeError("Duplicate Message Check: %d" % msg.address)

      self._add_message(name_or_addr, freq)

    self.can_invalid_cnt: int = CAN_INVALID_CNT
    self.last_nonempty_nanos: int = 0
    self._last_update_nanos: int = 0
    self._last_invalid_log_nanos: int | None = None

  def _add_message(self, name_or_addr: str | int, freq: float | None = None) -> None:
    if isinstance(name_or_addr, numbers.Number):
      msg = self.dbc.addr_to_msg.get(int(name_or_addr))
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
    assert msg is not None
    assert msg.address not in self.addresses

    self.addresses.add(msg.address)
    if self.signals is not None:
      missing_signals = self.signals - msg.sigs.keys()
      if missing_signals:
        raise RuntimeError(f"could not find signals {sorted(missing_signals)} in message {msg.name}")
      signal_names = [name for name in msg.sigs if name in self.signals]
    else:
      signal_names = list(msg.sigs.keys())
    signals_dict = {s: 0.0 for s in signal_names}
    dict.__setitem__(self.vl, msg.address, signals_dict)
    dict.__setitem__(self.vl, msg.name, signals_dict)
    self.vl_all[msg.address] = defaultdict(list)
    self.vl_all[msg.name] = self.vl_all[msg.address]
    self.ts_nanos[msg.address] = {s: 0 for s in signal_names}
    self.ts_nanos[msg.name] = self.ts_nanos[msg.address]

    state = MessageState(
      address=msg.address,
      name=msg.name,
      size=msg.size,
      signals=[msg.sigs[name] for name in signal_names],
      ignore_alive=freq is not None and math.isnan(freq),
      ignore_checksum=self.dbc.checksum_state.ignore_checksum if self.dbc.checksum_state else False,
      ignore_counter=self.dbc.checksum_state.ignore_counter if self.dbc.checksum_state else False,
      track_all_vals=self.track_all_vals,
    )
    if freq is not None and freq > 0:
      state.frequency = freq
    else:
      # if frequency not specified, assume 1Hz until we learn it
      freq = 1
    state.timeout_threshold = (1_000_000_000 / freq) * 10

    self.message_states[msg.address] = state
    if not state.ignore_alive:
      self._ignore_alive = False
    if state.timeout_threshold > 0:
      self._bus_timeout_threshold = min(self._bus_timeout_threshold, state.timeout_threshold)

  @property
  def bus_timeout(self) -> bool:
    return ((self._last_update_nanos - self.last_nonempty_nanos) > self._bus_timeout_threshold) and not self._ignore_alive

  @property
  def can_valid(self) -> bool:
    valid = True
    counters_valid = True
    bus_timeout = self.bus_timeout
    invalid_messages = set()
    for state in self.message_states.values():
      if state.counter_fail >= MAX_BAD_COUNTER:
        counters_valid = False
        state.rate_limited_log(self._last_update_nanos, f"counter invalid, {state.counter_fail=} {MAX_BAD_COUNTER=}")
      if not state.valid(self._last_update_nanos, bus_timeout):
        valid = False
        invalid_messages.add(state.address)

    if invalid_messages and (self._last_invalid_log_nanos is None or
                             self._last_update_nanos - self._last_invalid_log_nanos >= 10_000_000_000):
      messages = ', '.join(f"{hex(addr)} {self.message_states[addr].name}" for addr in sorted(invalid_messages))
      carlog.warning(f"CANParser: bus {self.bus} not valid (timeout or missing): {messages}")
      self._last_invalid_log_nanos = self._last_update_nanos

    # TODO: probably only want to increment this once per update() call
    self.can_invalid_cnt = 0 if valid else min(self.can_invalid_cnt + 1, CAN_INVALID_CNT)
    return self.can_invalid_cnt < CAN_INVALID_CNT and counters_valid

  def update(self, strings, sendcan: bool = False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]

    track_all_vals = self.track_all_vals
    if track_all_vals and self._dirty_vl_all_addrs:
      vl_all = self.vl_all
      for addr in self._dirty_vl_all_addrs:
        for lst in vl_all[addr].values():
          lst.clear()
      self._dirty_vl_all_addrs.clear()

    updated_addrs: set[int] = set()
    bus = self.bus
    msg_states = self.message_states
    vl = self.vl
    vl_all = self.vl_all
    ts_nanos = self.ts_nanos
    for entry in strings:
      t = entry[0]
      frames = entry[1]
      bus_empty = True
      for address, dat, src in frames:
        if src != bus:
          continue
        bus_empty = False
        state = msg_states.get(address)
        if state is None or len(dat) > 64:
          continue
        if state.parse(t, dat):
          updated_addrs.add(address)

          vl_addr = dict.__getitem__(vl, address)
          ts_addr = ts_nanos[address]
          sig_names = state._sig_names
          vals = state.vals
          last_ts = state.timestamps[-1]

          if track_all_vals:
            vl_all_addr = vl_all[address]
            all_vals = state.all_vals
            for i, name in enumerate(sig_names):
              vl_addr[name] = vals[i]
              vl_all_addr[name] = all_vals[i]
              ts_addr[name] = last_ts
          else:
            for i, name in enumerate(sig_names):
              vl_addr[name] = vals[i]
              ts_addr[name] = last_ts

      if not bus_empty:
        self.last_nonempty_nanos = t

      self._last_update_nanos = t

    if track_all_vals and updated_addrs:
      self._dirty_vl_all_addrs.update(updated_addrs)

    return updated_addrs


class CANDefine:
  def __init__(self, dbc_name: str):
    dbc = DBC(dbc_name)

    dv = defaultdict(dict)
    for val in dbc.vals:
      sgname = val.name
      address = val.address
      msg = dbc.addr_to_msg.get(address)
      if msg is None:
        raise KeyError(address)
      msgname = msg.name
      parts = val.def_val.split()
      values = [int(v) for v in parts[::2]]
      defs = parts[1::2]
      dv[address][sgname] = dict(zip(values, defs, strict=True))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv = dict(dv)
