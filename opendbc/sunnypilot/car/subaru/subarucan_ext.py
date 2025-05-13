from opendbc.car.subaru.values import CanBus


def create_brake_pedal(packer, frame, brake_pedal_msg, speed_cmd, brake_cmd):
  values = {s: brake_pedal_msg[s] for s in [
    "Signal1",
    "Speed",
    "Signal2",
    "Brake_Lights",
    "Signal3",
    "Brake_Pedal",
    "Signal4",
  ]}

  values["COUNTER"] = frame % 0x10

  if speed_cmd:
    values["Speed"] = 3
  if brake_cmd:
    values["Brake_Pedal"] = 5
    values["Brake_Lights"] = 1

  return packer.make_can_msg("Brake_Pedal", CanBus.camera, values)


def create_throttle(packer, frame, throttle_msg, throttle_cmd):
  values = {s: throttle_msg[s] for s in [
    "CHECKSUM",
    "Signal1",
    "Engine_RPM",
    "Signal2",
    "Throttle_Pedal",
    "Throttle_Cruise",
    "Throttle_Combo",
    "Signal3",
    "Off_Accel",
  ]}

  values["COUNTER"] = frame % 0x10

  if throttle_cmd:
    values["Throttle_Pedal"] = 5

  return packer.make_can_msg("Throttle", CanBus.camera, values)


def create_preglobal_stop_and_go(packer, throttle_msg, throttle_cmd):
  values = {s: throttle_msg[s] for s in [
    "Throttle_Pedal",
    "Signal1",
    "Not_Full_Throttle",
    "Signal2",
    "Engine_RPM",
    "Off_Throttle",
    "Signal3",
    "Throttle_Cruise",
    "Throttle_Combo",
    "Throttle_Body",
    "Off_Throttle_2",
    "Signal4",
  ]}

  values["COUNTER"] = (throttle_msg["COUNTER"] + 1) % 0x10

  if throttle_cmd:
    values["Throttle_Pedal"] = 5

  return packer.make_can_msg("Throttle", CanBus.camera, values)
