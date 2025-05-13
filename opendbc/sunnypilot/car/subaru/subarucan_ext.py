from opendbc.car.subaru.values import CanBus


def create_counter(msg):
  return (msg["COUNTER"] + 1) % 0x10


def create_stop_and_go(packer, frame, throttle_msg, brake_pedal_msg, pcm_cancel_cmd, throttle_cmd, speed_cmd):
  ret = []

  throttle_values = {s: throttle_msg[s] for s in [
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
  throttle_values["COUNTER"] = create_counter(throttle_msg)

  if throttle_cmd:
    throttle_values["Throttle_Pedal"] = 5
  ret.append(packer.make_can_msg("Throttle", CanBus.camera, throttle_values))

  if frame % 2 == 0:
    brake_pedal_values = {s: brake_pedal_msg[s] for s in [
      "Signal1",
      "Speed",
      "Signal2",
      "Brake_Lights",
      "Signal3",
      "Brake_Pedal",
      "Signal4",
    ]}
    brake_pedal_values["COUNTER"] = create_counter(brake_pedal_msg)

    if speed_cmd:
      brake_pedal_values["Speed"] = 3
    if pcm_cancel_cmd:
      brake_pedal_values["Brake_Pedal"] = 5
      brake_pedal_values["Brake_Lights"] = 1
    ret.append(packer.make_can_msg("Brake_Pedal", CanBus.camera, brake_pedal_values))

  return ret


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

  values["COUNTER"] = create_counter(throttle_msg)

  if throttle_cmd:
    values["Throttle_Pedal"] = 5

  return packer.make_can_msg("Throttle", CanBus.camera, values)
