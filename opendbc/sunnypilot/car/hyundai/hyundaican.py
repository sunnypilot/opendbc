from opendbc.car.hyundai.hyundaican import hyundai_checksum
from opendbc.sunnypilot.car.hyundai.escc import EnhancedSmartCruiseControl

def create_acc_commands_escc(packer, enabled, accel, upper_jerk, idx, hud_control, set_speed, stopping, long_override, use_fca,
                             ESCC: EnhancedSmartCruiseControl = None):

  def get_scc11_values():
    return {
      "MainMode_ACC": 1,
      "TauGapSet": hud_control.leadDistanceBars,
      "VSetDis": set_speed if enabled else 0,
      "AliveCounterACC": idx % 0x10,
      "ObjValid": 1, # close lead makes controls tighter
      "ACC_ObjStatus": 1, # close lead makes controls tighter
      "ACC_ObjLatPos": 0,
      "ACC_ObjRelSpd": 0,
      "ACC_ObjDist": 1, # close lead makes controls tighter
    }

  def get_scc12_values():
    values = {
      "ACCMode": 2 if enabled and long_override else 1 if enabled else 0,
      "StopReq": 1 if stopping else 0,
      "aReqRaw": accel,
      "aReqValue": accel,  # stock ramps up and down respecting jerk limit until it reaches aReqRaw
      "CR_VSM_Alive": idx % 0xF,
    }

    # show AEB disabled indicator on dash with SCC12 if not sending FCA messages.
    # these signals also prevent a TCS fault on non-FCA cars with alpha longitudinal
    if not use_fca:
      values["CF_VSM_ConfMode"] = 1
    values["AEB_Status"] = 1 # AEB disabled

    # Since we have ESCC available, we can update SCC12 with ESCC values.
    ESCC.update_scc12(values)

    return values

  def calculate_scc12_checksum(values):
    scc12_dat = packer.make_can_msg("SCC12", 0, values)[1]
    values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10
    return values

  def get_scc14_values():
    return {
      "ComfortBandUpper": 0.0, # stock usually is 0 but sometimes uses higher values
      "ComfortBandLower": 0.0, # stock usually is 0 but sometimes uses higher values
      "JerkUpperLimit": upper_jerk, # stock usually is 1.0 but sometimes uses higher values
      "JerkLowerLimit": 5.0, # stock usually is 0.5 but sometimes uses higher values
      "ACCMode": 2 if enabled and long_override else 1 if enabled else 4, # stock will always be 4 instead of 0 after first disengage
      "ObjGap": 2 if hud_control.leadVisible else 0, # 5: >30, m, 4: 25-30 m, 3: 20-25 m, 2: < 20 m, 0: no lead
    }

  def get_fca11_values():
    return {
      "CR_FCA_Alive": idx % 0xF,
      "PAINT1_Status": 1,
      "FCA_DrvSetStatus": 1,
      "FCA_Status": 1,
    }

  def calculate_fca11_checksum(values):
    fca11_dat = packer.make_can_msg("FCA11", 0, values)[1]
    values["CR_FCA_ChkSum"] = hyundai_checksum(fca11_dat[:7])
    return values

  commands = []

  scc11_values = get_scc11_values()
  commands.append(packer.make_can_msg("SCC11", 0, scc11_values))

  scc12_values = get_scc12_values()
  scc12_values = calculate_scc12_checksum(scc12_values)
  commands.append(packer.make_can_msg("SCC12", 0, scc12_values))

  scc14_values = get_scc14_values()
  commands.append(packer.make_can_msg("SCC14", 0, scc14_values))

  return commands
