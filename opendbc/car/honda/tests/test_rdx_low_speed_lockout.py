"""Exercises the ACURA_RDX_3G-specific LOW_SPEED_LOCKOUT -> steerFaultTemporary fix in
carstate.py (see RDX-Project/rdx-eps-tuning-project.md section 2n for the field data this
is based on: STEER_MOTOR_TORQUE decaying to ~0 over 8-15s of sustained steering at highway
speed while STEER_STATUS reports LOW_SPEED_LOCKOUT, silently ignored by the pre-fix logic).

Doesn't go through opendbc.car.car_helpers.interfaces / CarInterface, because Honda's
carcontroller.py imports openpilot.common.params.Params, which needs a compiled
libparams_c native library not available outside a full openpilot build (see this repo's
own CLAUDE.md testing note). Instead this constructs CarState directly and feeds it real,
correctly-encoded CAN frames (via CANPacker) through its actual update() method, so the
real production code path -- including the real speed Kalman filter -- is what's tested,
just without the CarInterface/CarController layers this fix doesn't touch.

CP.safetyConfigs needs exactly one entry: opendbc.car.CanBusBase derives its bus offset
from len(CP.safetyConfigs), and get_can_parsers()'s pt_messages list is empty for this car
(relies on lazy per-message auto-subscription via cp.vl[...], which didn't populate values
in this test environment) -- so the CANParser here explicitly subscribes every pt-bus
message carstate.py's update() touches, built from CAR.ACURA_RDX_3G's actual DBC.
"""
import unittest

from opendbc.car import structs, Bus, CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.carstate import CarState
from opendbc.car.honda.values import CAR, HondaFlags, DBC
from opendbc.car.honda.hondacan import CanBus
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser

# Every pt-bus message name carstate.py's update() accesses via cp.vl[...] for this car
# (RADAR_REFERENCE/CRUISE_FAULT_STATUS are other-platform-only and aren't in this DBC).
PT_MESSAGES = ["ACC_CONTROL", "BRAKE_MODULE", "CAR_SPEED", "CRUISE",
               "DOORS_STATUS", "ENGINE_DATA", "POWERTRAIN_DATA",
               "SCM_BUTTONS", "SCM_FEEDBACK", "SEATBELT_STATUS", "STEER_STATUS",
               "STEERING_SENSORS", "VSA_STATUS", "WHEEL_SPEEDS"]


def make_rdx_cp() -> structs.CarParams:
  CP = structs.CarParams()
  CP.carFingerprint = str(CAR.ACURA_RDX_3G)
  CP.minSteerSpeed = 3. * CV.MPH_TO_MS
  CP.flags = int(HondaFlags.BOSCH)  # matches HondaBoschPlatformConfig.init() for this car
  CP.enableBsm = False
  CP.safetyConfigs = [structs.CarParams.SafetyConfig()]  # single panda, offset 0
  return CP


class TestRdxLowSpeedLockout(unittest.TestCase):
  def setUp(self):
    self.CP = make_rdx_cp()
    self.CP_SP = structs.CarParamsSP()
    self.dbc_name = DBC[self.CP.carFingerprint][Bus.pt]
    self.packer = CANPacker(self.dbc_name)
    self.cs = CarState(self.CP, self.CP_SP)
    self.cp = CANParser(self.dbc_name, [(m, 0) for m in PT_MESSAGES], CanBus(self.CP).pt)
    self.cp_cam = CANParser(self.dbc_name, [], CanBus(self.CP).camera)

  def _pack(self, msg_name: str, values: dict) -> CanData:
    addr, dat, _ = self.packer.make_can_msg(msg_name, 1, values)
    return CanData(addr, dat, 1)

  def _run(self, steer_status: int, speed_kph: float, n: int = 3) -> structs.CarState:
    """Feeds STEER_STATUS + WHEEL_SPEEDS for n frames (20ms apart) and returns the final CarState."""
    frames = [
      self._pack('STEER_STATUS', {'STEER_STATUS': steer_status, 'STEER_TORQUE_SENSOR': 0,
                                   'STEER_ANGLE_RATE': 0, 'STEER_CONTROL_ACTIVE': 1}),
      self._pack('WHEEL_SPEEDS', {f'WHEEL_SPEED_{w}': speed_kph for w in ('FL', 'FR', 'RL', 'RR')}),
    ]
    ret = None
    for i in range(n):
      self.cp.update([(i * int(0.02e9), frames)])
      self.cp_cam.update([(i * int(0.02e9), [])])
      ret, _ = self.cs.update({Bus.pt: self.cp, Bus.cam: self.cp_cam})
    return ret

  def test_low_speed_lockout_at_highway_speed_is_flagged(self):
    """The fix: LOW_SPEED_LOCKOUT at real driving speed (30mph) must now raise steerFaultTemporary."""
    ret = self._run(steer_status=3, speed_kph=48.28)  # 3 = LOW_SPEED_LOCKOUT, 48.28kph = 30mph
    self.assertGreater(ret.vEgo, 10, "sanity check: vEgo should reflect the packed 30mph")
    self.assertTrue(ret.steerFaultTemporary)
    self.assertFalse(ret.steerFaultPermanent)

  def test_low_speed_lockout_at_genuine_low_speed_is_unaffected(self):
    """Original behavior preserved: LOW_SPEED_LOCKOUT below minSteerSpeed (~1.5mph) stays non-fault."""
    ret = self._run(steer_status=3, speed_kph=2.4)  # ~1.5mph, below this car's 3mph minSteerSpeed
    self.assertFalse(ret.steerFaultTemporary)
    self.assertFalse(ret.steerFaultPermanent)

  def test_normal_status_at_speed_is_not_flagged(self):
    """Sanity check: unrelated to this fix, NORMAL status never faults."""
    ret = self._run(steer_status=0, speed_kph=48.28)  # 0 = NORMAL
    self.assertFalse(ret.steerFaultTemporary)
    self.assertFalse(ret.steerFaultPermanent)

  def test_no_torque_alert_1_at_speed_is_unaffected_by_this_fix(self):
    """Pre-existing fault path (unconditional exclusion list) still works as before."""
    ret = self._run(steer_status=2, speed_kph=48.28)  # 2 = NO_TORQUE_ALERT_1
    self.assertTrue(ret.steerFaultTemporary)
    self.assertFalse(ret.steerFaultPermanent)


if __name__ == '__main__':
  unittest.main()
