"""The Bosch-A radar decoder must only be enabled when openpilot longitudinal is off (radar ECU left alive)."""
import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.common.params import Params


@pytest.fixture
def radar_toggle():
  Params().put_bool("HondaBoschARadar", True)
  yield
  Params().remove("HondaBoschARadar")


def _cp(car, alpha_long, docs=False):
  return CarInterface.get_params(car, gen_empty_fingerprint(), [], alpha_long, False, docs)


def test_rdx_decoder_on_when_alpha_long_off(radar_toggle):
  assert _cp(CAR.ACURA_RDX_3G, alpha_long=False).radarUnavailable is False


def test_rdx_decoder_off_when_alpha_long_on(radar_toggle):
  cp = _cp(CAR.ACURA_RDX_3G, alpha_long=True)
  assert cp.radarUnavailable is True
  assert cp.openpilotLongitudinalControl is True


def test_rdx_decoder_off_when_param_unset():
  Params().remove("HondaBoschARadar")
  assert _cp(CAR.ACURA_RDX_3G, alpha_long=False).radarUnavailable is True


def test_rdx_decoder_off_for_docs(radar_toggle):
  assert _cp(CAR.ACURA_RDX_3G, alpha_long=False, docs=True).radarUnavailable is True
