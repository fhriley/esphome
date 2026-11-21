"""A poll starts a new measurement, so update_interval must outlast one measurement."""

import pytest

from esphome import config_validation as cv
from esphome.components.max31865.sensor import (
    CONF_DATA_READY_PIN,
    _measurement_time_ms,
    _validate_update_interval,
)
from esphome.const import CONF_MAINS_FILTER, CONF_UPDATE_INTERVAL, SCHEDULER_DONT_RUN
from esphome.core import TimePeriodMilliseconds
from esphome.types import ConfigType


def _config(mains: str, interval_ms: int | None = None) -> ConfigType:
    config: ConfigType = {CONF_MAINS_FILTER: mains}
    if interval_ms is not None:
        config[CONF_UPDATE_INTERVAL] = TimePeriodMilliseconds(milliseconds=interval_ms)
    return config


@pytest.mark.parametrize(("mains", "expected_ms"), [("60HZ", 73), ("50HZ", 84)])
def test_measurement_time(mains: str, expected_ms: int) -> None:
    assert _measurement_time_ms(_config(mains)) == expected_ms


def test_interval_shorter_than_measurement_is_rejected() -> None:
    with pytest.raises(cv.Invalid, match="84ms"):
        _validate_update_interval(_config("50HZ", 50))


def test_interval_longer_than_measurement_is_accepted() -> None:
    config = _validate_update_interval(_config("50HZ", 100))
    assert config[CONF_UPDATE_INTERVAL].total_milliseconds == 100


def test_never_is_accepted() -> None:
    _validate_update_interval(_config("50HZ", SCHEDULER_DONT_RUN))


def test_default_interval_is_60s() -> None:
    config = _validate_update_interval(_config("60HZ"))
    assert config[CONF_UPDATE_INTERVAL].total_milliseconds == 60000


def test_autoconversion_mode_has_no_interval() -> None:
    config = _config("50HZ")
    config[CONF_DATA_READY_PIN] = {"number": 4}
    assert CONF_UPDATE_INTERVAL not in _validate_update_interval(config)
