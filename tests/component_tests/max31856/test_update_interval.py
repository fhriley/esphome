"""A poll starts a new conversion, so update_interval must outlast one conversion."""

import pytest

from esphome import config_validation as cv
from esphome.components.max31856.sensor import (
    CONF_DATA_READY_PIN,
    CONF_SAMPLES_PER_VALUE,
    _conversion_time_ms,
    _validate_update_interval,
)
from esphome.const import CONF_MAINS_FILTER, CONF_UPDATE_INTERVAL, SCHEDULER_DONT_RUN
from esphome.core import TimePeriodMilliseconds
from esphome.types import ConfigType


def _config(mains: int, samples: int, interval_ms: int | None = None) -> ConfigType:
    config: ConfigType = {CONF_MAINS_FILTER: mains, CONF_SAMPLES_PER_VALUE: samples}
    if interval_ms is not None:
        config[CONF_UPDATE_INTERVAL] = TimePeriodMilliseconds(milliseconds=interval_ms)
    return config


@pytest.mark.parametrize(
    ("mains", "samples", "expected_ms"),
    [(60, 1, 170), (50, 1, 200), (60, 16, 725), (50, 16, 860)],
)
def test_conversion_time(mains: int, samples: int, expected_ms: int) -> None:
    assert _conversion_time_ms(_config(mains, samples)) == expected_ms


def test_interval_shorter_than_conversion_is_rejected() -> None:
    with pytest.raises(cv.Invalid, match="860ms"):
        _validate_update_interval(_config(50, 16, 500))


def test_interval_longer_than_conversion_is_accepted() -> None:
    config = _validate_update_interval(_config(50, 16, 1000))
    assert config[CONF_UPDATE_INTERVAL].total_milliseconds == 1000


def test_never_is_accepted() -> None:
    _validate_update_interval(_config(50, 16, SCHEDULER_DONT_RUN))


def test_default_interval_is_60s() -> None:
    config = _validate_update_interval(_config(60, 1))
    assert config[CONF_UPDATE_INTERVAL].total_milliseconds == 60000


def test_autoconversion_mode_has_no_interval() -> None:
    config = _config(50, 16)
    config[CONF_DATA_READY_PIN] = {"number": 4}
    assert CONF_UPDATE_INTERVAL not in _validate_update_interval(config)
