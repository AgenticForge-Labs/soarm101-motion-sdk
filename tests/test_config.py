import pytest

from soarm101_motion import SOARM101Config
from soarm101_motion.exceptions import ConfigurationError


def test_valid_config() -> None:
    assert SOARM101Config().command_frequency_hz == 30.0


def test_invalid_frequency() -> None:
    with pytest.raises(ConfigurationError):
        SOARM101Config(command_frequency_hz=0)
