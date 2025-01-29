""" Represents the Raspberry Pi identity, used to select configuration.

Don't put anything in this class about the actual configuration, use a different class for that.

Keep this synchronized with java team100.config.Camera.
"""

# pylint: disable=C0103,

from enum import Enum, unique
from typing import Any


def _read_cpu_info() -> str:
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


try:
    # the raspberry pi 4 puts a sort-of-unique number here
    # TODO: does the rpi 5 do the same thing?
    _serial = _read_cpu_info()
except FileNotFoundError:
    # windows doesn't have this file
    _serial = ""


@unique
class Identity(Enum):
    # 2024 comp bot cameras
    SHOOTER = "10000000a7a892c0"
    RIGHTAMP = "10000000caeaae82"
    LEFTAMP = "100000004e0a1fb9"
    GAME_PIECE = "1000000013c9c96c"

    # camera-bot cameras
    GLOBAL_GAME_PIECE = "d44649628c20d4d4"
    GLOBAL_RIGHT = "364f07fb090a3bf7"
    GLOBAL_LEFT = "06ece53b019a5c2e"

    # for testing
    DEV = "10000000a7c673d9"  # rpi4 used for development
    DEV2 = "8ddb2ed6c49a9bce"
    FLIPPED = "flipme"  # example for per-identity config
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Identity.UNKNOWN

    @staticmethod
    def get() -> "Identity":
        serial = _serial
        print(serial)
        # print(f"Coprocessor serial: {serial}")
        identity: Identity = Identity(serial)
        print(identity)
        # print(f"Coprocessor identity: {identity.name}")
        return identity
