"""find_device_port picks the board's telemetry CDC, skipping the debug probe.

The footgun it guards: with the CMSIS-DAP debug probe and the board both
plugged in, both enumerate a USB CDC under the Raspberry Pi vendor id, and the
old default (/dev/ttyACM0) often grabbed the probe -- which carries no
telemetry -- so the live monitor showed nothing.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from plrs_sim.monitor import _DEBUG_PROBE_PID, _RP_USB_VID, find_device_port


@dataclass
class _Port:
    device: str
    vid: int | None
    pid: int | None


def _patch_ports(monkeypatch, ports: list[_Port]) -> None:
    from serial.tools import list_ports

    monkeypatch.setattr(list_ports, "comports", lambda: ports)


def test_picks_the_board_over_the_probe(monkeypatch) -> None:
    _patch_ports(
        monkeypatch,
        [
            _Port("/dev/ttyACM0", _RP_USB_VID, _DEBUG_PROBE_PID),  # debug probe
            _Port("/dev/ttyACM1", _RP_USB_VID, 0x000F),  # Pico 2 CDC
        ],
    )
    assert find_device_port() == "/dev/ttyACM1"


def test_ignores_non_raspberry_ports(monkeypatch) -> None:
    _patch_ports(
        monkeypatch,
        [
            _Port("/dev/ttyUSB0", 0x0403, 0x6001),  # an FTDI adapter
            _Port("/dev/ttyACM1", _RP_USB_VID, 0x000A),  # Pico CDC
        ],
    )
    assert find_device_port() == "/dev/ttyACM1"


def test_raises_when_only_the_probe_is_present(monkeypatch) -> None:
    _patch_ports(monkeypatch, [_Port("/dev/ttyACM0", _RP_USB_VID, _DEBUG_PROBE_PID)])
    with pytest.raises(RuntimeError, match="pass --port"):
        find_device_port()


def test_raises_on_ambiguous_multiple_boards(monkeypatch) -> None:
    _patch_ports(
        monkeypatch,
        [
            _Port("/dev/ttyACM0", _RP_USB_VID, 0x000F),
            _Port("/dev/ttyACM1", _RP_USB_VID, 0x000A),
        ],
    )
    with pytest.raises(RuntimeError, match="multiple candidate"):
        find_device_port()
