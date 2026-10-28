"""
mcontrol Python example.
Supports: Windows, Linux, macOS.

The native API returns JSON strings allocated by the dynamic library.
Every returned pointer is released through free_string after decoding.
"""

import ctypes
import json
import os
import platform
from dataclasses import dataclass


class MControlError(RuntimeError):
    pass


LIB_NAME = "mcontrol"


def resolve_library_path() -> str:
    base_dir = os.path.dirname(os.path.abspath(__file__))
    libs_dir = os.path.join(base_dir, "..", "libs")
    sysname = platform.system().lower()
    if sysname == "windows":
        name = f"{LIB_NAME}.dll"
    elif sysname == "linux":
        name = f"{LIB_NAME}.so"
    elif sysname == "darwin":
        name = f"{LIB_NAME}.dylib"
    else:
        raise RuntimeError(
            f"Unsupported platform: {platform.system()}. Supported: Windows, Linux, macOS."
        )

    path = os.path.join(libs_dir, name)
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Library not found: {path}. Build the library for this platform first."
        )
    return os.path.abspath(path)


def load_library(path: str) -> ctypes.CDLL:
    try:
        sysname = platform.system().lower()
        if sysname == "windows":
            return ctypes.WinDLL(path)
        if sysname in {"linux", "darwin"}:
            return ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
        return ctypes.CDLL(path)
    except OSError as e:
        raise MControlError(f"load library failed: {e}") from e


def bind_json_api(lib: ctypes.CDLL, name: str, argtypes: list[object]) -> None:
    func = getattr(lib, name)
    func.argtypes = argtypes
    func.restype = ctypes.c_void_p


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baudrate: int
    station: int


@dataclass(frozen=True)
class PCANConfig:
    usb_bus: str
    rate: str
    station: int


@dataclass(frozen=True)
class CxCANConfig:
    can_port: str
    rate: str
    station: int


class MControl:
    def __init__(self) -> None:
        self.lib_path = resolve_library_path()
        self.lib = load_library(self.lib_path)
        self._bind_api()
        self._opened = False
        self.station: int | None = None

    def _bind_api(self) -> None:
        lib = self.lib

        bind_json_api(lib, "create_serial_port", [ctypes.c_char_p, ctypes.c_int])
        bind_json_api(lib, "create_pcan_port", [ctypes.c_char_p, ctypes.c_char_p])
        bind_json_api(lib, "create_cxcan_port", [ctypes.c_char_p, ctypes.c_char_p])
        bind_json_api(lib, "close_session", [])

        bind_json_api(lib, "read_int", [ctypes.c_int, ctypes.c_int, ctypes.c_char_p])
        bind_json_api(
            lib,
            "write_int",
            [ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int32],
        )
        bind_json_api(lib, "read_string", [ctypes.c_int, ctypes.c_int])
        bind_json_api(lib, "write_string", [ctypes.c_int, ctypes.c_int, ctypes.c_char_p])
        bind_json_api(lib, "list_ports", [ctypes.c_char_p])

        bind_json_api(lib, "read_encoder_ppr", [ctypes.c_int])
        bind_json_api(lib, "read_pos_actual", [ctypes.c_int])
        bind_json_api(lib, "read_speed_real", [ctypes.c_int])
        bind_json_api(lib, "read_torque", [ctypes.c_int])
        bind_json_api(lib, "set_operation_mode", [ctypes.c_int, ctypes.c_int])
        bind_json_api(lib, "set_controlword", [ctypes.c_int, ctypes.c_int])
        bind_json_api(lib, "set_target_position", [ctypes.c_int, ctypes.c_int32])
        bind_json_api(lib, "set_position_p", [ctypes.c_int, ctypes.c_int32])
        bind_json_api(lib, "set_speed_p", [ctypes.c_int, ctypes.c_int32])
        bind_json_api(lib, "set_speed_i", [ctypes.c_int, ctypes.c_int32])
        bind_json_api(lib, "drive_reboot", [ctypes.c_int])
        bind_json_api(lib, "read_torque_coefficient", [ctypes.c_int])
        bind_json_api(lib, "set_stiffness", [ctypes.c_int, ctypes.c_int32])
        bind_json_api(lib, "set_damping", [ctypes.c_int, ctypes.c_int32])

        bind_json_api(lib, "set_profile_speed", [ctypes.c_int, ctypes.c_int32])
        bind_json_api(lib, "set_profile_acc", [ctypes.c_int, ctypes.c_int32])
        bind_json_api(lib, "set_profile_dec", [ctypes.c_int, ctypes.c_int32])

        self.lib.free_string.argtypes = [ctypes.c_void_p]
        self.lib.free_string.restype = None

    def _call_json(self, func, *args):
        ptr = func(*args)
        if not ptr:
            raise MControlError("native function returned null pointer")
        try:
            raw = ctypes.string_at(ptr).decode("utf-8")
            result = json.loads(raw)
        finally:
            self.lib.free_string(ptr)

        if not result.get("ok"):
            raise MControlError(result.get("error") or "unknown error")
        return result.get("data")

    def _station(self) -> int:
        if self.station is None:
            raise MControlError("no station selected, open a connection first")
        return self.station

    def open_serial(self, cfg: SerialConfig) -> "MControl":
        self._call_json(self.lib.create_serial_port, cfg.port.encode(), cfg.baudrate)
        self.station = cfg.station
        self._opened = True
        return self

    def open_pcan(self, cfg: PCANConfig) -> "MControl":
        self._call_json(self.lib.create_pcan_port, cfg.usb_bus.encode(), cfg.rate.encode())
        self.station = cfg.station
        self._opened = True
        return self

    def open_cxcan(self, cfg: CxCANConfig) -> "MControl":
        self._call_json(self.lib.create_cxcan_port, cfg.can_port.encode(), cfg.rate.encode())
        self.station = cfg.station
        self._opened = True
        return self

    def close(self) -> None:
        if self._opened:
            self._call_json(self.lib.close_session)
            self._opened = False
            self.station = None

    def list_ports(self, port_type: str = "SERIAL") -> list[str]:
        return self._call_json(self.lib.list_ports, port_type.upper().encode())

    def set_operation_mode(self, mode: int) -> None:
        self._call_json(self.lib.set_operation_mode, self._station(), mode)

    def set_target_position(self, position_deg: int) -> None:
        self._call_json(
            self.lib.set_target_position,
            self._station(),
            ctypes.c_int32(position_deg),
        )

    def set_profile_speed(self, speed: int) -> None:
        self._call_json(self.lib.set_profile_speed, self._station(), ctypes.c_int32(speed))

    def set_profile_acc(self, acc: int) -> None:
        self._call_json(self.lib.set_profile_acc, self._station(), ctypes.c_int32(acc))

    def set_profile_dec(self, dec: int) -> None:
        self._call_json(self.lib.set_profile_dec, self._station(), ctypes.c_int32(dec))

    def set_controlword(self, controlword: int) -> None:
        self._call_json(self.lib.set_controlword, self._station(), controlword)

    def read_encoder_ppr(self) -> int:
        return self._call_json(self.lib.read_encoder_ppr, self._station())

    def read_pos_actual(self) -> int:
        return self._call_json(self.lib.read_pos_actual, self._station())


def run_position_mode_example(ctrl: MControl, target_deg: int = 90) -> None:
    ctrl.set_operation_mode(1)
    print("set_operation_mode(1) ok")

    ctrl.set_target_position(target_deg)
    print(f"set_target_position({target_deg}) ok")

    ctrl.set_profile_speed(50)
    ctrl.set_profile_acc(50)
    ctrl.set_profile_dec(50)
    print("set_profile_speed/acc/dec(50) ok")

    ctrl.set_controlword(0x2F)
    print("set_controlword(0x2F) ok")


def print_serial_ports(ctrl: MControl) -> None:
    ports = ctrl.list_ports("SERIAL")
    print("Detected serial ports:", ports)


def main() -> None:
    ctrl = MControl()
    print("Library loaded:", ctrl.lib_path)
    print_serial_ports(ctrl)

    sysname = platform.system().lower()
    if sysname == "windows":
        port = "COM8"
    elif sysname == "linux":
        port = "/dev/ttyUSB0"
    else:
        port = "/dev/cu.usbserial-D30B0SHO"

    cfg = SerialConfig(port=port, baudrate=38400, station=127)

    try:
        ctrl.open_serial(cfg)
        print("create_serial_port ok")
        run_position_mode_example(ctrl, target_deg=90)
        print("read_encoder_ppr:", ctrl.read_encoder_ppr())
        print("read_pos_actual:", ctrl.read_pos_actual())
    except MControlError as e:
        print("Error:", e)
    finally:
        ctrl.close()
        print("close_session done")


if __name__ == "__main__":
    main()
