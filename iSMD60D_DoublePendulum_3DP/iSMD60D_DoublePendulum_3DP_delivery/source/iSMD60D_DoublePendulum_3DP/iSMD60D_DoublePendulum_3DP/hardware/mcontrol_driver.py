"""mcontrol Python wrapper for iSMD60D deployment."""
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
    libs_dir = os.path.join(base_dir, "libs")
    sysname = platform.system().lower()
    if sysname == "windows":
        name = f"{LIB_NAME}.dll"
    elif sysname == "linux":
        name = f"{LIB_NAME}.so"
    elif sysname == "darwin":
        name = f"{LIB_NAME}.dylib"
    else:
        raise RuntimeError(f"Unsupported platform: {platform.system()}")
    path = os.path.join(libs_dir, name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Library not found: {path}")
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
        bind_json_api(lib, "close_session", [])
        bind_json_api(lib, "read_int", [ctypes.c_int, ctypes.c_int, ctypes.c_char_p])
        bind_json_api(lib, "write_int", [ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int32])

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

    def close(self) -> None:
        if self._opened:
            self._call_json(self.lib.close_session)
            self._opened = False
            self.station = None

    def __enter__(self) -> "MControl":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def read_int(self, addr: int, dtype: str) -> int:
        return self._call_json(self.lib.read_int, self._station(), addr, dtype.encode())

    def write_int(self, addr: int, dtype: str, value: int) -> None:
        self._call_json(self.lib.write_int, self._station(), addr, dtype.encode(), ctypes.c_int32(value))
