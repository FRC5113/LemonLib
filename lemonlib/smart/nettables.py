import threading
import time
from typing import Any, Callable, Dict, Optional
from ntcore import NetworkTableInstance, NetworkTableEntry


class SmartNT:
    def __init__(
        self, root_table: str = "/", verbose: bool = False, poll_period: float = 0.02
    ):
        self.nt = NetworkTableInstance.getDefault()
        self.table = self.nt.getTable(root_table.strip("/"))
        self._entries: Dict[str, NetworkTableEntry] = {}
        self._properties: Dict[str, Dict[str, Callable]] = {}
        self.verbose = verbose
        self.poll_period = poll_period
        self._running = False

    def _get_entry(self, key: str) -> NetworkTableEntry:
        key = str(key)
        if key not in self._entries:
            path_parts = key.strip("/").split("/")
            table = self.table
            for part in path_parts[:-1]:
                table = table.getSubTable(part)
            entry = table.getEntry(path_parts[-1])
            self._entries[key] = entry
            if self.verbose:
                print(f"[SmartNT] Created entry: /{'/'.join(path_parts)}")
        return self._entries[key]

    def put(self, key: str, value: Any):
        entry = self._get_entry(key)
        if isinstance(value, (float, int)):
            entry.setDouble(float(value))
        elif isinstance(value, bool):
            entry.setBoolean(value)
        elif isinstance(value, str):
            entry.setString(value)
        else:
            raise TypeError(f"Unsupported value type for key '{key}': {type(value)}")
        if self.verbose:
            print(f"[SmartNT] Set {key} = {value} (type: {type(value).__name__})")

    def get(self, key: str, default: Any = None) -> Any:
        entry = self._get_entry(key)
        if isinstance(default, (float, int)):
            return entry.getDouble(default)
        elif isinstance(default, bool):
            return entry.getBoolean(default)
        elif isinstance(default, str):
            return entry.getString(default)
        else:
            raise TypeError(
                f"Unsupported default type for key '{key}': {type(default)}"
            )

    def add_double_property(
        self, key: str, getter: Callable[[], float], setter: Callable[[float], None]
    ):
        self._properties[key] = {"getter": getter, "setter": setter, "type": "double"}
        self._get_entry(key)

    def add_boolean_property(
        self, key: str, getter: Callable[[], bool], setter: Callable[[bool], None]
    ):
        self._properties[key] = {"getter": getter, "setter": setter, "type": "boolean"}
        self._get_entry(key)

    def add_string_property(
        self, key: str, getter: Callable[[], str], setter: Callable[[str], None]
    ):
        self._properties[key] = {"getter": getter, "setter": setter, "type": "string"}
        self._get_entry(key)

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._update_loop, daemon=True)
            self._thread.start()
            if self.verbose:
                print("[SmartNT] Update thread started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
            if self.verbose:
                print("[SmartNT] Update thread stopped")

    def _update_loop(self):

        while self._running:
            for key, funcs in self._properties.items():

                entry = self._get_entry(key)
                getter = funcs["getter"]
                setter = funcs["setter"]
                typ = funcs["type"]

                try:
                    # Push current getter value to NetworkTables
                    val = getter()
                    if typ == "double":
                        entry.setDouble(float(val))
                    elif typ == "boolean":
                        entry.setBoolean(bool(val))
                    elif typ == "string":
                        entry.setString(str(val))
                except Exception as e:
                    if self.verbose:
                        print(f"[SmartNT] Getter error for '{key}': {e}")

                try:
                    # Get the value from NT and update local property if different
                    if typ == "double":
                        nt_val = entry.getDouble(getter())
                        if abs(nt_val - getter()) > 1e-6:
                            setter(nt_val)
                    elif typ == "boolean":
                        nt_val = entry.getBoolean(getter())
                        if nt_val != getter():
                            setter(nt_val)
                    elif typ == "string":
                        nt_val = entry.getString(getter())
                        if nt_val != getter():
                            setter(nt_val)
                except Exception as e:
                    if self.verbose:
                        print(f"[SmartNT] Setter error for '{key}': {e}")

            time.sleep(self.poll_period)

    # Optional legacy-style helpers
    def put_number(self, key: str, value: float):
        self.put(key, float(value))

    def put_boolean(self, key: str, value: bool):
        self.put(key, value)

    def put_string(self, key: str, value: str):
        self.put(key, value)

    def get_number(self, key: str, default: float = 0.0) -> float:
        return float(self.get(key, default))

    def get_boolean(self, key: str, default: bool = False) -> bool:
        return bool(self.get(key, default))

    def get_string(self, key: str, default: str = "") -> str:
        return str(self.get(key, default))

    def value(self, key: str, val_or_default: Any = None, set: bool = False) -> Any:
        if set:
            self.put(key, val_or_default)
            return val_or_default
        else:
            return self.get(key, val_or_default)
