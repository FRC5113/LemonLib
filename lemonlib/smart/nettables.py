from ntcore import NetworkTableInstance
from typing import Any, Dict

class SmartNT:
    def __init__(self, root_table: str = "/", verbose: bool = False):
        self.nt = NetworkTableInstance.getDefault()
        self.table = self.nt.getTable(root_table.strip("/"))
        self._entries: Dict[str, Any] = {}
        self.verbose = verbose

    def _get_entry(self, key: str):
        if key not in self._entries:
            path_parts = key.strip("/").split("/")
            table = self.table
            for part in path_parts[:-1]:
                table = table.getSubTable(part)
            entry = table.getEntry(path_parts[-1])
            self._entries[key] = entry
            if self.verbose:
                print(f"[SmartNT] Created entry: /{ '/'.join(path_parts) }")
        return self._entries[key]

    def put(self, key: str, value: Any, persistent: bool = False):
        entry = self._get_entry(key)

        if isinstance(value, float) or isinstance(value, int):
            entry.setDouble(float(value))
        elif isinstance(value, bool):
            entry.setBoolean(value)
        elif isinstance(value, str):
            entry.setString(value)
        else:
            raise TypeError(f"Unsupported value type for key '{key}': {type(value)}")

        if persistent:
            entry.setPersistent()

        if self.verbose:
            print(f"[SmartNT] Set {key} = {value} (type: {type(value).__name__})")

    def get(self, key: str, default: Any = None) -> Any:
        entry = self._get_entry(key)
        if isinstance(default, float) or isinstance(default, int):
            return entry.getDouble(default)
        elif isinstance(default, bool):
            return entry.getBoolean(default)
        elif isinstance(default, str):
            return entry.getString(default)
        else:
            raise TypeError(f"Unsupported default type for key '{key}': {type(default)}")

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
