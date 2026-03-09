from typing import Any, Dict, List

from ntcore import NetworkTableEntry, NetworkTableInstance


class SmartNT:
    """Lightweight NetworkTables wrapper for simple key-value publishing.

    Entries are cached on first access so repeated puts/gets only pay
    a single dict lookup — no string splitting, no sub-table traversal.
    """

    __slots__ = ("table", "_entries", "_nt", "_sa_pubs")

    def __init__(self, root_table: str = "/"):
        self._nt = NetworkTableInstance.getDefault()
        self.table = self._nt.getTable(root_table.strip("/"))
        self._entries: Dict[str, NetworkTableEntry] = {}
        self._sa_pubs: Dict[str, Any] = {}  # StringArray publishers

    def _get_entry(self, key: str) -> NetworkTableEntry:
        try:
            return self._entries[key]
        except KeyError:
            pass
        path_parts = key.strip("/").split("/")
        table = self.table
        for part in path_parts[:-1]:
            table = table.getSubTable(part)
        entry = table.getEntry(path_parts[-1])
        self._entries[key] = entry
        return entry

    def set_type(self, type_name: str) -> None:
        """Set the `.type` metadata entry so dashboards render the correct widget."""
        self.table.getEntry(".type").setString(type_name)

    # ── typed puts (inlined for speed — no isinstance chain) ──

    def put_number(self, key: str, value: float) -> None:
        self._get_entry(key).setDouble(value)

    def put_boolean(self, key: str, value: bool) -> None:
        self._get_entry(key).setBoolean(value)

    def put_string(self, key: str, value: str) -> None:
        self._get_entry(key).setString(value)

    def put_string_array(self, key: str, value: List[str]) -> None:
        try:
            pub = self._sa_pubs[key]
        except KeyError:
            topic = self.table.getStringArrayTopic(key)
            pub = topic.publish()
            self._sa_pubs[key] = pub
        pub.set(value)

    # ── typed gets ──

    def get_number(self, key: str, default: float = 0.0) -> float:
        return self._get_entry(key).getDouble(default)

    def get_boolean(self, key: str, default: bool = False) -> bool:
        return self._get_entry(key).getBoolean(default)

    def get_string(self, key: str, default: str = "") -> str:
        return self._get_entry(key).getString(default)

    # ── generic put/get (kept for backwards compat) ──

    def put(self, key: str, value: Any) -> None:
        entry = self._get_entry(key)
        if isinstance(value, bool):
            entry.setBoolean(value)
        elif isinstance(value, (float, int)):
            entry.setDouble(float(value))
        elif isinstance(value, str):
            entry.setString(value)
        else:
            raise TypeError(f"Unsupported value type for key '{key}': {type(value)}")

    def get(self, key: str, default: Any = None) -> Any:
        entry = self._get_entry(key)
        if isinstance(default, bool):
            return entry.getBoolean(default)
        elif isinstance(default, (float, int)):
            return entry.getDouble(default)
        elif isinstance(default, str):
            return entry.getString(default)
        else:
            raise TypeError(
                f"Unsupported default type for key '{key}': {type(default)}"
            )
