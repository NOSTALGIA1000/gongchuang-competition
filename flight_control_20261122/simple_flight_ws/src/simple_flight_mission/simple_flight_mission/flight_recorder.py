"""Bounded asynchronous diagnostics. No flight commands or ROS dependencies."""
from collections import deque
import copy
import json
from pathlib import Path
import threading
import time


class FlightRecorder:
    def __init__(self, path=None, *, stream=None, capacity=512, critical_reserve=64):
        if type(capacity) is not int or type(critical_reserve) is not int or not 0 < critical_reserve < capacity:
            raise ValueError("Require 0 < critical_reserve < capacity")
        self.capacity, self.reserve = capacity, critical_reserve
        self._stream = stream
        if stream is None and path is not None:
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._stream = path.open("w", encoding="utf-8", buffering=1)
        self._condition = threading.Condition()
        self._events = deque()
        self._snapshot = None
        self._closing = False
        self._stats = dict(dropped_telemetry=0, dropped_critical=0, write_errors=0,
                           events_written=0, snapshots_written=0, high_water=0,
                           last_error=None, last_write_monotonic=None)
        self._thread = threading.Thread(target=self._run, name="flight-recorder", daemon=True)
        self._thread.start()

    def emit(self, kind, data, *, monotonic=None, critical=True):
        if self._stream is None:
            return False
        # Copy only small observation dictionaries. Encoding and all disk I/O
        # belong to the writer. A later controller mutation cannot alter history.
        payload = dict(kind=kind, monotonic=time.monotonic() if monotonic is None else monotonic,
                       data=copy.deepcopy(data))
        with self._condition:
            if self._closing:
                return False
            if not critical and len(self._events) >= self.capacity-self.reserve:
                self._stats["dropped_telemetry"] += 1
                return False
            if len(self._events) >= self.capacity:
                removable = next((i for i, (important, _) in enumerate(self._events) if not important), None)
                if removable is None:
                    self._stats["dropped_critical"] += 1
                    return False
                del self._events[removable]
                self._stats["dropped_telemetry"] += 1
            self._events.append((critical, payload))
            self._stats["high_water"] = max(self._stats["high_water"], len(self._events))
            self._condition.notify()
            return True

    def snapshot(self, path, data):
        payload = (Path(path), copy.deepcopy(data))
        with self._condition:
            if self._closing:
                return False
            self._snapshot = payload  # Latest wins; never an unbounded file queue.
            self._condition.notify()
            return True

    def stats(self):
        with self._condition:
            return dict(self._stats, queued=len(self._events),
                        pending_snapshots=int(self._snapshot is not None),
                        writer_alive=self._thread.is_alive())

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._events or self._snapshot or self._closing)
                    if not self._events and self._snapshot is None and self._closing:
                        return
                    snapshot, self._snapshot = self._snapshot, None
                    event = self._events.popleft()[1] if self._events else None
                for operation, item in (("snapshot", snapshot), ("event", event)):
                    if item is None:
                        continue
                    try:
                        if operation == "snapshot":
                            path, value = item
                            path.parent.mkdir(parents=True, exist_ok=True)
                            temporary = path.with_suffix(path.suffix+".tmp")
                            temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")
                            temporary.replace(path)
                        else:
                            self._stream.write(json.dumps(item, ensure_ascii=False, allow_nan=False)+"\n")
                        with self._condition:
                            self._stats["snapshots_written" if operation == "snapshot" else "events_written"] += 1
                            self._stats["last_write_monotonic"] = time.monotonic()
                    except Exception as error:
                        with self._condition:
                            self._stats["write_errors"] += 1
                            self._stats["last_error"] = f"{operation}: {type(error).__name__}: {error}"
        finally:
            if self._stream is not None:
                try:
                    self._stream.close()
                except Exception as error:
                    with self._condition:
                        self._stats["write_errors"] += 1
                        self._stats["last_error"] = f"close: {error}"

    def close(self, timeout=2.):
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._thread.join(timeout=timeout)
        return not self._thread.is_alive()
