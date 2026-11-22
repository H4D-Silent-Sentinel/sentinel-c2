"""USB serial link to the ESP32 base receiver (silent-sentinel firmware/base_rx).

Uplink   ESP32 -> C2   `PKT <rssi> <snr> <hex>`   node packets heard over LoRa
Downlink C2 -> ESP32   `TX <hex>`                  packet for base_rx to transmit (HINT)
                       base_rx answers `TXOK <n>` or `TXERR <reason>`

A node plugged straight into USB (no radio) speaks the same lines, so the C2 works with
either. Reconnects forever if the cable is pulled.
"""
from __future__ import annotations

import threading
import time

from cuas.io import parse_rx_line
from cuas.packet import Packet, PktType


class Esp32Link:
    def __init__(self, port: str, base, baud: int = 115200, log=print, retry_s: float = 2.0):
        self.port, self.base, self.baud, self.log, self.retry_s = port, base, baud, log, retry_s
        self.state = "down"  # "up" once the port is open
        self.last_rx = 0.0  # host time of last line from the ESP32
        self.rx_packets = self.tx_ok = self.tx_err = self.bad_lines = 0
        self._ser = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="esp32")

    def start(self) -> "Esp32Link":
        self._thread.start()
        return self

    def close(self, timeout_s: float = 2.0):
        """Stop the reader and release the port. The reader thread closes its own port (readline
        returns within 0.5 s); closing it from here while that thread is inside read() can block
        in the kernel on macOS, so only fall back to that if the thread doesn't exit in time."""
        self._stop.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout_s)
        with self._wlock:
            ser, self._ser = self._ser, None
        if ser:
            _release(ser)

    # -- downlink ---------------------------------------------------------------
    def send_packet(self, pkt: Packet) -> bool:
        """Queue one packet for the base receiver to transmit. False if the port is down."""
        line = f"TX {pkt.encode().hex()}\n".encode()
        with self._wlock:
            ser = self._ser
            if ser is None or self.state != "up":
                return False
            try:
                ser.write(line)
                ser.flush()
                return True
            except Exception as e:
                self.log(f"esp32: write failed: {e}")
                return False

    # -- uplink -----------------------------------------------------------------
    def _open(self):
        import serial  # pyserial
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, self.baud, 0.5
        s.dtr = s.rts = False  # don't reset the ESP32 (or strap it into flash mode) on open
        s.open()
        return s

    def _run(self):
        while not self._stop.is_set():
            try:
                ser = self._open()
            except Exception as e:
                if self.state != "down":
                    self.log(f"esp32: {self.port} unavailable ({e})")
                self.state = "down"
                self._stop.wait(self.retry_s)
                continue
            with self._wlock:
                self._ser, self.state = ser, "up"
            self.log(f"esp32: connected {self.port}")
            try:
                while not self._stop.is_set():
                    raw = ser.readline()
                    if raw:
                        self._on_line(raw.decode(errors="replace").strip())
            except Exception as e:
                if not self._stop.is_set():
                    self.log(f"esp32: lost {self.port} ({e}); retrying")
            finally:
                with self._wlock:
                    mine = self._ser is ser
                    self._ser, self.state = None, "down"
                if mine:
                    _release(ser)
            self._stop.wait(self.retry_s)

    def _on_line(self, line: str):
        if not line:
            return
        self.last_rx = time.time()
        if line.startswith("TXOK"):
            self.tx_ok += 1
            return
        if line.startswith("TXERR"):
            self.tx_err += 1
            self.log(f"esp32: {line}")
            return
        try:
            pkt = parse_rx_line(line)
        except ValueError as e:
            self.bad_lines += 1
            self.log(f"esp32: bad packet ({e}): {line}")
            return
        if pkt is None:
            if not line.startswith(("TX type=", "DBG", "HINT")):  # node console chatter
                self.log(f"esp32: {line}")
            return
        if pkt.type == PktType.HINT:  # our own downlink echoed by a USB node; not a report
            return
        self.rx_packets += 1
        self.base.on_packet(pkt, self.last_rx)


def _release(ser):
    """Close a serial port without waiting on unsent output. A tty close drains pending output
    first, and if the other end is gone (cable pulled, ESP32 reset) macOS can wait on that
    indefinitely, ignoring Ctrl-C."""
    try:
        ser.reset_output_buffer()
    except Exception:
        pass
    try:
        ser.close()
    except Exception:
        pass
