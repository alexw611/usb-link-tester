#!/usr/bin/env python3
"""
install:
python3 --version
Python 3.10.11

python3 -m pip install pyserial matplotlib numpy



USB-Strecken-Tester (Laptop <-> ESP32) mit Live-Diagrammen und Auswertung.

Installation:   pip install pyserial matplotlib numpy
Start:          python usblinktester.py

Gegenstelle: usbtester.ino auf dem ESP32.
Wichtig: Serial Monitor der Arduino IDE schliessen (COM-Port ist exklusiv).
"""


import binascii
import csv
import os
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

SYNC = b"\xAA\x55"
FRAME_LEN = 256
PAYLOAD_LEN = 240
BIN = 0.5                 # Zeitauflösung der Diagramme in Sekunden
BASE_BAUD = 115200
LOST_TIMEOUT = 4.0        # so lange keine Frames -> Verbindung gilt als abgerissen
MAX_EVENTS = 1000
PATTERN = os.urandom(512)
FIELDS = ["rx_bytes", "rx_lost", "rx_bad", "tx_bytes", "tx_lost", "gap_ms"]


class HandshakeError(Exception):
    pass


class LinkLost(Exception):
    pass


def build_frame(seq):
    off = (seq * 7) % 256
    body = struct.pack("<III", seq & 0xFFFFFFFF, 0, 0) + PATTERN[off:off + PAYLOAD_LEN]
    return SYNC + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


class Parser:
    """Sucht Frames im Bytestrom, prüft CRC, synchronisiert sich nach Fehlern neu."""

    def __init__(self):
        self.buf = bytearray()
        self.resync = False

    def feed(self, data):
        buf = self.buf
        buf += data
        frames, bad = [], 0
        while True:
            j = buf.find(SYNC)
            if j < 0:
                keep = 1 if buf.endswith(b"\xAA") else 0
                garbage = len(buf) - keep
                if garbage > 0:
                    if not self.resync:
                        bad += 1
                        self.resync = True
                    del buf[:garbage]
                break
            if j > 0:
                if not self.resync:
                    bad += 1
                    self.resync = True
                del buf[:j]
            if len(buf) < FRAME_LEN:
                break
            body = bytes(buf[2:254])
            if binascii.crc_hqx(body, 0xFFFF) == (buf[254] | (buf[255] << 8)):
                frames.append(struct.unpack_from("<III", body))
                self.resync = False
                del buf[:FRAME_LEN]
            else:
                if not self.resync:
                    bad += 1
                    self.resync = True
                del buf[:1]
        return frames, bad


class Tester(threading.Thread):
    def __init__(self, port, baud, mode, duration):
        super().__init__(daemon=True)
        self.port, self.baud, self.mode, self.duration = port, baud, mode, duration
        self.lock = threading.RLock()
        self.stop_evt = threading.Event()
        self.bins = {k: [] for k in FIELDS}
        self.events = []
        self.outages = []          # [start, end|None]
        self.t0 = None
        self.last_frame_t = None
        self.done = False
        self.error = None
        self.writer_err = None

    # ---------- Hilfsfunktionen ----------
    @property
    def gap_thr(self):
        return max(0.15, 8 * 2560.0 / self.baud) if (self.mode & 1) else 0.6

    def elapsed(self):
        return 0.0 if self.t0 is None else time.perf_counter() - self.t0

    def _bin(self, t):
        idx = int(t / BIN)
        while len(self.bins["rx_bytes"]) <= idx:
            for k in FIELDS:
                self.bins[k].append(0)
        return idx

    def _event(self, t, kind, text):
        with self.lock:
            if len(self.events) < MAX_EVENTS:
                self.events.append((t, kind, text))

    def _outage_begin(self, msg):
        with self.lock:
            if self.outages and self.outages[-1][1] is None:
                return
            t = self.last_frame_t if self.last_frame_t is not None else self.elapsed()
            self.outages.append([t, None])
            self._event(t, "Verbindung abgerissen", msg)

    def _outage_end(self, t):
        with self.lock:
            if self.outages and self.outages[-1][1] is None:
                self.outages[-1][1] = t
                self._event(t, "Verbindung wieder da", f"Ausfall {t - self.outages[-1][0]:.1f} s")

    def snapshot(self):
        with self.lock:
            d = {k: np.array(v, dtype=float) for k, v in self.bins.items()}
            return d, list(self.events), [tuple(o) for o in self.outages], self.elapsed()

    # ---------- Seriell ----------
    def _open(self, baud):
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = baud
        ser.timeout = 0.05
        ser.write_timeout = 2
        ser.dtr = False      # verhindert (meist) den Auto-Reset des ESP32 beim Öffnen
        ser.rts = False
        ser.open()
        return ser

    @staticmethod
    def _wait(ser, token, timeout):
        """Wartet auf token. Gibt die Bytes NACH dem Token zurück (oder None)."""
        end = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < end:
            buf += ser.read(max(1, ser.in_waiting))
            if token in buf:
                return buf.partition(token)[2]
        return None

    def _ping(self, ser, timeout=0.6):
        ser.write(b"X\nP\n")
        return self._wait(ser, b"PONG\n", timeout) is not None

    def _connect(self):
        ser = self._open(self.baud)
        try:
            ok = False
            if self.baud != BASE_BAUD:
                ok = self._ping(ser) or self._ping(ser)
            if not ok:
                ser.baudrate = BASE_BAUD
                deadline = time.monotonic() + 9
                while time.monotonic() < deadline and not ok:
                    if self.stop_evt.is_set():
                        raise HandshakeError("abgebrochen")
                    ok = self._ping(ser)
                if not ok:
                    raise HandshakeError("ESP32 antwortet nicht (Sketch geflasht? Serial Monitor offen?)")
                if self.baud != BASE_BAUD:
                    ser.write(f"B:{self.baud}\n".encode())
                    if self._wait(ser, b"OK B\n", 1.0) is None:
                        raise HandshakeError("Baudraten-Wechsel nicht bestätigt")
                    ser.baudrate = self.baud
                    time.sleep(0.2)
                    if not (self._ping(ser) or self._ping(ser)):
                        raise HandshakeError(f"Keine Verbindung bei {self.baud} Baud "
                                             f"(vom USB-Chip nicht unterstützt?)")
            ser.write(f"G:{self.mode}\n".encode())
            rest = self._wait(ser, b"OK G\n", 1.0)
            if rest is None:
                raise HandshakeError("Start-Kommando nicht bestätigt")
            return ser, rest
        except BaseException:
            try:
                ser.close()
            except Exception:
                pass
            raise

    def _writer(self, ser, stop):
        seq = 0
        try:
            while not stop.is_set() and not self.stop_evt.is_set():
                if self.mode & 2:
                    ser.write(build_frame(seq))
                    seq = (seq + 1) & 0xFFFFFFFF
                else:
                    ser.write(b"K\n")
                    stop.wait(0.5)
        except Exception as e:
            self.writer_err = e

    # ---------- Testlauf ----------
    def run(self):
        fails = 0
        try:
            while not self.stop_evt.is_set() and (self.t0 is None or self.elapsed() < self.duration):
                ser = None
                try:
                    ser, rest = self._connect()
                    fails = 0
                    self._session(ser, rest)
                except (serial.SerialException, OSError, LinkLost, HandshakeError) as e:
                    if self.stop_evt.is_set():
                        break
                    if self.t0 is None:
                        fails += 1
                        if fails >= 3:
                            self.error = f"Verbindung fehlgeschlagen: {e}"
                            break
                    else:
                        self._outage_begin(str(e))
                finally:
                    if ser is not None:
                        try:
                            ser.close()
                        except Exception:
                            pass
                if not self.stop_evt.is_set():
                    time.sleep(0.5)
        except Exception as e:  # noqa
            self.error = f"Unerwarteter Fehler: {e!r}"
        finally:
            with self.lock:
                if self.outages and self.outages[-1][1] is None:
                    self.outages[-1][1] = self.elapsed()
            self.done = True

    def _session(self, ser, carry):
        if self.t0 is None:
            self.t0 = time.perf_counter()
        self.writer_err = None
        parser = Parser()
        stop_w = threading.Event()
        wt = threading.Thread(target=self._writer, args=(ser, stop_w), daemon=True)
        wt.start()
        thr = self.gap_thr
        t_last = time.perf_counter()
        first = True
        stalled = False
        expected = last_g = last_l = 0
        normal_end = False
        try:
            while True:
                if self.stop_evt.is_set() or self.elapsed() >= self.duration:
                    normal_end = True
                    break
                if self.writer_err is not None:
                    raise self.writer_err
                data = carry + ser.read(max(1, ser.in_waiting))
                carry = b""
                now = time.perf_counter()
                frames, bad = parser.feed(data) if data else ([], 0)
                t = now - self.t0
                with self.lock:
                    if bad:
                        self.bins["rx_bad"][self._bin(t)] += bad
                        self._event(t, "Fehlerhafte Daten", f"{bad}x CRC-/Sync-Fehler")
                    for seq, g, l in frames:
                        idx = self._bin(t)
                        gap = now - t_last
                        t_last = now
                        stalled = False
                        if first:
                            first = False
                            self._outage_end(t)
                            expected, last_g, last_l = seq, g, l
                        else:
                            if gap > thr:
                                self.bins["gap_ms"][idx] = max(self.bins["gap_ms"][idx], gap * 1000)
                                self._event(t, "Datenlücke", f"{gap * 1000:.0f} ms ohne Daten")
                            if seq < expected:
                                self._event(t, "ESP-Reset?", "Sequenz zurückgesprungen - ESP neu gestartet")
                                last_g, last_l = g, l
                            elif seq > expected:
                                lost = seq - expected
                                self.bins["rx_lost"][idx] += lost
                                self._event(t, "Frames verloren", f"{lost} Frame(s) ESP->PC fehlen")
                        expected = seq + 1
                        self.bins["rx_bytes"][idx] += FRAME_LEN
                        if g < last_g or l < last_l:
                            last_g, last_l = g, l
                        dg, dl = g - last_g, l - last_l
                        self.bins["tx_bytes"][idx] += dg * FRAME_LEN
                        self.bins["tx_lost"][idx] += dl
                        if dl:
                            self._event(t, "PC->ESP Verlust", f"{dl} Frame(s) beim ESP verloren/fehlerhaft")
                        last_g, last_l = g, l
                        self.last_frame_t = t
                if not frames:
                    idle = now - t_last
                    if idle > LOST_TIMEOUT:
                        raise LinkLost(f"keine Daten seit {idle:.1f} s")
                    if idle > 1.0 and not stalled:
                        stalled = True
                        self._event(t, "Stillstand", "seit >1 s keine Daten")
        finally:
            stop_w.set()
            wt.join(1.0)
        if normal_end:
            try:
                ser.write(b"X\n")
            except Exception:
                pass


class App:
    MODES = {"Beide Richtungen": 3, "ESP -> PC (Download)": 1, "PC -> ESP (Upload)": 2}
    DURATIONS = {"30 s": 30, "1 min": 60, "5 min": 300, "10 min": 600,
                 "30 min": 1800, "60 min": 3600, "Endlos": 0}
    WINDOWS = {"Alles": None, "letzte 1 min": 60, "letzte 5 min": 300,
               "letzte 10 min": 600, "letzte 30 min": 1800}
    BAUDS = [115200, 230400, 460800, 921600, 1500000, 2000000, 3000000]

    def __init__(self, root):
        self.root = root
        root.title("USB-Strecken-Tester  Laptop <-> ESP32")
        root.geometry("1250x900")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.tester = None
        self.spans = []
        self.ev_shown = 0

        top = ttk.Frame(root, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="e")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=36, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=3)
        ttk.Button(top, text="Neu laden", command=self.refresh_ports).grid(row=0, column=2)
        ttk.Label(top, text="Baudrate:").grid(row=0, column=3, sticky="e", padx=(12, 0))
        self.baud_var = tk.StringVar(value="921600")
        ttk.Combobox(top, textvariable=self.baud_var, values=self.BAUDS, width=9).grid(row=0, column=4, padx=3)
        ttk.Label(top, text="Richtung:").grid(row=0, column=5, sticky="e", padx=(12, 0))
        self.mode_var = tk.StringVar(value="Beide Richtungen")
        ttk.Combobox(top, textvariable=self.mode_var, values=list(self.MODES), width=20,
                     state="readonly").grid(row=0, column=6, padx=3)
        ttk.Label(top, text="Testdauer:").grid(row=0, column=7, sticky="e", padx=(12, 0))
        self.dur_var = tk.StringVar(value="1 min")
        ttk.Combobox(top, textvariable=self.dur_var, values=list(self.DURATIONS), width=8,
                     state="readonly").grid(row=0, column=8, padx=3)
        self.btn_start = ttk.Button(top, text="Start", command=self.start)
        self.btn_start.grid(row=0, column=9, padx=(12, 3))
        self.btn_stop = ttk.Button(top, text="Stop", command=self.stop, state="disabled")
        self.btn_stop.grid(row=0, column=10)
        ttk.Label(top, text="Anzeigefenster:").grid(row=1, column=3, columnspan=2, sticky="e", pady=(6, 0))
        self.win_var = tk.StringVar(value="Alles")
        self.win_cb = ttk.Combobox(top, textvariable=self.win_var, values=list(self.WINDOWS),
                                   width=14, state="readonly")
        self.win_cb.grid(row=1, column=5, columnspan=2, sticky="w", pady=(6, 0), padx=3)
        self.win_cb.bind("<<ComboboxSelected>>", lambda e: self.redraw())
        ttk.Button(top, text="CSV speichern", command=self.save_csv).grid(row=1, column=9, columnspan=2,
                                                                        pady=(6, 0), padx=(12, 0))

        self.status_var = tk.StringVar(value="Bereit.  (Serial Monitor der Arduino IDE schließen!)")
        ttk.Label(root, textvariable=self.status_var, font=("Consolas", 10), padding=(8, 0)).pack(fill="x")

        figf = ttk.Frame(root)
        figf.pack(fill="both", expand=True)
        self.fig = Figure(figsize=(11, 6), dpi=100, constrained_layout=True)
        self.ax1, self.ax2, self.ax3 = self.fig.subplots(3, 1, sharex=True)
        (self.l_rx,) = self.ax1.plot([], [], color="tab:blue", lw=1.2, label="ESP → PC")
        (self.l_tx,) = self.ax1.plot([], [], color="tab:green", lw=1.2, label="PC → ESP")
        self.l_theo = self.ax1.axhline(0, color="gray", ls="--", lw=1, label="theoretisches Maximum")
        self.ax1.set_ylabel("Durchsatz [kB/s]")
        self.ax1.legend(loc="lower left", fontsize=8)
        (self.l_lost,) = self.ax2.step([], [], where="post", color="tab:red", label="Frames verloren ESP→PC")
        (self.l_bad,) = self.ax2.step([], [], where="post", color="tab:orange", label="CRC-/Sync-Fehler")
        (self.l_txl,) = self.ax2.step([], [], where="post", color="tab:purple", label="Frames verloren PC→ESP")
        self.ax2.set_ylabel(f"Fehler pro {BIN:g} s")
        self.ax2.legend(loc="upper left", fontsize=8)
        (self.l_gap,) = self.ax3.step([], [], where="post", color="tab:brown", label="größte Datenlücke")
        self.l_thr = self.ax3.axhline(0, color="red", ls="--", lw=1, label="Alarmschwelle")
        self.ax3.set_ylabel("Lücke [ms]")
        self.ax3.set_xlabel("Zeit [s]")
        self.ax3.legend(loc="upper left", fontsize=8)
        for ax in (self.ax1, self.ax2, self.ax3):
            ax.grid(True, alpha=0.3)
        self.canvas = FigureCanvasTkAgg(self.fig, master=figf)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.canvas, figf)

        bottom = ttk.Frame(root, padding=6)
        bottom.pack(fill="x")
        self.summary = tk.Text(bottom, height=11, width=88, font=("Consolas", 9), state="disabled")
        self.summary.pack(side="left", fill="y")
        evf = ttk.Frame(bottom)
        evf.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.tree = ttk.Treeview(evf, columns=("t", "art", "info"), show="headings", height=10)
        for c, txt, w in (("t", "Zeit", 70), ("art", "Ereignis", 160), ("info", "Details", 380)):
            self.tree.heading(c, text=txt)
            self.tree.column(c, width=w, anchor="w")
        sb = ttk.Scrollbar(evf, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")

        self.refresh_ports()

    # ---------- Steuerung ----------
    def refresh_ports(self):
        ports = [f"{p.device} - {p.description}" for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def start(self):
        if self.tester and self.tester.is_alive():
            return
        if not self.port_var.get():
            messagebox.showerror("Port", "Bitte einen COM-Port wählen.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Baudrate", "Ungültige Baudrate.")
            return
        dur = self.DURATIONS[self.dur_var.get()]
        self.tester = Tester(self.port_var.get().split(" ")[0], baud, self.MODES[self.mode_var.get()],
                             dur if dur else float("inf"))
        self.tree.delete(*self.tree.get_children())
        self.ev_shown = 0
        self.btn_start.state(["disabled"])
        self.btn_stop.state(["!disabled"])
        self.status_var.set("Verbinde ...")
        self.tester.start()
        self.root.after(500, self.tick)

    def stop(self):
        if self.tester:
            self.tester.stop_evt.set()

    def on_close(self):
        self.stop()
        self.root.after(200, self.root.destroy)

    def tick(self):
        t = self.tester
        if not t:
            return
        running = not t.done
        self.update_view(running)
        if running:
            self.root.after(1000, self.tick)
        else:
            self.btn_start.state(["!disabled"])
            self.btn_stop.state(["disabled"])
            if t.error:
                messagebox.showerror("Test abgebrochen", t.error)

    def redraw(self):
        if self.tester:
            self.update_view(not self.tester.done)

    # ---------- Darstellung ----------
    def update_view(self, running):
        t = self.tester
        d, ev, outages, now_t = t.snapshot()
        n = len(d["rx_bytes"]) - (1 if running else 0)
        if n >= 1:
            self._draw(d, outages, now_t, n)
        self._status(d, outages, now_t, n, running)
        self._set_summary(self._make_summary(d, outages, now_t, n, running))
        new = ev[self.ev_shown:]
        for te, kind, txt in new:
            self.tree.insert("", "end", values=(f"{te:.1f} s", kind, txt))
        if new:
            self.tree.yview_moveto(1)
        self.ev_shown = len(ev)

    def _draw(self, d, outages, now_t, n):
        t = self.tester
        x = (np.arange(n) + 1) * BIN
        rx = d["rx_bytes"][:n] / BIN / 1000
        tx = d["tx_bytes"][:n] / BIN / 1000
        theo = t.baud / 10 / 1000
        win = self.WINDOWS[self.win_var.get()]
        xmax = x[-1]
        x0 = 0 if win is None else max(0, xmax - win)
        mask = x >= x0

        self.l_rx.set_data(*((x, rx) if t.mode & 1 else ([], [])))
        self.l_tx.set_data(*((x, tx) if t.mode & 2 else ([], [])))
        self.l_theo.set_ydata([theo, theo])
        top = theo
        if t.mode & 1:
            top = max(top, rx[mask].max())
        if t.mode & 2:
            top = max(top, tx[mask].max())
        self.ax1.set_ylim(0, top * 1.08)

        lost, bad, txl = d["rx_lost"][:n], d["rx_bad"][:n], d["tx_lost"][:n]
        self.l_lost.set_data(x, lost)
        self.l_bad.set_data(x, bad)
        self.l_txl.set_data(x, txl)
        self.ax2.set_ylim(0, max(1.0, lost[mask].max(), bad[mask].max(), txl[mask].max()) * 1.2)

        gap = d["gap_ms"][:n]
        thr_ms = t.gap_thr * 1000
        self.l_gap.set_data(x, gap)
        self.l_thr.set_ydata([thr_ms, thr_ms])
        self.ax3.set_ylim(0, max(thr_ms * 1.3, gap[mask].max() * 1.1))
        self.ax3.set_xlim(x0, max(xmax, x0 + 1))

        for s in self.spans:
            s.remove()
        self.spans = []
        for a, b in outages[-200:]:
            b = now_t if b is None else b
            for ax in (self.ax1, self.ax2, self.ax3):
                self.spans.append(ax.axvspan(a, max(b, a + BIN), color="red", alpha=0.25))
        self.canvas.draw_idle()

    def _status(self, d, outages, now_t, n, running):
        t = self.tester
        if n < 1:
            self.status_var.set("Läuft ... warte auf Daten" if running else "Keine Daten empfangen.")
            return
        last = slice(max(0, n - 2), n)
        rxr = d["rx_bytes"][last].mean() / BIN / 1000
        txr = d["tx_bytes"][last].mean() / BIN / 1000
        dur = "∞" if t.duration == float("inf") else f"{int(t.duration) // 60:02d}:{int(t.duration) % 60:02d}"
        el = int(now_t)
        errs = int(d["rx_lost"][:n].sum() + d["rx_bad"][:n].sum() + d["tx_lost"][:n].sum())
        txt = (f"{'LÄUFT' if running else 'FERTIG'} {el // 60:02d}:{el % 60:02d} / {dur} | "
               f"RX {rxr:7.1f} kB/s | TX {txr:7.1f} kB/s | Fehler {errs} | Ausfälle {len(outages)}")
        self.status_var.set(txt)

    def _make_summary(self, d, outages, now_t, n, running):
        t = self.tester
        if n < 1:
            return "Warte auf Daten ..."
        sl = slice(1, n) if n > 2 else slice(0, n)
        theo = t.baud / 10 / 1000
        L = [f"Ausgewertet: {n * BIN:.0f} s   |   Baudrate {t.baud}  ->  theor. Max {theo:.0f} kB/s (8N1)", ""]

        def rate(name, key):
            a = d[key][sl] / BIN / 1000
            if a.size:
                L.append(f"{name}: Ø {a.mean():.1f} | min {a.min():.1f} | max {a.max():.1f} | "
                         f"5%-Perz. {np.percentile(a, 5):.1f} kB/s  ({a.mean() / theo * 100:.0f}% vom Max)")

        if t.mode & 1:
            rate("ESP→PC", "rx_bytes")
        if t.mode & 2:
            rate("PC→ESP", "tx_bytes")
        ok_rx = d["rx_bytes"][:n].sum() / FRAME_LEN
        lost = d["rx_lost"][:n].sum()
        bad = d["rx_bad"][:n].sum()
        ok_tx = d["tx_bytes"][:n].sum() / FRAME_LEN
        txl = d["tx_lost"][:n].sum()
        gaps = int((d["gap_ms"][:n] > 0).sum())
        maxgap = d["gap_ms"][:n].max() if n else 0
        down = sum((now_t if b is None else b) - a for a, b in outages)
        L.append("")
        L.append(f"Frames ESP→PC: {ok_rx:.0f} ok, {lost:.0f} verloren, {bad:.0f} CRC-/Sync-Fehler")
        if t.mode & 2:
            L.append(f"Frames PC→ESP: {ok_tx:.0f} ok, {txl:.0f} verloren")
        L.append(f"Datenlücken über Schwelle: {gaps}  (längste {maxgap:.0f} ms)")
        L.append(f"Verbindungsabbrüche: {len(outages)}  (Ausfallzeit gesamt {down:.1f} s)")
        L.append("")
        problems = lost + bad + txl + gaps + len(outages)
        if problems == 0:
            L.append("ERGEBNIS: OK - keine Fehler, Strecke stabil." + ("" if not running else "  (läuft noch)"))
        else:
            L.append("ERGEBNIS: FEHLER erkannt - Details siehe Ereignisliste / rote Bereiche im Diagramm.")
        return "\n".join(L)

    def _set_summary(self, text):
        self.summary.configure(state="normal")
        self.summary.delete("1.0", "end")
        self.summary.insert("1.0", text)
        self.summary.configure(state="disabled")

    # ---------- Export ----------
    def save_csv(self):
        if not self.tester:
            return
        d, ev, outages, now_t = self.tester.snapshot()
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            initialfile=time.strftime("linktest_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f, delimiter=";")
            w.writerow(["zeit_s", "rx_kBps", "tx_kBps", "rx_frames_verloren", "rx_crc_sync_fehler",
                        "tx_frames_verloren", "max_luecke_ms"])
            for i in range(len(d["rx_bytes"])):
                w.writerow([f"{(i + 1) * BIN:.1f}", f"{d['rx_bytes'][i] / BIN / 1000:.2f}",
                            f"{d['tx_bytes'][i] / BIN / 1000:.2f}", int(d["rx_lost"][i]),
                            int(d["rx_bad"][i]), int(d["tx_lost"][i]), f"{d['gap_ms'][i]:.0f}"])
        with open(os.path.splitext(path)[0] + "_events.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f, delimiter=";")
            w.writerow(["zeit_s", "ereignis", "details"])
            for te, kind, txt in ev:
                w.writerow([f"{te:.2f}", kind, txt])
        messagebox.showinfo("Gespeichert", f"{path}\nund *_events.csv")


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()