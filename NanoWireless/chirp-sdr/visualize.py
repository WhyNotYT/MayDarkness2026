#!/usr/bin/env python3
"""
Flask + JS realtime visualizer for the Pico W chirp receiver's raw ADC
debug stream. Replaces the matplotlib version: a background thread keeps
pulling frames from the Pico over serial as fast as it can, the numeric
crunching (FFT, per-bin activity/correlation) happens once in Python with
numpy, and the browser just polls a small JSON endpoint and draws with
<canvas> -- much faster to redraw than matplotlib's figure pipeline.

Usage:
    pip install flask pyserial numpy
    python app.py --port /dev/ttyACM0
    -> open http://127.0.0.1:5000
"""

import argparse
import re
import struct
import threading
import time

import numpy as np
import serial
from flask import Flask, jsonify, send_from_directory

SYNC = bytes([0xAA, 0x55, 0xAA, 0x55])
END_MARKER = bytes([0xFF, 0xFF, 0xFF, 0xFF])
BINS = 10  # must match BINS in main-receiver.cpp

app = Flask(__name__, static_folder="static", static_url_path="")

# Shared state written by the reader thread, read by Flask request threads.
_lock = threading.Lock()
_latest = {
    "t_ms": [],
    "waveform": [],
    "bin_activity": [],
    "r": 0.0,
    "waterfall_row": [],
    "waterfall_freqs": [],
    "frame_count": 0,
    "error": None,
}

# The firmware's normal decode loop narrates everything it's doing over
# Serial.print/println (sync/onset waiting, preamble/message decode, the
# final decoded byte) -- separately from the binary 's' stream dump. The
# old version of read_frame() below just threw those bytes away while
# hunting for the stream's sync marker. We now capture them as text lines,
# parse out a coarse decoder "state" and any decoded message, and expose
# both to the web page alongside the raw log lines.
MAX_LOG_LINES = 500
MAX_DECODED_MESSAGES = 50

_log_lines = []          # rolling buffer of recent log lines, oldest first
_decoder_state = "unknown"
_decoded_messages = []   # rolling buffer of {hex, char, parity_ok, ts}, oldest first
_line_leftover = bytearray()  # bytes not yet terminated by \n, carried across reads


def _parse_log_line_locked(text):
    """Update _decoder_state / _decoded_messages from one log line. Caller
    must already hold _lock."""
    global _decoder_state

    if "waiting for quiet" in text:
        _decoder_state = "waiting_for_quiet"
    elif "quiet detected" in text:
        _decoder_state = "listening_for_onset"
    elif "signal onset" in text:
        _decoder_state = "frame_start"
    elif "decoding preamble" in text:
        _decoder_state = "decoding_preamble"
    elif "preamble matched" in text:
        _decoder_state = "preamble_ok"
    elif "mismatch" in text and "preamble" in text:
        _decoder_state = "preamble_mismatch"
    elif "decoding message" in text:
        _decoder_state = "decoding_message"
    elif text.startswith("=== DECODED BYTE:"):
        _decoder_state = "decoded"
        m = re.search(r"0x([0-9A-Fa-f]+) = '(.)'.*parity (OK|MISMATCH)", text)
        if m:
            _decoded_messages.append({
                "hex": m.group(1),
                "char": m.group(2),
                "parity_ok": m.group(3) == "OK",
                "ts": time.time(),
            })
            if len(_decoded_messages) > MAX_DECODED_MESSAGES:
                del _decoded_messages[0 : len(_decoded_messages) - MAX_DECODED_MESSAGES]


def capture_log_bytes(data: bytes):
    """Feed raw bytes pulled off the serial port (that weren't part of a
    binary stream frame) into the line buffer, emitting complete lines into
    the shared log/state as they arrive. Safe to call with empty/partial
    data at any time."""
    global _line_leftover
    if not data:
        return
    _line_leftover += data
    while b"\n" in _line_leftover:
        line, _, rest = _line_leftover.partition(b"\n")
        _line_leftover = bytearray(rest)
        text = line.decode("utf-8", errors="replace").rstrip("\r").strip()
        if not text:
            continue
        with _lock:
            _log_lines.append(text)
            if len(_log_lines) > MAX_LOG_LINES:
                del _log_lines[0 : len(_log_lines) - MAX_LOG_LINES]
            _parse_log_line_locked(text)


WATERFALL_BINS = 128


def compute_waterfall_row(samples: np.ndarray, sample_rate: float, nbins: int = WATERFALL_BINS):
    """One full-frame FFT, bucketed down to a fixed number of bins, in raw
    dB (not pre-normalized) -- the client auto-contrasts against whatever
    range is actually in view, so it adapts instead of clipping to a
    fixed range that may not match your signal's actual levels."""
    windowed = samples - np.mean(samples)
    spectrum = np.abs(np.fft.rfft(windowed * np.hanning(len(windowed))))
    freqs = np.fft.rfftfreq(len(windowed), d=1.0 / sample_rate)

    edges = np.linspace(0, len(spectrum), nbins + 1).astype(int)
    bucketed = np.array([
        spectrum[edges[i]:edges[i + 1]].mean() if edges[i + 1] > edges[i] else 0.0
        for i in range(nbins)
    ])
    bucket_freqs = np.array([
        freqs[edges[i]:edges[i + 1]].mean() if edges[i + 1] > edges[i] else freqs[min(edges[i], len(freqs) - 1)]
        for i in range(nbins)
    ])

    db = 20 * np.log10(bucketed + 1e-6)
    return db.tolist(), bucket_freqs.tolist()


def read_frame(ser: serial.Serial) -> np.ndarray:
    """Trigger one stream dump and read the raw uint16 samples back.

    Anything read off the port that isn't part of the binary stream frame
    is the decode loop's own narration (sync/onset waiting, preamble/
    message decode, decoded byte, etc.) -- capture it as log text instead
    of discarding it, so the web page can show it too.
    """
    # Grab whatever's already sitting in the OS buffer -- this is decode-loop
    # log output, not stream data, so capture it as text rather than
    # resetting/discarding it.
    pending = ser.in_waiting
    if pending:
        capture_log_bytes(ser.read(pending))

    ser.write(b"s\n")

    buf = bytearray()
    text_buf = bytearray()
    deadline = time.time() + 8.0  # a decode-in-progress can delay response by a couple seconds
    while True:
        if time.time() > deadline:
            capture_log_bytes(bytes(text_buf))
            raise TimeoutError("timed out waiting for stream sync bytes")
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if len(buf) > 4:
            # Everything before the trailing 4 bytes can no longer be part of
            # the sync sequence -- it's log text the decode loop printed
            # while we were waiting for it to notice the 's' command.
            text_buf += buf[:-4]
            buf = buf[-4:]
        if bytes(buf) == SYNC:
            break
    capture_log_bytes(bytes(text_buf))

    count_bytes = ser.read(4)
    if len(count_bytes) != 4:
        raise TimeoutError("timed out reading sample count")
    (count,) = struct.unpack("<I", count_bytes)

    raw = ser.read(count * 2)
    if len(raw) != count * 2:
        raise TimeoutError(f"expected {count*2} bytes of samples, got {len(raw)}")
    samples = np.frombuffer(raw, dtype="<u2").astype(np.float64)

    ser.read(4)  # end marker, not strictly checked here
    return samples


def bin_activity_and_correlation(samples: np.ndarray, bins: int = BINS):
    """Mirrors sampleChirpBit()'s per-bin mean-abs-delta + Pearson r."""
    chunks = np.array_split(samples, bins)
    activity = np.array(
        [np.mean(np.abs(np.diff(c))) if len(c) > 1 else 0.0 for c in chunks]
    )
    template = np.arange(bins, dtype=np.float64)
    if np.std(activity) > 1e-9 and np.std(template) > 1e-9:
        r = float(np.corrcoef(activity, template)[0, 1])
    else:
        r = 0.0
    return activity, r


def reader_thread(port: str, baud: int, sample_rate: float, max_points: int):
    ser = serial.Serial(port, baud, timeout=3)
    while True:
        try:
            samples = read_frame(ser)
        except TimeoutError as e:
            with _lock:
                _latest["error"] = str(e)
            time.sleep(0.5)
            continue

        # Downsample the waveform for the browser (raw counts can be
        # thousands of points; the eye can't tell the difference past a
        # couple thousand pixels wide, and it keeps JSON payloads small).
        if len(samples) > max_points:
            idx = np.linspace(0, len(samples) - 1, max_points).astype(int)
            wave_ds = samples[idx]
            t_ds = idx / sample_rate * 1000.0
        else:
            wave_ds = samples
            t_ds = np.arange(len(samples)) / sample_rate * 1000.0

        activity, r = bin_activity_and_correlation(samples)
        waterfall_row, waterfall_freqs = compute_waterfall_row(samples, sample_rate)

        with _lock:
            _latest["t_ms"] = t_ds.tolist()
            _latest["waveform"] = wave_ds.tolist()
            _latest["bin_activity"] = activity.tolist()
            _latest["r"] = r
            _latest["waterfall_row"] = waterfall_row
            _latest["waterfall_freqs"] = waterfall_freqs
            _latest["frame_count"] += 1
            _latest["error"] = None


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/api/frame")
def api_frame():
    with _lock:
        payload = dict(_latest)
        payload["decoder_state"] = _decoder_state
        payload["decoded_messages"] = list(_decoded_messages)
        payload["log_lines"] = list(_log_lines)
        return jsonify(payload)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sample-rate", type=float, default=500000.0,
                     help="Must match FAST_ADC_SAMPLE_RATE_HZ in firmware")
    ap.add_argument("--max-points", type=int, default=2000,
                     help="Downsample waveform/spectrum to this many points before sending to the browser")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--web-port", type=int, default=5000)
    args = ap.parse_args()

    t = threading.Thread(
        target=reader_thread,
        args=(args.port, args.baud, args.sample_rate, args.max_points),
        daemon=True,
    )
    t.start()

    app.run(host=args.host, port=args.web_port, debug=False, threaded=True)


if __name__ == "__main__":
    main()