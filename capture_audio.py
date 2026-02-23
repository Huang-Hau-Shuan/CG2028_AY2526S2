#!/usr/bin/env python3
"""
capture_audio.py — Receive audio from the STM32 Grove Sound Sensor
                    over a serial port and save to a WAV file.

Usage:
    python capture_audio.py [COM_PORT] [OUTPUT_FILE]

Examples:
    python capture_audio.py COM3 fall_audio.wav
    python capture_audio.py COM5 recording.wav

Requires:
    pip install pyserial

Protocol (matches sound_sensor.c):
    Each chunk:
      [0xAA][0x55]             — 2-byte sync header
      [len_hi][len_lo]         — number of uint16 samples (big-endian)
      [sample0_lo][sample0_hi] — little-endian uint16 samples × len
    Lines starting with '#' are debug messages (printed to console).

The captured PCM data is written to a WAV file:
    16-bit signed PCM, 1 channel, 8000 Hz (matches SOUND_SAMPLE_RATE_HZ).
    ADC values (0-4095 unsigned) are re-centred to signed int16 for WAV.
"""

import sys
import struct
import wave
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)

# ---------- Configuration (match sound_sensor.h) ----------
BAUD_RATE    = 115200
SAMPLE_RATE  = 8000      # Must match SOUND_SAMPLE_RATE_HZ
SYNC_BYTE_0  = 0xAA
SYNC_BYTE_1  = 0x55
# -----------------------------------------------------------

def main():
    # Parse arguments
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <COM_PORT> [output.wav]")
        print(f"  e.g. {sys.argv[0]} COM3 fall_audio.wav")
        sys.exit(1)

    port     = sys.argv[1]
    out_file = sys.argv[2] if len(sys.argv) > 2 else "captured_audio.wav"

    print(f"[*] Opening {port} at {BAUD_RATE} baud ...")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(0.5)          # let STM32 boot / reset

    samples = []             # collected int16 samples
    chunks_received = 0

    print(f"[*] Listening for audio chunks (Ctrl+C to stop and save) ...")
    print(f"    Debug messages from the board are shown below.\n")

    try:
        while True:
            # ---- Check for debug text lines (start with '#') ----
            # Peek at the next byte; if it's '#' read a full text line
            b = ser.read(1)
            if len(b) == 0:
                continue   # timeout, no data

            if b[0] == ord('#'):
                # Read the rest of the line
                line = b + ser.readline()
                print(line.decode('ascii', errors='replace').rstrip())
                continue

            # ---- Look for sync header 0xAA 0x55 ----
            if b[0] != SYNC_BYTE_0:
                continue   # not sync — skip
            b2 = ser.read(1)
            if len(b2) == 0 or b2[0] != SYNC_BYTE_1:
                continue   # false start

            # ---- Read chunk length (2 bytes, big-endian) ----
            hdr = ser.read(2)
            if len(hdr) < 2:
                continue
            num_samples = (hdr[0] << 8) | hdr[1]
            if num_samples == 0 or num_samples > 8192:
                continue   # sanity check

            # ---- Read sample data (num_samples × 2 bytes, little-endian uint16) ----
            raw = ser.read(num_samples * 2)
            if len(raw) < num_samples * 2:
                print(f"  [!] Incomplete chunk ({len(raw)}/{num_samples*2} bytes)")
                continue

            # Convert unsigned 12-bit ADC (0-4095) to signed int16 for WAV
            for i in range(num_samples):
                val = struct.unpack_from('<H', raw, i * 2)[0]  # uint16 LE
                # Re-centre: 0-4095 → -2048..+2047, then scale to int16 range
                signed_val = (int(val) - 2048) * 16   # ×16 to use full int16 range
                signed_val = max(-32768, min(32767, signed_val))
                samples.append(signed_val)

            chunks_received += 1
            elapsed = len(samples) / SAMPLE_RATE
            print(f"  Chunk {chunks_received}: {num_samples} samples  |  "
                  f"Total: {len(samples)} samples ({elapsed:.1f} s)", end='\r')

    except KeyboardInterrupt:
        print(f"\n\n[*] Stopped.  Received {len(samples)} samples "
              f"({len(samples)/SAMPLE_RATE:.1f} s of audio).")

    ser.close()

    if len(samples) == 0:
        print("[!] No audio captured — nothing to save.")
        return

    # ---- Write WAV file ----
    print(f"[*] Saving to {out_file} ...")
    with wave.open(out_file, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)          # 16-bit
        wf.setframerate(SAMPLE_RATE)
        # Pack samples as signed int16 little-endian
        wf.writeframes(struct.pack(f'<{len(samples)}h', *samples))

    print(f"[*] Done!  {out_file}  ({len(samples)} samples, "
          f"{len(samples)/SAMPLE_RATE:.1f} s, 16-bit PCM, {SAMPLE_RATE} Hz)")
    print(f"    Open in Audacity or any media player to listen.")


if __name__ == '__main__':
    main()
