import argparse
import sys
import time
from dataclasses import dataclass

import serial


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def cobs_decode(packet: bytes) -> bytes:
    """
    Decode COBS (packet does NOT include trailing 0x00 delimiter)
    Returns decoded bytes or raises ValueError
    """
    if not packet:
        raise ValueError("empty COBS packet")

    out = bytearray()
    idx = 0
    n = len(packet)

    while idx < n:
        code = packet[idx]
        if code == 0:
            raise ValueError("COBS code=0 inside packet")
        idx += 1

        copy_len = code - 1
        if idx + copy_len > n and code != 1:
            raise ValueError("COBS overrun")

        out.extend(packet[idx: idx + copy_len])
        idx += copy_len

        if code != 0xFF and idx < n:
            out.append(0x00)

    return bytes(out)


@dataclass
class Frame:
    ftype: int
    seq: int
    payload: bytes
    crc_ok: bool


def parse_frame(decoded: bytes) -> Frame | None:
    """
    decoded format (your firmware):
      [0]=0xB2
      [1]=type
      [2]=seq
      [3]=len
      [4..4+len-1]=payload
      [4+len]=crc_lo
      [5+len]=crc_hi
    CRC computed over decoded[1 .. 1+(3+len)-1] == bytes([type,seq,len] + payload)
    """
    if len(decoded) < 6:
        return None
    if decoded[0] != 0xB2:
        return None

    ftype = decoded[1]
    seq = decoded[2]
    plen = decoded[3]
    need = 4 + plen + 2
    if len(decoded) != need:
        # strict: must match exactly
        return None

    payload = decoded[4: 4 + plen]
    crc_lo = decoded[4 + plen]
    crc_hi = decoded[5 + plen]
    rx_crc = crc_lo | (crc_hi << 8)

    calc = crc16_ccitt(decoded[1: 4 + plen])  # type,seq,len,payload
    return Frame(ftype=ftype, seq=seq, payload=payload, crc_ok=(rx_crc == calc))


def hexdump(b: bytes, maxlen: int = 64) -> str:
    if len(b) > maxlen:
        b = b[:maxlen] + b"..."
    return " ".join(f"{x:02X}" for x in b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM11")
    ap.add_argument("--baud", type=int, default=115200, help="USB serial baud (S3 console baud)")
    ap.add_argument("--raw", action="store_true", help="print raw frames (hex)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"[OK] Open {args.port} @ {args.baud}")
    print("[INFO] Waiting COBS frames delimited by 0x00 ...")

    buf = bytearray()
    ok = 0
    bad = 0
    last_stat = time.time()

    while True:
        chunk = ser.read(4096)
        if chunk:
            for c in chunk:
                if c == 0x00:
                    if not buf:
                        continue
                    packet = bytes(buf)
                    buf.clear()

                    try:
                        decoded = cobs_decode(packet)
                    except Exception:
                        bad += 1
                        continue

                    fr = parse_frame(decoded)
                    if fr is None:
                        bad += 1
                        continue

                    if not fr.crc_ok:
                        bad += 1
                        continue

                    ok += 1

                    # --- Print decoded meaning (minimal) ---
                    if args.raw:
                        print(f"OK type={fr.ftype} seq={fr.seq} len={len(fr.payload)} payload={hexdump(fr.payload)}")
                    else:
                        # If draw job: your JOB_DRAW type == 5 (in your enum), and payload[0]=0x10, payload[1]=cnt
                        if fr.ftype == 5 and len(fr.payload) >= 2 and fr.payload[0] == 0x10:
                            cnt = fr.payload[1]
                            pts = fr.payload[2:]
                            # each pt = 5 bytes: xlo,xhi,ylo,yhi,p
                            print(f"DRAW seq={fr.seq} cnt={cnt} bytes={len(fr.payload)}")
                            show = min(cnt, 3)
                            for i in range(show):
                                off = i * 5
                                if off + 5 <= len(pts):
                                    x = pts[off] | (pts[off+1] << 8)
                                    y = pts[off+2] | (pts[off+3] << 8)
                                    p = pts[off+4]
                                    print(f"  pt{i}: x={x} y={y} p={p}")
                        else:
                            print(f"CTRL type={fr.ftype} seq={fr.seq} len={len(fr.payload)} payload0={fr.payload[0] if fr.payload else None}")

        now = time.time()
        if now - last_stat >= 1.0:
            last_stat = now
            print(f"[STAT] ok={ok} bad={bad} buf={len(buf)}")

        # allow Ctrl+C
        # (no extra sleep; timeout handles it)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT] bye")
        sys.exit(0)
