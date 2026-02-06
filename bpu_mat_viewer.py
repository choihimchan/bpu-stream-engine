import sys, time
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt

MAGIC = 0xB2

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def cobs_decode(inp: bytes):
    out = bytearray()
    i = 0
    n = len(inp)
    while i < n:
        code = inp[i]
        if code == 0:
            return None
        i += 1
        for _ in range(1, code):
            if i >= n:
                return None
            out.append(inp[i])
            i += 1
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)

def pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    scored = []
    for p in ports:
        desc = (p.description or "").lower()
        dev  = (p.device or "").lower()
        score = 0
        if "usb" in desc: score += 2
        if "acm" in dev: score += 2
        if "ttyusb" in dev: score += 2
        if "uart" in desc: score += 1
        scored.append((score, p.device, p.description))
    scored.sort(reverse=True)
    return scored[0][1]

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else pick_port()
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200

    if not port:
        print("No serial port found. Usage: python bpu_mat_viewer.py COM7")
        sys.exit(1)

    print(f"Opening {port} @ {baud}")
    ser = serial.Serial(port, baudrate=baud, timeout=0.0)

    # plot setup
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title("BPU Draw Viewer (matplotlib)")
    ax.set_xlim(0, 1023)
    ax.set_ylim(0, 1023)
    ax.invert_yaxis()  # 화면 좌표 느낌
    scat = ax.scatter([], [], s=2)

    buf = bytearray()

    ok=drop=cobs_fail=magic_fail=crc_fail=len_fail=seq_gap=0
    last_seq=None
    draw_pts=0

    xs, ys = [], []
    MAX_POINTS = 20000  # 너무 많으면 느려짐

    last_ui = time.time()
    last_stat = time.time()
    ok0 = 0
    draw0 = 0
    pps = 0.0
    dps = 0.0

    while True:
        # read
        data = ser.read(8192)
        if data:
            buf.extend(data)

        # parse by delimiter 0x00
        updated = False
        while True:
            try:
                idx = buf.index(0)
            except ValueError:
                break
            frame = bytes(buf[:idx])
            del buf[:idx+1]
            if not frame:
                continue

            dec = cobs_decode(frame)
            if dec is None:
                cobs_fail += 1; drop += 1; continue
            if len(dec) < 6:
                len_fail += 1; drop += 1; continue
            if dec[0] != MAGIC:
                magic_fail += 1; drop += 1; continue

            ptype = dec[1]
            seq   = dec[2]
            plen  = dec[3]
            need  = 4 + plen + 2
            if len(dec) != need:
                len_fail += 1; drop += 1; continue

            got  = dec[4+plen] | (dec[4+plen+1] << 8)
            calc = crc16_ccitt(dec[1:1+(3+plen)])
            if got != calc:
                crc_fail += 1; drop += 1; continue

            if last_seq is not None:
                expect = (last_seq + 1) & 0xFF
                if seq != expect:
                    seq_gap += 1
            last_seq = seq

            ok += 1

            if ptype == 5 and plen >= 2:
                payload = dec[4:4+plen]
                if payload[0] == 0x10:
                    cnt = payload[1]
                    if 2 + cnt*5 == plen:
                        draw_pts += cnt
                        base = 2
                        for i in range(cnt):
                            off = base + i*5
                            x = payload[off+0] | (payload[off+1] << 8)
                            y = payload[off+2] | (payload[off+3] << 8)
                            xs.append(x)
                            ys.append(y)
                        # cap
                        if len(xs) > MAX_POINTS:
                            extra = len(xs) - MAX_POINTS
                            xs = xs[extra:]
                            ys = ys[extra:]
                        updated = True

        now = time.time()
        if now - last_stat >= 0.5:
            dt = now - last_stat
            pps = (ok - ok0) / dt if dt > 0 else 0.0
            dps = (draw_pts - draw0) / dt if dt > 0 else 0.0
            ok0 = ok
            draw0 = draw_pts
            last_stat = now
            fig.suptitle(
                f"ok={ok} drop={drop} pps={pps:.1f} drawPts={draw_pts} dps={dps:.1f} | "
                f"cobs={cobs_fail} magic={magic_fail} crc={crc_fail} len={len_fail} seqGap={seq_gap}"
            )

        # update UI (throttle)
        if updated and (now - last_ui) >= 0.05:
            scat.set_offsets(list(zip(xs, ys)))
            fig.canvas.draw()
            fig.canvas.flush_events()
            last_ui = now

        # allow close
        if not plt.fignum_exists(fig.number):
            break

    ser.close()

if __name__ == "__main__":
    main()
