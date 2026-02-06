# BPU Stream Engine

# BPU Stream Engine

## Demo

![Demo](demo.gif)


High-speed realtime streaming engine demo using ESP32 devices.

This project demonstrates a reliable serial data pipeline:

ESP32-WROOM → ESP32-S3 → PC  
with COBS framing, CRC16 validation, and live visualization.

Designed for stress-testing embedded streaming systems.

---

## Features

- COBS framing (0x00 delimited packets)
- CRC16-CCITT integrity check
- Sequence number validation
- High-rate draw stress generator
- Live visualization on PC (Python + matplotlib)
- Throughput and error statistics

---

## System Architecture

```
ESP32-WROOM (Generator)
        |
     UART @921600
        |
ESP32-S3 (Bridge)
        |
   USB CDC
        |
       PC Viewer
```

---

## Firmware

### Main Files

| File | Description |
|------|-------------|
| bpu_r4_safe.ino | Main firmware entry |
| bpu_r4.cpp | Core stream engine |
| bpu_r4.h | Engine definitions |

Runs on ESP32-WROOM.

---

## Wiring

### UART Connection

| WROOM | S3 |
|-------|----|
| TX | RX |
| RX (optional) | TX |
| GND | GND |

Baudrate: 921600 (default firmware setting)

---

## PC Viewer

### Requirements

Install Python dependencies:

```bash
pip install pyserial matplotlib
```

---

### List Serial Ports

```bash
python -m serial.tools.list_ports
```

Example output:

```
COM11 USB Serial Device
```

---

### Run Viewer

```bash
python bpu_mat_viewer.py COM11
```

(Change COM port as needed)

---

## Viewer Output

The viewer displays:

- Realtime draw points
- Packets per second (PPS)
- Draw points per second (DPS)
- CRC error count
- Sequence gap detection

Stable run example:

```
crc=0  seqGap=0  drop=0
```

---

## Performance

Tested configuration:

- UART: 921600 baud
- Continuous draw stress
- 1000+ packets/sec sustained
- Zero CRC errors in stable mode

---

## Project Status

- Streaming pipeline: Stable
- Error detection: Verified
- PC visualization: Working
- Stress mode: Enabled

This repository is suitable as a reference implementation for embedded serial streaming systems.

---

## Future Work

- Bidirectional control channel
- Binary compression
- Web-based viewer
- Multi-device synchronization
- FPGA offload experiments

---

## Author

Himchan Choi  
Embedded Systems / Streaming Engine Development  
GitHub: https://github.com/choihimchan
