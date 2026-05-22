#!/usr/bin/env python3
"""
BT-DISK-BRIDGE host for ATARI_XF551_RP BT_GATEWAY F47 STABLE FLOW.

Uso Windows:
  pip install pyserial
  python tools/bt_disk_host.py --port COM7 --atr C:\\Atari\\disk.atr --dev D1

Uso Linux/macOS:
  pip install pyserial
  python3 tools/bt_disk_host.py --port /dev/rfcomm0 --atr ./disk.atr --dev D1

El MASTER espera frames BridgeProtocol sobre el puerto serie Bluetooth externo
(HC-05/HC-06 conectado al ESP32 MASTER por UART TTL).
"""
import argparse
import os
import struct
import sys
import time
from typing import Optional

try:
    import serial  # type: ignore
except ImportError:
    print("Falta pyserial. Instala con: pip install pyserial", file=sys.stderr)
    raise

UART_SYNC = 0x55
TYPE_CMD_FRAME = 0x01
TYPE_SECTOR_CHUNK = 0x10
TYPE_ACK = 0x11
TYPE_NAK = 0x12
TYPE_HELLO = 0x20
CHUNK_PAYLOAD = 240
PERCOM_SEC_MAGIC = 0xFFFF

DEV_CODES = {"D1": 0x31, "D2": 0x32, "D3": 0x33, "D4": 0x34}


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def make_frame(payload: bytes) -> bytes:
    if not 1 <= len(payload) <= 255:
        raise ValueError("payload len invalido")
    return bytes([UART_SYNC, len(payload)]) + payload + bytes([checksum(payload)])


def send_payload(ser: serial.Serial, payload: bytes, flush: bool = False) -> None:
    ser.write(make_frame(payload))
    # F47: no hacer flush por cada frame. En Bluetooth serial esto bloquea
    # y baja mucho la velocidad. Se permite flush solo para HELLO/config.
    if flush:
        ser.flush()


def read_payload(ser: serial.Serial, timeout: float = 0.02) -> Optional[bytes]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != UART_SYNC:
            continue
        lb = ser.read(1)
        if not lb:
            return None
        ln = lb[0]
        if ln == 0:
            return None
        data = ser.read(ln)
        if len(data) != ln:
            return None
        chk = ser.read(1)
        if len(chk) != 1:
            return None
        if checksum(data) != chk[0]:
            print(f"[BT-HOST] checksum invalido len={ln}")
            return None
        return data
    return None


class AtrImage:
    def __init__(self, path: str):
        self.path = path
        with open(path, "rb") as f:
            self.data = f.read()
        if len(self.data) < 16 or self.data[0:2] != b"\x96\x02":
            raise ValueError("No parece ATR valido: falta magic 96 02")
        self.header = self.data[:16]
        self.sec_size = struct.unpack_from("<H", self.header, 4)[0]
        if self.sec_size not in (128, 256):
            print(f"[BT-HOST] Aviso: sector size ATR no comun: {self.sec_size}; se intentara igual")
        self.total_sectors = self._estimate_total_sectors()

    def _estimate_total_sectors(self) -> int:
        body = len(self.data) - 16
        if self.sec_size <= 128:
            return body // 128
        if body <= 384:
            return body // 128
        return 3 + ((body - 384) // self.sec_size)

    def sector_size_for(self, sec: int) -> int:
        if self.sec_size > 128 and sec <= 3:
            return 128
        return self.sec_size

    def sector_offset(self, sec: int) -> int:
        if sec < 1:
            raise ValueError("sector debe partir en 1")
        if self.sec_size <= 128:
            return 16 + (sec - 1) * 128
        if sec <= 3:
            return 16 + (sec - 1) * 128
        return 16 + 3 * 128 + (sec - 4) * self.sec_size

    def read_sector(self, sec: int) -> bytes:
        size = self.sector_size_for(sec)
        off = self.sector_offset(sec)
        if off < 16 or off + size > len(self.data):
            raise IndexError(f"sector fuera del ATR: {sec}")
        return self.data[off:off + size]

    def percom(self) -> bytes:
        # PERCOM 12 bytes usado por XF551/1050-like. Valores 16-bit en big-endian.
        total = self.total_sectors
        bps = self.sec_size if self.sec_size in (128, 256) else 128
        tracks = 40
        spt = 18
        sides = 1

        if bps == 128 and total in (1040, 1041):
            tracks, spt, sides = 40, 26, 1
        elif bps == 128 and total >= 2880:
            tracks, spt, sides = 80, 18, 2
        elif bps == 128 and total >= 1440:
            tracks, spt, sides = 40, 18, 2
        elif bps == 256 and total >= 1440:
            tracks, spt, sides = 80, 18, 1
        elif bps == 256:
            tracks, spt, sides = 40, 18, 1

        p = bytearray(12)
        p[0] = tracks & 0xFF
        p[1] = 0x04
        p[2] = (spt >> 8) & 0xFF
        p[3] = spt & 0xFF
        p[4] = 0x01 if sides == 2 else 0x00
        p[5] = 0x04 if bps == 256 else 0x00
        p[6] = (bps >> 8) & 0xFF
        p[7] = bps & 0xFF
        return bytes(p)


def send_ack(ser: serial.Serial, dev: int) -> None:
    send_payload(ser, bytes([TYPE_ACK, dev]))


def send_nak(ser: serial.Serial, dev: int) -> None:
    send_payload(ser, bytes([TYPE_NAK, dev]))


def send_chunks(ser: serial.Serial, dev: int, sec: int, data: bytes) -> None:
    count = max(1, (len(data) + CHUNK_PAYLOAD - 1) // CHUNK_PAYLOAD)
    for idx in range(count):
        part = data[idx * CHUNK_PAYLOAD:(idx + 1) * CHUNK_PAYLOAD]
        payload = bytes([
            TYPE_SECTOR_CHUNK,
            dev,
            sec & 0xFF,
            (sec >> 8) & 0xFF,
            idx & 0xFF,
            count & 0xFF,
        ]) + part
        send_payload(ser, payload)


def send_hello(ser: serial.Serial, dev: int, supports256: bool) -> None:
    send_payload(ser, bytes([TYPE_HELLO, dev, 1 if supports256 else 0, dev]), flush=True)


def status_bytes(img: AtrImage) -> bytes:
    # Estado logico simple: OK. Mantener 4 bytes como dispositivo D: Atari.
    # Para DD dejamos byte 2 distinto para ayudar a diagnostico, pero el RP usa principalmente PERCOM.
    if img.sec_size == 256:
        return bytes([0x10, 0xFF, 0xFE, 0x00])
    return bytes([0x10, 0xFF, 0xE0, 0x00])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="COMx, /dev/rfcomm0, /dev/tty.*")
    ap.add_argument("--atr", required=True, help="archivo ATR")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dev", default="D1", choices=sorted(DEV_CODES.keys()))
    ap.add_argument("--hello-interval", type=float, default=5.0, help="segundos entre keepalive HELLO cuando el enlace esta idle")
    ap.add_argument("--hello-idle-gap", type=float, default=0.8, help="no enviar HELLO si hubo trafico reciente")
    ap.add_argument("--verbose", action="store_true", help="imprime cada comando/sector; por defecto solo resumen")
    ap.add_argument("--summary-interval", type=float, default=2.0, help="segundos entre resumenes de actividad")
    args = ap.parse_args()

    img = AtrImage(args.atr)
    dev = DEV_CODES[args.dev]
    supports256 = img.sec_size == 256

    print(f"[BT-HOST] ATR={args.atr}")
    print(f"[BT-HOST] dev={args.dev} secSize={img.sec_size} totalSectors~{img.total_sectors} baud={args.baud}")

    with serial.Serial(args.port, args.baud, timeout=0.005, write_timeout=1) as ser:
        time.sleep(0.5)
        last_hello = 0.0
        last_activity = time.monotonic()
        last_summary = time.monotonic()
        read_count = 0
        status_count = 0
        percom_count = 0
        nak_count = 0
        bytes_sent = 0
        send_hello(ser, dev, supports256)
        last_hello = time.monotonic()
        print("[BT-HOST] HELLO inicial enviado; esperando comandos...")
        while True:
            p = read_payload(ser, timeout=0.015)
            now = time.monotonic()

            if args.summary_interval > 0 and (now - last_summary) >= args.summary_interval:
                elapsed = max(0.001, now - last_summary)
                print(f"[BT-HOST] resumen read={read_count} status={status_count} percom={percom_count} nak={nak_count} tx={bytes_sent}B")
                last_summary = now

            if not p:
                # FIX1: enviar HELLO solo como keepalive en reposo. No lo intercalamos
                # entre comandos READ y respuestas, porque eso puede contaminar el flujo
                # MASTER -> RP2040 durante cargas rápidas.
                if (now - last_hello >= args.hello_interval) and (now - last_activity >= args.hello_idle_gap):
                    send_hello(ser, dev, supports256)
                    last_hello = now
                    last_activity = now
                continue

            last_activity = now
            t = p[0]
            if t != TYPE_CMD_FRAME or len(p) < 6:
                continue

            cmd = p[1]
            rdev = p[2]
            aux1 = p[3]
            aux2 = p[4]
            base = cmd & 0x7F
            sec = aux1 | (aux2 << 8)

            if rdev != dev:
                nak_count += 1
                if args.verbose:
                    print(f"[BT-HOST] CMD para otro dev 0x{rdev:02X}; NAK")
                send_nak(ser, rdev)
                continue

            try:
                if base == 0x52:  # READ
                    data = img.read_sector(sec)
                    send_ack(ser, dev)
                    send_chunks(ser, dev, sec, data)
                    bytes_sent += len(data)
                    read_count += 1
                    last_activity = time.monotonic()
                    if args.verbose:
                        print(f"[BT-HOST] READ {args.dev} sec={sec} len={len(data)}")

                elif base == 0x53:  # STATUS
                    st = status_bytes(img)
                    send_ack(ser, dev)
                    send_chunks(ser, dev, 0, st)
                    bytes_sent += len(st)
                    status_count += 1
                    last_activity = time.monotonic()
                    if args.verbose:
                        print(f"[BT-HOST] STATUS {args.dev}")

                elif base == 0x4E:  # READ PERCOM
                    pc = img.percom()
                    send_ack(ser, dev)
                    send_chunks(ser, dev, PERCOM_SEC_MAGIC, pc)
                    bytes_sent += len(pc)
                    percom_count += 1
                    last_activity = time.monotonic()
                    if args.verbose:
                        print(f"[BT-HOST] READ PERCOM {args.dev}")

                else:
                    # Primera etapa: solo lectura. WRITE/FORMAT quedan protegidos.
                    nak_count += 1
                    if args.verbose:
                        print(f"[BT-HOST] CMD 0x{cmd:02X} no soportado en F47; NAK")
                    send_nak(ser, dev)

            except Exception as ex:
                nak_count += 1
                print(f"[BT-HOST] ERROR cmd=0x{cmd:02X} sec={sec}: {ex}")
                send_nak(ser, dev)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
