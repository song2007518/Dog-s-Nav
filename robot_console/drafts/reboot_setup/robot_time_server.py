#!/usr/bin/env python3
"""Restricted NTP reference for this robot; managed by the user systemd unit."""
import json
import socket
import struct
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / 'runtime/time_server.json'
TRUST = ROOT / 'runtime/pc_clock.json'
ALLOWED = {'192.168.234.1', '192.168.234.234'}
HOLDOVER = 86400


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temp.replace(path)


def robot_ip():
    r = subprocess.run(['ip', '-j', '-4', 'route', 'get', '192.168.234.234'],
                       capture_output=True, text=True, timeout=3, check=True)
    row = json.loads(r.stdout)[0]
    host = row.get('prefsrc', row.get('src', ''))
    if not host.startswith('192.168.234.') or host in ALLOWED:
        raise RuntimeError('电脑尚未连接机器人 192.168.234.0/24 网络')
    return host


def clock_quality():
    now = time.time()
    r = subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                       capture_output=True, text=True, timeout=3)
    if r.returncode == 0 and r.stdout.strip() == 'yes' and now > 1735689600:
        save(TRUST, {'last_sync': now})
        return True, .05, '电脑系统已同步'
    try:
        age = now - float(json.loads(TRUST.read_text())['last_sync'])
    except (OSError, ValueError, KeyError):
        age = -1
    if 0 <= age <= HOLDOVER:
        return True, .05 + age * .00003, '离线保持电脑时钟（最近同步在 24 小时内）'
    return False, 16, '电脑需先联网校时；最近可信同步已超过 24 小时或时钟异常'


def stamp(value):
    seconds = value + 2208988800
    whole = int(seconds)
    return struct.pack('!II', whole & 0xffffffff, int((seconds-whole)*2**32))


def reply(data, now, trusted, dispersion, host):
    if len(data) < 48 or data[0] & 7 != 3 or (data[0] >> 3) & 7 not in (3, 4):
        return None
    packet = bytearray(48)
    packet[0] = (0 if trusted else 3) << 6 | 4 << 3 | 4
    packet[1] = 10 if trusted else 16
    packet[2] = 4
    packet[3] = (-20) & 255
    struct.pack_into('!II', packet, 4, int(.02*65536), int(dispersion*65536))
    packet[12:16] = socket.inet_aton(host)
    packet[16:24] = stamp(now)
    packet[24:32] = data[40:48]
    packet[32:40] = stamp(now)
    packet[40:48] = stamp(time.time())
    return bytes(packet)


def main():
    sock = None
    host = None
    trusted, dispersion, reason = False, 16, '检查中'
    check_at = 0
    last_message = None
    count = 0
    while True:
        if time.monotonic() >= check_at:
            check_at = time.monotonic() + 5
            try:
                address = robot_ip()
                trusted, dispersion, reason = clock_quality()
                if address != host or sock is None:
                    if sock:
                        sock.close()
                    sock = None
                    new_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    try:
                        new_socket.bind((address, 1123))
                    except Exception:
                        new_socket.close()
                        raise
                    sock = new_socket
                    sock.settimeout(1)
                    host = address
                phase = 'ready' if trusted else 'clock_untrusted'
            except Exception as exc:
                if sock:
                    sock.close()
                sock, host = None, None
                phase, reason = 'waiting', str(exc)
            info = {'phase': phase, 'ip': host, 'reason': reason,
                    'updated': time.time(), 'replies': count}
            save(STATE, info)
            message = (phase, host, reason)
            if message != last_message:
                print(json.dumps(info, ensure_ascii=False), flush=True)
                last_message = message
        if sock is None:
            time.sleep(1)
            continue
        try:
            data, address = sock.recvfrom(512)
            if address[0] not in ALLOWED:
                continue
            packet = reply(data, time.time(), trusted, dispersion, host)
            if packet:
                sock.sendto(packet, address)
                count += 1
        except socket.timeout:
            pass
        except OSError:
            sock.close()
            sock, host = None, None
            check_at = 0


if __name__ == '__main__':
    main()
