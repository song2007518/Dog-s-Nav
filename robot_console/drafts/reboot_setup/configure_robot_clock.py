#!/usr/bin/env python3
"""Runs on a robot board under sudo; only modifies chrony's PC source."""
import datetime
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def configured_text(original, host):
    lines = []
    for line in original.splitlines():
        tokens = line.split()
        if len(tokens) >= 2 and tokens[0] == 'server' and (
                tokens[1] == '192.168.234.18' or '# robot-console-pc' in line):
            continue
        if line.startswith('# Dog-s-Nav temporary time-sync test;'):
            continue
        lines.append(line)
    # Runtime source changes can be applied without restarting the algorithms.
    source = f'server {host} port 1123 iburst minpoll 2 maxpoll 4 # robot-console-pc'
    lines.append(source)
    return '\n'.join(lines) + '\n'


def run(args, check=True):
    return subprocess.run(args, text=True, capture_output=True, timeout=10, check=check)


def main():
    host = str(ipaddress.ip_address(sys.argv[1]))
    assert host.startswith('192.168.234.') and host not in ('192.168.234.1', '192.168.234.234')
    target = Path('/etc/chrony/chrony.conf')
    original = target.read_text()
    content = configured_text(original, host)
    backup = None
    if content != original:
        backup = str(target) + '.robot-console-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.bak'
        run(['cp', '-a', str(target), backup])
        stat = target.stat()
        descriptor, temporary = tempfile.mkstemp(prefix='.robot-console-', dir=str(target.parent))
        try:
            with os.fdopen(descriptor, 'w') as f:
                f.write(content)
                f.flush()
                os.fsync(f.fileno())
            os.chmod(temporary, stat.st_mode & 0o777)
            os.chown(temporary, stat.st_uid, stat.st_gid)
            run(['chronyd', '-p', '-f', temporary])
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    # Vendor boot scripts start chrony. Do not change their service ordering.
    run(['systemctl', 'start', 'chrony'])
    sources = run(['chronyc', '-n', 'sources']).stdout
    if not any(line.split()[1:2] == [host] for line in sources.splitlines()):
        run(['chronyc', 'add', 'server', host, 'port', '1123', 'iburst', 'minpoll', '2', 'maxpoll', '4'])
    # Remove only former console-owned sources, leaving vendor sources alone.
    previous = {'192.168.234.18'}
    for line in original.splitlines():
        if '# robot-console-pc' in line and line.startswith('server '):
            previous.add(line.split()[1])
    for old in previous - {host}:
        if any(line.split()[1:2] == [old] for line in sources.splitlines()):
            run(['chronyc', 'delete', old])
    tracking = run(['chronyc', '-n', 'tracking']).stdout
    if 'Leap status     : Normal' not in tracking:
        run(['chronyc', 'makestep', '0.1', '3'])
    run(['chronyc', 'burst', '4/4'])
    print(json.dumps({'changed': content != original, 'backup': backup, 'source': host,
                      'configuration': content, 'tracking': tracking}, ensure_ascii=False))


if __name__ == '__main__':
    main()
