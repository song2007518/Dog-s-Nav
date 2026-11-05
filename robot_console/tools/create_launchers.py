#!/usr/bin/env python3
"""Generate local desktop entries; no installation or robot connection."""
from pathlib import Path


def desktop_string(value):
    return str(value).replace('\\', '\\\\').replace('\n', '\\n').replace('\r', '\\r').replace('\t', '\\t')


def exec_argument(value):
    # Exec is parsed by the desktop launcher, not by a shell. Escape both the
    # quoted argument layer and the Desktop Entry string layer.
    escaped = ''.join('\\' + c if c in '\\"`$' else c for c in str(value))
    return '"' + desktop_string(escaped).replace('%', '%%') + '"'


def main():
    root = Path(__file__).resolve().parents[1]
    entries = [
        ('启动控制台', '智航机器狗控制台', '离线演示；rqt 开发中', ''),
        ('启动实机控制台', '智航控制台（实机只读）', '自动读取设备状态，真实操作默认关闭', '--robot'),
        ('启动系统控制台', '智航控制台（系统启动管理）', '开发中；打开窗口不自动运行 start_all', '--stack'),
        ('拍照展示', '智航控制台（拍照展示）', '离线展示界面和模拟路径', '--presentation'),
    ]
    for filename, name, comment, argument in entries:
        target = root / (filename + '.desktop')
        command = exec_argument(root / 'start_console.sh')
        if argument:
            command += ' ' + argument
        target.write_text('\n'.join([
            '[Desktop Entry]', 'Type=Application', 'Name=' + name,
            'Comment=' + comment, 'Exec=' + command, 'Path=' + desktop_string(root),
            'Icon=applications-engineering', 'Terminal=false',
            'Categories=Development;Robotics;', '',
        ]), encoding='utf-8')
        target.chmod(0o755)
    print('已生成 4 个本地快捷方式：' + str(root))


if __name__ == '__main__':
    main()
