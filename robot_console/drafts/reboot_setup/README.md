# 重启恢复草稿（未部署、未验收）

这三个文件保留此前未完成的重启配置工作，不在 `build.sh`、`start_console.sh` 或原 `scripts/start_all.sh` 的调用链中。本次归档没有执行它们，也没有安装 systemd 服务。

| 文件 | 草稿用途 | 待完成 |
|---|---|---|
| `robot_time_server.py` | 识别电脑机器人网卡地址，根据电脑授时状态提供受限 NTP 响应 | 断网与时钟可信度验证、服务安装和重启试验；文首提及的 systemd unit 尚未提供 |
| `configure_robot_clock.py` | 板卡上备份并修改 chrony 的电脑时间源 | 配置备份/回滚和两板重启验证；执行会写 `/etc/chrony/chrony.conf`，可能校正系统时钟 |
| `prepare_laser.py` | 在 NX 上按感知状态唤醒、激活扫描并检查数据 | 部署环境与服务字段验证、错误状态和重启回归 |

先前成功的是临时运行期授时、一次定位加载和感知激活；这些证据在 `../../validation/`。不能据此声称重启后能自动恢复。实际导航继续使用仓库原 `scripts/start_all.sh` 方案，并检查定位与扫描状态。
