#!/usr/bin/env bash
root="$(cd "$(dirname "$0")/.." && pwd)"
python3 "$root/tools/deploy_robot.py"
status=$?
echo
if [[ "$status" = 0 ]]; then echo '部署完成，没有发车。请按开测说明继续。'; else echo '部署未完成。请保留上方报错，不要开始运动测试。'; fi
read -r -p '按回车关闭窗口。' _
exit "$status"
