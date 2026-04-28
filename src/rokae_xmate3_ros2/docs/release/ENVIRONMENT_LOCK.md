# Environment Lock

> 状态：Active  
> 受众：发布维护者 / CI 维护者  
> 作用：唯一环境锁定说明  
> 上游事实来源：`tools/check_target_environment.sh`、`tools/write_target_env_report.py`、CI workflow  
> 最后校验：2026-04-18

## Locked target environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11

## Required tools

- `tools/check_target_environment.sh`
- `write_target_env_report.py`
- `tools/run_target_env_acceptance.sh`

目标环境报告输出到：
- `artifacts/target_env_acceptance/`

环境锁与 release gate 必须同步维护；不能只改 workflow 不改本页。
