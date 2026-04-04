# xMate6 Official SDK Alignment Matrix (Call/Build Layer)

This matrix freezes the install-facing xMate6 compatibility lane against the official SDK call shape.
It is kept in sync with the single-source manifest:
`docs/xmate6_official_alignment_manifest.json`.

## Baseline

- Header baseline: `librokae/include/rokae/*.h`
- Example baseline: `librokae/example/sdk_example.cpp`, `move_example.cpp`, `read_robot_state.cpp`, `path_record.cpp`
- Excluded modules (public lane): `io`, `rl`, `calibration`

## Contract Decisions

| Area | Official-style target | xMate6 public lane |
|---|---|---|
| Construction and connect | `xMateRobot(remoteIP, localIP)` and no-`ec` connect throw on failure | Aligned |
| Default endpoint | No implicit endpoint fallback | Aligned |
| Toolset by name | `setToolset(toolName, wobjName, ec)` returns `Toolset` | Aligned |
| Joint torque accessor | `jointTorque(ec)` primary entry | Aligned (`jointTorques` kept as deprecated alias) |
| Flange pose alias | `flangePos(ec)` compatibility alias | Aligned (deprecated) |
| NRT move API shape | `moveAppend/executeCommand/moveStart/stop` | Aligned |
| RT API shape | `getRtMotionController()` and RT command/config surface | Aligned (simulation-grade backend) |
| Runtime profile naming | explicit NRT/RT profile tags | `nrt_strict_parity`, `hard_1khz` |
| IO/register | symbols kept for source compatibility | Unsupported (`not_implemented`) |
| RL project | symbols kept for source compatibility | Unsupported (`not_implemented`) |
| Calibration | symbol kept for source compatibility | Unsupported (`not_implemented`) |

## Machine-checked Source of Truth

See `docs/xmate6_official_alignment_manifest.json` and
`test/harness/check_xmate6_official_alignment.py`.
