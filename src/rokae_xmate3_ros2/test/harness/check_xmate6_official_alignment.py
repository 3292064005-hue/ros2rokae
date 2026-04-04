#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
failures: list[str] = []

manifest_path = ROOT / "docs" / "xmate6_official_alignment_manifest.json"
if not manifest_path.is_file():
    failures.append(f"missing alignment manifest: {manifest_path}")
else:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("target_family") != "xmate6":
        failures.append("manifest target_family must be xmate6")
    if manifest.get("alignment_scope") != "call_build_only":
        failures.append("manifest alignment_scope must be call_build_only")
    unsupported = manifest.get("unsupported_public_modules", [])
    if unsupported != ["io", "rl", "calibration"]:
        failures.append("manifest unsupported_public_modules must be exactly ['io', 'rl', 'calibration']")

robot_header = (ROOT / "include" / "rokae" / "robot.h").read_text(encoding="utf-8")
if "Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept;" not in robot_header:
    failures.append("robot.h must expose Toolset-returning setToolset(toolName, wobjName, ec)")
if "std::array<double, 6> flangePos(error_code &ec) const noexcept;" not in robot_header:
    failures.append("robot.h must keep flangePos(ec) compatibility alias")
if "[[deprecated(\"Use jointTorque() instead\")]]" not in robot_header:
    failures.append("robot.h must mark jointTorques as deprecated alias")

compat_api = (ROOT / "src" / "compat" / "robot_api.cpp").read_text(encoding="utf-8")
if "throw_if_error<ExecutionException>(ec, \"connectToRobot\");" not in compat_api:
    failures.append("no-error-code connect overload must throw ExecutionException")
if "Robot_T<WorkType::collaborative, 6>::Robot_T(const std::string &remoteIP, const std::string &localIP)" not in compat_api:
    failures.append("Robot_T<collaborative,6> remote constructor definition is missing")
if "throw_if_error<ExecutionException>(ec, \"Robot_T::Robot_T(connectToRobot)\");" not in compat_api:
    failures.append("Robot_T remote constructor must auto-connect and throw on failure")
if "if (remoteIP.empty()) {" not in compat_api:
    failures.append("connectToRobot(remoteIP, localIP, ec) must reject empty remoteIP")
if "if (!has_remote_endpoint(handle_)) {" not in compat_api:
    failures.append("connectToRobot(ec) must reject missing configured remote endpoint")

compat_shared_hpp = (ROOT / "src" / "compat" / "internal" / "compat_shared.hpp").read_text(encoding="utf-8")
if not re.search(r"std::string\s+remote_ip\s*;", compat_shared_hpp):
    failures.append("CompatRobotHandle::remote_ip must not have an implicit default endpoint")

cmake_text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
if 'set(ROKAE_PUBLIC_UNSUPPORTED_MODULES "io;rl;calibration"' not in cmake_text:
    failures.append("CMakeLists.txt must keep public unsupported modules fixed to io;rl;calibration")

if failures:
    print("xmate6 official alignment check failed:")
    for item in failures:
        print(f"- {item}")
    sys.exit(1)

print("xmate6 official alignment check passed")
