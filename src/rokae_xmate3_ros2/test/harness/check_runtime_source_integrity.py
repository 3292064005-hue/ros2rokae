#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []


def compile_alias_policy_header() -> None:
    header = ROOT / "src" / "runtime" / "compatibility_alias_policy.hpp"
    if not header.is_file():
        FAILURES.append(f"missing runtime compatibility alias policy header: {header}")
        return
    text = header.read_text(encoding='utf-8')
    if 'inline CompatibilityAliasPolicy defaultCompatibilityAliasPolicy()' not in text:
        FAILURES.append('compatibility_alias_policy.hpp must declare defaultCompatibilityAliasPolicy()')
    marker = 'inline CompatibilityAliasPolicy defaultCompatibilityAliasPolicy()'
    marker_pos = text.find(marker)
    if marker_pos >= 0:
        close_pos = text.find('\n}\n', marker_pos)
        body = text[marker_pos:close_pos + 3 if close_pos >= 0 else len(text)]
        if 'policy_name' in body:
            FAILURES.append('defaultCompatibilityAliasPolicy() must not reference parse-time policy_name state')
    import os
    if os.environ.get('ROKAE_STRICT_COMPILER_SYNTAX_CHECK') != '1':
        return
    compiler = None
    for candidate in ("g++", "clang++", "c++"):
        try:
            subprocess.run([candidate, "--version"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
            compiler = candidate
            break
        except Exception:
            continue
    if compiler is None:
        FAILURES.append("no C++ compiler available to syntax-check compatibility_alias_policy.hpp")
        return
    with tempfile.TemporaryDirectory() as td:
        source = Path(td) / "check_alias_policy.cpp"
        source.write_text(f'#include "{header}"\nint main() {{ return 0; }}\n', encoding='utf-8')
        try:
            result = subprocess.run([compiler, "-std=c++17", "-fsyntax-only", str(source)], capture_output=True, text=True, timeout=20)
        except subprocess.TimeoutExpired:
            FAILURES.append("compatibility_alias_policy.hpp syntax-check timed out")
            return
        if result.returncode != 0:
            FAILURES.append("compatibility_alias_policy.hpp failed standalone syntax-check:\n" + result.stderr.strip())

def ensure_manifest_has_balanced_preprocessor() -> None:
    manifest = ROOT / "src" / "runtime" / "service_contract_manifest.cpp"
    text = manifest.read_text(encoding='utf-8')
    stack: list[tuple[str, int]] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        stripped = raw.strip()
        if stripped.startswith("#if"):
            stack.append((stripped.split()[0], lineno))
        elif stripped == "#endif":
            if not stack:
                FAILURES.append(f"service_contract_manifest.cpp has extra #endif at line {lineno}")
                return
            stack.pop()
    if stack:
        opener, lineno = stack[-1]
        FAILURES.append(f"service_contract_manifest.cpp has unclosed preprocessor block from line {lineno}: {opener}")




def ensure_public_model_facade_uses_interface_only() -> None:
    facade = ROOT / "include" / "rokae_xmate3_ros2" / "model_facade.hpp"
    if not facade.is_file():
        FAILURES.append(f"missing public model facade header: {facade}")
        return
    text = facade.read_text(encoding="utf-8")
    for token in [
        "rokae_xmate3_ros2/runtime/kinematics_provider.hpp",
        "rokae_xmate3_ros2/gazebo/",
        "OwnedGazeboProvider",
        "GazeboProvider",
        "gazebo::",
        "gazebo_model::",
    ]:
        if token in text:
            FAILURES.append(f"model_facade.hpp must depend only on the provider interface, not concrete simulation details: {token}")
    if "rokae_xmate3_ros2/runtime/kinematics_provider_interface.hpp" not in text:
        FAILURES.append("model_facade.hpp must include the backend-neutral kinematics_provider_interface.hpp")


def ensure_gazebo_model_facade_uses_resolvable_provider_boundary() -> None:
    gazebo_facade = ROOT / "include" / "rokae_xmate3_ros2" / "gazebo" / "model_facade.hpp"
    if not gazebo_facade.is_file():
        FAILURES.append(f"missing Gazebo model facade header: {gazebo_facade}")
        return
    text = gazebo_facade.read_text(encoding="utf-8")
    removed_public_provider = "rokae_xmate3_ros2/runtime/kinematics_provider.hpp"
    if removed_public_provider in text:
        FAILURES.append(
            "gazebo/model_facade.hpp must not include the removed public concrete provider path: "
            f"{removed_public_provider}"
        )
    if "rokae_xmate3_ros2/runtime/kinematics_provider_interface.hpp" not in text:
        FAILURES.append("gazebo/model_facade.hpp must depend on the backend-neutral provider interface")


def ensure_no_removed_concrete_provider_include_remains() -> None:
    removed_public_provider = 'rokae_xmate3_ros2/runtime/kinematics_provider.hpp'
    allowed_text_files = {
        Path('test/harness/check_runtime_source_integrity.py'),
        Path('test/harness/check_compat_public_abi.py'),
        Path('docs/public/KINEMATICS_AND_MODEL.md'),
        Path('docs/public/COMPATIBILITY.md'),
        Path('docs/public/QUICKSTART.md'),
        Path('docs/release/BUILD_RELEASE.md'),
        Path('README.md'),
    }
    for root_name in ['include', 'src', 'test']:
        root = ROOT / root_name
        if not root.exists():
            continue
        for candidate in root.rglob('*'):
            if not candidate.is_file():
                continue
            rel = candidate.relative_to(ROOT)
            if rel in allowed_text_files:
                continue
            if candidate.suffix not in {'.h', '.hpp', '.hh', '.cpp', '.cc', '.cxx'}:
                continue
            try:
                text = candidate.read_text(encoding='utf-8')
            except UnicodeDecodeError:
                continue
            if removed_public_provider in text:
                FAILURES.append(f"removed public concrete provider include remains in {rel}")


def ensure_provider_interface_is_backend_neutral() -> None:
    interface = ROOT / "include" / "rokae_xmate3_ros2" / "runtime" / "kinematics_provider_interface.hpp"
    if not interface.is_file():
        FAILURES.append(f"missing backend-neutral provider interface header: {interface}")
        return
    text = interface.read_text(encoding="utf-8")
    for token in ["rokae_xmate3_ros2/gazebo/", "OwnedGazeboProvider", "GazeboProvider", "gazebo::", "gazebo_model::"]:
        if token in text:
            FAILURES.append(f"kinematics_provider_interface.hpp must not expose concrete simulation provider detail: {token}")
    if "class Provider" not in text:
        FAILURES.append("kinematics_provider_interface.hpp must declare the abstract Provider contract")


def ensure_concrete_provider_is_separate_from_interface() -> None:
    public_concrete = ROOT / "include" / "rokae_xmate3_ros2" / "runtime" / "kinematics_provider.hpp"
    if public_concrete.exists():
        FAILURES.append(
            "concrete simulation provider must not live under public include/: "
            f"{public_concrete.relative_to(ROOT)}"
        )
    private_concrete = ROOT / "src" / "runtime" / "kinematics_provider.hpp"
    if not private_concrete.is_file():
        FAILURES.append(f"missing private concrete provider header: {private_concrete}")
        return
    text = private_concrete.read_text(encoding="utf-8")
    if "kinematics_provider_interface.hpp" not in text:
        FAILURES.append("private concrete provider must build on the backend-neutral provider interface")
    if "rokae_xmate3_ros2/gazebo/kinematics.hpp" not in text:
        FAILURES.append("private concrete provider must be the explicit simulation-provider implementation boundary")

def ensure_quickstart_consumer_language_is_consistent() -> None:
    quickstart = (ROOT / "docs" / "public" / "QUICKSTART.md").read_text(encoding='utf-8')
    bad = "install-facing 主消费者 `xCoreSDK::xCoreSDK_shared`"
    if bad in quickstart:
        FAILURES.append(
            "docs/public/QUICKSTART.md still describes xCoreSDK::xCoreSDK_shared as the install-facing primary consumer"
        )


def ensure_public_model_header_has_no_concrete_sim_provider() -> None:
    model_header = ROOT / "include" / "rokae_xmate3_ros2" / "model.hpp"
    if not model_header.is_file():
        FAILURES.append(f"missing public model header: {model_header}")
        return
    text = model_header.read_text(encoding="utf-8")
    forbidden = [
        "OwnedGazeboProvider",
        "GazeboProvider",
        "rokae_xmate3_ros2/runtime/kinematics_provider.hpp",
        "rokae_xmate3_ros2/gazebo/",
        "gazebo::",
        "gazebo_model::",
    ]
    for token in forbidden:
        if token in text:
            FAILURES.append(f"model.hpp must not expose concrete simulation provider detail: {token}")
    required = ["struct Impl", "std::unique_ptr<Impl>", "~XMateModel()"]
    for token in required:
        if token not in text:
            FAILURES.append(f"model.hpp must keep concrete provider ownership behind an opaque implementation: {token}")


def ensure_public_model_impl_owns_sim_provider_privately() -> None:
    impl = ROOT / "src" / "sdk" / "xmate_model.cpp"
    if not impl.is_file():
        FAILURES.append("missing compiled public model implementation source: src/sdk/xmate_model.cpp")
        return
    text = impl.read_text(encoding="utf-8")
    if "OwnedGazeboProvider" not in text or "makeModelFacade" not in text:
        FAILURES.append("xmate_model.cpp must own the simulation provider privately and feed the public provider facade")


compile_alias_policy_header()
ensure_manifest_has_balanced_preprocessor()
ensure_public_model_facade_uses_interface_only()
ensure_gazebo_model_facade_uses_resolvable_provider_boundary()
ensure_no_removed_concrete_provider_include_remains()
ensure_provider_interface_is_backend_neutral()
ensure_concrete_provider_is_separate_from_interface()
model_facade = (ROOT / "include" / "rokae_xmate3_ros2" / "model_facade.hpp").read_text(encoding="utf-8")
for forbidden in [
    "using ModelFacade = gazebo_model::ModelFacade",
    "rokae_xmate3_ros2/gazebo/model_facade.hpp",
    "rokae_xmate3_ros2/runtime/kinematics_provider.hpp",
    "gazebo_model::",
    "OwnedGazeboProvider",
    "GazeboProvider",
    "gazebo::",
    "delegate_",
]:
    if forbidden in model_facade:
        FAILURES.append(f"model_facade.hpp must not expose concrete simulation provider detail: {forbidden}")
if "kinematics::Provider" not in model_facade:
    FAILURES.append("model_facade.hpp must consume the runtime kinematics::Provider boundary directly")
if not (ROOT / "tools" / "run_full_source_tree_build_gate.sh").is_file():
    FAILURES.append("missing non-replay full source-tree build gate script")

ensure_quickstart_consumer_language_is_consistent()
ensure_public_model_header_has_no_concrete_sim_provider()
ensure_public_model_impl_owns_sim_provider_privately()

if FAILURES:
    print("runtime source integrity check failed:")
    for item in FAILURES:
        print(f"- {item}")
    sys.exit(1)

print("runtime source integrity check passed")
