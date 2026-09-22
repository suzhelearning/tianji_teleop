"""Build the retained ROS-free retargeting tools, not the Manus ROS pipeline."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile


def main() -> None:
    if os.environ.get("PIXI_ENVIRONMENT_NAME") != "manus":
        raise SystemExit("build_runtime.py must run in pixi's manus environment")
    root = Path(sys.argv[1]).resolve()
    build = root / "build" / "manus"
    build.mkdir(parents=True, exist_ok=True)
    packages = (
        ("tianji_tools", root / "src/tools/tianji_tools", False),
        ("wuji_retargeting", root / "src/teleop_outputs/wuji/wuji_retargeting", True),
    )
    with tempfile.TemporaryDirectory(prefix="wheels-", dir=build) as temporary:
        for name, source, native in packages:
            command = [sys.executable, "-I", "setup.py", "build",
                       "--build-base", str(build / name)]
            if native:
                command.append("--force")
            command.extend(["bdist_wheel", "--dist-dir", temporary,
                            "--bdist-dir", str(build / name / "wheel")])
            subprocess.run(command, cwd=source, check=True)
        wheels = sorted(Path(temporary).glob("*.whl"))
        subprocess.run([sys.executable, "-I", "-m", "pip", "install", "--no-deps",
                        "--force-reinstall", *(str(wheel) for wheel in wheels)], check=True)


if __name__ == "__main__":
    main()
