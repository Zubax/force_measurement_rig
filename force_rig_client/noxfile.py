# Copyright (C) 2023 Zubax Robotics
# type: ignore

import shutil
from pathlib import Path
import nox
import os


PYTHONS = ["3.12"]

nox.options.error_on_external_run = True

PROJECT_ROOT = Path(__file__).parent.resolve()
LIB_DIR = PROJECT_ROOT / "lib"
DSDL_DIR = PROJECT_ROOT / "dsdl_types"


@nox.session(python=False)
def clean(session):
    for w in [
        "*.egg-info",
        ".coverage*",
        "html*",
        ".*cache",
        ".*compiled",
        "*.log",
        "*.tmp",
    ]:
        for f in Path.cwd().glob(w):
            session.log(f"Removing: {f}")
            if f.is_dir():
                shutil.rmtree(f, ignore_errors=True)
            else:
                f.unlink(missing_ok=True)


@nox.session(python=PYTHONS)
def test(session):
    session.install("-e", ".")
    session.install("-r", "requirements.txt")

    pytest_env = {
        "CYPHAL_PATH": os.pathsep.join(  # DSDL namespace directories.
            map(
                str,
                [  # Notice that we're not including the existing path here but overriding it entirely.
                    LIB_DIR / "public_regulated_data_types",
                    LIB_DIR / "zubax_dsdl",
                ],
            )
        ),
        "PYCYPHAL_PATH": DSDL_DIR,  # Use temp dir to not interfere with the system installation.
        # "PYTHONPATH": os.pathsep.join(
        #     [
        #         os.environ.get("PYTHONPATH", ""),
        #         str(EPM_PROJECT_ROOT) + "/scripts",
        #         ]
        # ),
        # "EPM_PROJECT_ROOT": str(EPM_PROJECT_ROOT),
    }
    session.log(f"Environment: {pytest_env}")

    session.install(
        "pytest     ~= 7.3",
        "coverage   ~= 7.2",
        "mypy       ~= 1.2",
        "pylint     ~= 2.17",
    )

    # PyTest
    session.run("coverage", "run", "-m", "pytest", env=pytest_env)

    # Coverage report
    session.run("coverage", "report", "--fail-under=25")
    if session.interactive:
        session.run("coverage", "html")
        report_file = Path.cwd().resolve() / "htmlcov" / "index.html"
        session.log(f"OPEN IN WEB BROWSER: file://{report_file}")

    # Static analysis
    session.run("mypy", "--strict", ".")
    # session.run("pylint", "src")


@nox.session(reuse_venv=True)
def black(session):
    session.install("black ~= 23.3")
    session.run("black", "--check", ".")
