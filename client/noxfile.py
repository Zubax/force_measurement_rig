# Copyright (C) 2023 Zubax Robotics
# type: ignore

import shutil
from pathlib import Path
import nox


PYTHONS = ["3.10", "3.11"]

nox.options.error_on_external_run = True


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
    session.install(
        "pytest     ~= 7.3",
        "coverage   ~= 7.2",
        "mypy       ~= 1.2",
        "pylint     ~= 2.17",
    )

    # PyTest
    session.run("coverage", "run", "-m", "pytest")

    # Coverage report
    session.run("coverage", "report", "--fail-under=25")
    if session.interactive:
        session.run("coverage", "html")
        report_file = Path.cwd().resolve() / "htmlcov" / "index.html"
        session.log(f"OPEN IN WEB BROWSER: file://{report_file}")

    # Static analysis
    session.run("mypy", "--strict", ".")
    session.run("pylint", "src")


@nox.session(reuse_venv=True)
def black(session):
    session.install("black ~= 23.3")
    session.run("black", "--check", ".")
