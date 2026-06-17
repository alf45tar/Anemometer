Import("env")

import importlib.util
import subprocess
import sys


def ensure_intelhex_installed() -> None:
    if importlib.util.find_spec("intelhex") is not None:
        return

    subprocess.check_call([sys.executable, "-m", "pip", "install", "intelhex"])


ensure_intelhex_installed()