import atexit
import os
import shutil
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# The viewer remembers the last port, baud and panel choices in
# QSettings("icarus_blimp", "tcview"). Tests get a store of their own, so they
# neither read the user's choices nor leave a fake camera's pty behind in them.
from PyQt6.QtCore import QSettings  # noqa: E402

_settings_dir = tempfile.mkdtemp(prefix="tcview-test-settings-")
atexit.register(shutil.rmtree, _settings_dir, True)
# QSettings(org, app) always uses NativeFormat, an .ini file on Linux.
QSettings.setPath(QSettings.Format.NativeFormat, QSettings.Scope.UserScope, _settings_dir)
