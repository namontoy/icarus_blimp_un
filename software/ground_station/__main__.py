"""`python -m software.ground_station [options]` - see server.main() for the options."""
import os
import sys

# blimp_link.py and mock_blimp.py live one level up, in software/, and are
# imported as top-level modules exactly as their own CLIs run them.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from .server import main  # noqa: E402

main()
