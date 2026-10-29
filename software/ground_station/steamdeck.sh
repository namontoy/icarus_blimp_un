#!/bin/bash
# The ground station on a Steam Deck, as the target of Steam's "Add a Non-Steam Game".
#
# In Desktop mode Steam Input applies its Desktop Layout to the Deck's controls, which makes
# the right stick a mouse, so the window never sees it (found 2026-10-29). An app launched
# through Steam gets its own layout instead: pick the "Gamepad" template for this entry and both
# sticks, A, B and the D-pad reach the window as the virtual X-Box pad that gamepad.py reads.
# Runs from Konsole too.
#
#   ./software/ground_station/steamdeck.sh              # the blimp (join its Wi-Fi from the header)
#   ./software/ground_station/steamdeck.sh --demo       # the demo blimp, nothing plugged in
#
# Needs the repo's .venv with PyQt6 installed (README, "Steam Deck"). Output goes to
# /tmp/ground_station.log, which is where to look when Steam launches it and nothing appears.
cd "$(dirname "$(readlink -f "$0")")/../.." || exit 1
exec .venv/bin/python -m software.ground_station --mode deck "$@" >> /tmp/ground_station.log 2>&1
