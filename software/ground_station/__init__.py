"""Ground station for the blimp: one native PyQt6 window.

The station (station.py) bridges the UDP packet link and console of
blimp_link.py, the camera data (camera.py: a demo until the S3 relays it),
this computer's Wi-Fi (wifi.py) and a demo mode to Qt signals; the window (app.py, panels.py,
widgets.py, theme.py) draws it all. Design: docs/ground-station-native-plan.md,
signal table in docs/ground-station-plan.md §4.

    python -m software.ground_station --demo      # from the repo root, no hardware
    python -m software.ground_station             # the real blimp at 192.168.4.1
"""
