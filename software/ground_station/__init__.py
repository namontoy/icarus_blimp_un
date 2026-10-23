"""Ground station for the blimp: a browser page served by a small Python back end.

The back end (server.py) bridges the UDP packet link and console of blimp_link.py,
the ESP32-CAM's serial protocol (camera.py) and a demo mode to a WebSocket; the
page in web/ draws it all. Design and message formats: docs/ground-station-plan.md.

    python -m software.ground_station --demo      # from the repo root, no hardware
    python -m software.ground_station             # the real blimp at 192.168.4.1
"""
