# Pi-side deployment files

`start_ntc_gui.sh` — kiosk launcher of the Pi GUI. Copy of the file used on the bench Pi
(`/home/ntc/start_ntc_gui.sh`, 394 bytes, supplied by the operator 2026-11-28). Contents: it changes
to `/home/ntc/NTC-1A-Control/Python`, sets `SDL_VIDEODRIVER=kmsdrm`, waits 5 s, and runs
`ntc_pygame_ui.py` with `/home/ntc/kivy-venv/bin/python3`, appending stdout/stderr to
`/home/ntc/ntc_gui.log`.

- The **Git copy is a record**; the file on the Pi is what runs. After changing one, update the other.
- How the script is started at boot (autostart entry, `.bash_profile`, systemd unit, ...) is **not
  recorded here** — UNRESOLVED; add it when checked.
- Contains no secrets (no SSID, password or key). Keep it that way.
- To restore on a new Pi: copy it to `/home/ntc/start_ntc_gui.sh` and `chmod +x`.
