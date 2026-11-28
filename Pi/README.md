# Pi-side deployment files

`start_ntc_gui.sh` — kiosk launcher of the Pi GUI. Copy of the file used on the bench Pi
(`/home/ntc/start_ntc_gui.sh`, 394 bytes, supplied by the operator 2026-11-28). Contents: it changes
to `/home/ntc/NTC-1A-Control/Python`, sets `SDL_VIDEODRIVER=kmsdrm`, waits 5 s, and runs
`ntc_pygame_ui.py` with `/home/ntc/kivy-venv/bin/python3`, appending stdout/stderr to
`/home/ntc/ntc_gui.log`.

- The **Git copy is a record**; the file on the Pi is what runs. After changing one, update the other.
- **Boot chain (CONFIRMED from the bench Pi, 2026-11-28):**
  1. `/etc/systemd/system/getty@tty1.service.d/override.conf` auto-logs `ntc` in on tty1
     (copy: `getty-tty1-override.conf`).
  2. `~/.bash_profile` runs `/home/ntc/start_ntc_gui.sh` only when not over SSH and the terminal is
     `/dev/tty1` (copy: `bash_profile.snippet`; the real file has only this block). So an ssh login
     never starts a second GUI.
  3. The script `exec`s the GUI (see above). No autostart entry, no systemd unit for the GUI.
  Consequence (DERIVED): the GUI runs in the tty1 login session; stopping that session (logout, reboot)
  stops the GUI, and `exit` of the GUI ends the session, after which getty logs in again and restarts it.
- Contains no secrets (no SSID, password or key). Keep it that way.
- To restore on a new Pi: copy it to `/home/ntc/start_ntc_gui.sh` and `chmod +x`.
