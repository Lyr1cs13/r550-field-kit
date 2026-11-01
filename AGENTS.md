# Development

- Read README.md and docs/ before changing launch or motion behavior.
- Run `python -m unittest discover -s demo/tests` and `python tools/check_repository.py`.
- Keep vehicle parameters, scene geometry and acquisition transport separate.
- Scene/route data always carries map_id. Never replace real pose with goal coordinates.
- Test fixtures use scenes/demo-room, a generated map; never commit personal maps, keys, logs or data.
- Do not start a real robot task unless the user requested that motion. Offline CI must not connect to a robot.
- Preserve vendor driver and license boundaries. Keep a single owner for serial RX and velocity control.
