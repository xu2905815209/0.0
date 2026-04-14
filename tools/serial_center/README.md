# Serial Center Tools

Run the interactive console with:

```bash
python tools/serial_center/console.py
```

The console keeps `COM13` open, sends single-character commands to the car, captures the resulting serial stream, writes session artifacts under `analysis/serial_center/sessions/`, and generates plots plus a short markdown summary for each session.

