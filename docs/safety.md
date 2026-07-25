# Safety

This is experimental software for a low-cost educational robot arm, not a certified industrial controller.

- Clear the workspace and remove payloads during initial tests.
- Start near the centered pose with small commands and low speeds.
- Keep physical power accessible.
- Do not depend on software stop as a certified emergency stop.
- Do not force joints during calibration.
- Confirm calibration, direction, TCP, and joint limits on the exact printed assembly.
- Inspect voltage, temperature, current, and status using `soarm101 diagnose`.
